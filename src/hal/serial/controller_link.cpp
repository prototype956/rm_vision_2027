#include "hal/serial/controller_link.hpp"

#include "hal/serial/attitude_wire.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <deque>
#include <random>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

namespace mv::hal::serial {
namespace {
using Clock = std::chrono::steady_clock;
double NowUs() {
  return std::chrono::duration<double, std::micro>(Clock::now().time_since_epoch()).count();
}
}  // namespace

struct ControllerLink::Impl {
  struct Sync {
    double host_us, offset_us, rtt_us;
  };
  ControllerLinkConfig config;
  UpdateCallback callback;
  ControllerLinkState health;
  std::deque<Sync> syncs;
  std::jthread worker;
  int fd{-1};
  std::uint64_t session{0}, pending_t1{0};
  std::uint32_t request{0}, pending_request{0};
  bool acknowledged{false};
  double last_rx_us{0}, last_sync_us{0}, rate_start{0}, input_last_us{0};
  std::uint64_t rate_count{0};
  std::vector<std::uint8_t> input;

  Impl(ControllerLinkConfig c, UpdateCallback update)
      : config(std::move(c)), callback(std::move(update)) {
    if (!callback || config.baud != 460800 || config.startup_sync_ms < 20 ||
        config.sync_ms < config.startup_sync_ms || config.sync_ms > 1000 ||
        config.timeout_ms < 3 * config.sync_ms)
      throw std::invalid_argument("invalid controller link configuration or callback");
    if (!config.enabled || config.device.empty()) {
      health.reason = config.enabled ? "serial_device_not_configured" : "disabled";
      (void)callback(health, std::nullopt);
      return;
    }
    worker = std::jthread([this](std::stop_token stop) { Run(stop); });
  }
  ~Impl() {
    worker.request_stop();
    if (worker.joinable()) worker.join();
    if (fd >= 0) close(fd);
  }
  // 仅收发线程调用；作废待完成请求并通知消费方清空历史样本。
  void Reset(const char* reason) {
    syncs.clear(); input.clear();
    acknowledged = false; pending_t1 = 0;
    health.synchronized = false;
    health.rtt_us = -1; health.offset_us = 0; health.reason = reason;
    ++health.generation;
    session = (static_cast<std::uint64_t>(std::random_device{}()) << 32U) ^
              static_cast<std::uint64_t>(NowUs());
    if (session == 0) session = 1;
    last_rx_us = NowUs(); last_sync_us = 0;
    rate_start = NowUs(); rate_count = 0; health.rate_hz = 0;
    (void)callback(health, std::nullopt);
  }
  bool Open() {
    fd = open(config.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return false;
    termios attrs{};
    if (flock(fd, LOCK_EX | LOCK_NB) != 0 || tcgetattr(fd, &attrs) != 0) {
      close(fd); fd = -1; return false;
    }
    cfmakeraw(&attrs);
    attrs.c_cflag = (attrs.c_cflag & ~(CSIZE | CSTOPB | PARENB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
    cfsetispeed(&attrs, B460800); cfsetospeed(&attrs, B460800);
    if (tcsetattr(fd, TCSANOW, &attrs) != 0) { close(fd); fd = -1; return false; }
    tcflush(fd, TCIOFLUSH);
    return true;
  }
  // 仅收发线程写串口，在限定时间内处理部分写入；t1 在首次写入前记录。
  bool Send(std::uint8_t type, std::uint64_t t1) {
    std::array<std::uint8_t, AV_MAX_FRAME> bytes{};
    pending_request = ++request;
    const auto SIZE = AvBegin(bytes.data(), type, type == AV_SYNC ? 8 : 0,
                              pending_request, session);
    if (type == AV_SYNC) AvWrite(bytes.data() + AV_HEADER, t1, 8);
    AvFinish(bytes.data(), SIZE);
    std::size_t sent = 0;
    const double DEADLINE = NowUs() + 20000;
    while (sent < SIZE && NowUs() < DEADLINE) {
      const auto N = write(fd, bytes.data() + sent, SIZE - sent);
      if (N > 0) sent += static_cast<std::size_t>(N);
      else if (N < 0 && errno != EAGAIN && errno != EINTR) return false;
      else { pollfd p{fd, POLLOUT, 0}; poll(&p, 1, 2); }
    }
    return sent == SIZE;
  }
  // 仅收发线程调用；先处理会话及同步，将其他合法帧交给消费方。
  void Dispatch(const std::vector<std::uint8_t>& f, double t4) {
    if (AvRead(f.data() + 10, 8) != session) return;
    const auto TYPE = f[3];
    const auto LEN = AvRead(f.data() + 4, 2);
    const auto SEQ = static_cast<std::uint32_t>(AvRead(f.data() + 6, 4));
    const auto* p = f.data() + AV_HEADER;
    if (TYPE == AV_ACK && LEN == 0 && SEQ == pending_request) {
      acknowledged = true; last_rx_us = t4; last_sync_us = 0;
      health.reason = "synchronizing";
    } else if (TYPE == AV_SYNC_REPLY && LEN == 24 && acknowledged &&
               SEQ == pending_request && pending_t1 != 0 && AvRead(p, 8) == pending_t1) {
      const double T1 = static_cast<double>(pending_t1);
      const double T2 = static_cast<double>(AvRead(p + 8, 8));
      const double T3 = static_cast<double>(AvRead(p + 16, 8));
      pending_t1 = 0;
      const double RTT = (t4 - T1) - (T3 - T2);
      if (T3 < T2 || RTT < 0 || RTT > 20000) return;
      const double OFFSET = ((T1 - T2) + (t4 - T3)) * 0.5;
      if (health.synchronized && std::abs(OFFSET - health.offset_us) > 20000) {
        Reset("clock_discontinuity"); return;
      }
      last_rx_us = t4;
      syncs.push_back({t4, OFFSET, RTT});
      while (!syncs.empty() && (syncs.size() > 32 || t4 - syncs.front().host_us > 10000000))
        syncs.pop_front();
      const auto BEST = std::min_element(syncs.begin(), syncs.end(),
          [](const Sync& a, const Sync& b) { return a.rtt_us < b.rtt_us; });
      health.offset_us = BEST->offset_us; health.rtt_us = BEST->rtt_us;
      health.synchronized = syncs.size() >= 3;
      if (health.synchronized) health.reason = "awaiting_attitude";
    } else if (acknowledged) {
      // 此层只识别消息信封，不解释姿态载荷或依赖 Eigen。
      if (TYPE == AV_ATTITUDE && LEN == 42) {
        last_rx_us = t4;
        ++rate_count;
        if (t4 - rate_start >= 1000000) {
          health.rate_hz = static_cast<double>(rate_count) * 1e6 / (t4 - rate_start);
          rate_start = t4;
          rate_count = 0;
        }
      }
      ControllerFrame frame{.type = TYPE, .sequence = SEQ,
                            .payload = std::vector<std::uint8_t>(p, p + LEN),
                            .receive_time_us = t4};
      if (!callback(health, frame)) Reset("mcu_restart_or_counter_reset");
    }
  }

  void Run(std::stop_token stop) {
    double retry_at = 0;
    while (!stop.stop_requested()) {
      if (fd < 0) {
        if (NowUs() < retry_at) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue;
        }
        const bool OK = Open();
        health.connected = OK;
        Reset(OK ? "handshake" : "serial_open_failed");
        if (!OK) { retry_at = NowUs() + 1000000; continue; }
      }
      bool io_ok = true;
      {
        const double NOW = NowUs();
        if (NOW - last_rx_us > config.timeout_ms * 1000.0 ||
            (health.synchronized && !syncs.empty() &&
             NOW - syncs.back().host_us > config.timeout_ms * 1000.0)) Reset("link_timeout");
        const int PERIOD = health.synchronized ? config.sync_ms : config.startup_sync_ms;
        if (NOW - last_sync_us >= PERIOD * 1000.0) {
          pending_t1 = acknowledged ? static_cast<std::uint64_t>(NowUs()) : 0;
          io_ok = Send(acknowledged ? AV_SYNC : AV_HELLO, pending_t1);
          last_sync_us = NOW;
        }
      }
      pollfd p{fd, POLLIN, 0};
      const int READY = poll(&p, 1, 5);
      if (READY < 0 && errno != EINTR) io_ok = false;
      if ((p.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) io_ok = false;
      if (io_ok && (p.revents & POLLIN) != 0) {
        std::array<std::uint8_t, 512> bytes{};
        const auto N = read(fd, bytes.data(), bytes.size());
        const double T4 = NowUs();
        if (N > 0) {
          if (T4 - input_last_us > 100000.0) input.clear();
          input_last_us = T4;
          input.insert(input.end(), bytes.begin(), bytes.begin() + N);
          while (input.size() >= 6) {
            const auto LEN = AvRead(input.data() + 4, 2);
            if (input[0] != 0xa6 || input[1] != 0x5a || input[2] != 1 || LEN > AV_MAX_FRAME - AV_HEADER - 2) {
              input.erase(input.begin()); continue;
            }
            const auto SIZE = static_cast<std::size_t>(AV_HEADER + LEN + 2);
            if (input.size() < SIZE) break;
            if (AvRead(input.data() + SIZE - 2, 2) != AvCrc(input.data(), static_cast<unsigned>(SIZE - 2))) {
              ++health.crc_errors; input.erase(input.begin()); continue;
            }
            std::vector<std::uint8_t> frame(input.begin(), input.begin() + SIZE);
            input.erase(input.begin(), input.begin() + SIZE);
            Dispatch(frame, T4);
          }
        } else if (N == 0 || (errno != EAGAIN && errno != EINTR)) io_ok = false;
      }
      if (!io_ok) {
        close(fd); fd = -1;
        health.connected = false; Reset("serial_disconnected");
        retry_at = NowUs() + 1000000;
      }
      (void)callback(health, std::nullopt);
    }
  }
};

ControllerLink::ControllerLink(ControllerLinkConfig config, UpdateCallback callback)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(callback))) {}
ControllerLink::~ControllerLink() = default;

}  // namespace mv::hal::serial
