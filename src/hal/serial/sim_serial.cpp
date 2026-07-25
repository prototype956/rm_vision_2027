/**
 * @file sim_serial.cpp
 * @brief SimSerial 实现：通过 TCP 与仿真器交换串口字节流
 */

#include "sim_serial.hpp"

#include "core/logger.hpp"

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace mv::hal {
namespace {

bool ParseEndpoint(const std::string& endpoint, std::string& host, int& port) {
  const auto POS = endpoint.rfind(':');
  if (POS == std::string::npos || POS == 0 || POS + 1 >= endpoint.size()) {
    return false;
  }

  host = endpoint.substr(0, POS);
  try {
    port = std::stoi(endpoint.substr(POS + 1));
  } catch (...) {
    return false;
  }

  return port > 0 && port <= 65535;
}

bool SetNonBlocking(int socket_fd, bool enable) {
  const int FLAGS = fcntl(socket_fd, F_GETFL, 0);
  if (FLAGS < 0) {
    return false;
  }
  const int NEW_FLAGS = enable ? (FLAGS | O_NONBLOCK) : (FLAGS & ~O_NONBLOCK);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) -- POSIX fcntl is C vararg API.
  return fcntl(socket_fd, F_SETFL, NEW_FLAGS) == 0;
}

}  // namespace

struct SimSerial::Impl {
  int sock_fd{-1};
  bool logical_open{false};

  std::string endpoint{"127.0.0.1:19091"};
  std::string host{"127.0.0.1"};
  int port{19091};

  int connect_timeout_ms{300};
  int recv_timeout_ms{100};
  int reconnect_interval_ms{100};

  std::chrono::steady_clock::time_point last_reconnect_try{};
  std::chrono::steady_clock::time_point last_warn_log{};

  void CloseSocket() {
    if (sock_fd >= 0) {
      close(sock_fd);
      sock_fd = -1;
    }
  }

  [[nodiscard]] bool Connected() const { return sock_fd >= 0; }

  void MaybeLogDisconnectWarn(const char* message) {
    const auto NOW = std::chrono::steady_clock::now();
    if (last_warn_log == std::chrono::steady_clock::time_point{} ||
        std::chrono::duration_cast<std::chrono::milliseconds>(NOW - last_warn_log).count() >=
            1000) {
      MV_LOG_WARN("HAL.Serial.Sim", "{}", message);
      last_warn_log = NOW;
    }
  }

  bool ConnectNow() {
    CloseSocket();

    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
      MaybeLogDisconnectWarn("socket() failed while reconnecting");
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
      MV_LOG_ERROR("HAL.Serial.Sim", "inet_pton failed for host '{}'", host);
      close(socket_fd);
      return false;
    }

    if (!SetNonBlocking(socket_fd, true)) {
      MV_LOG_WARN("HAL.Serial.Sim", "set nonblocking before connect failed");
    }

    const int RET = connect(socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (RET < 0 && errno != EINPROGRESS) {
      close(socket_fd);
      MaybeLogDisconnectWarn("connect() failed while reconnecting");
      return false;
    }

    if (RET < 0) {
      pollfd poll_fd{};
      poll_fd.fd = socket_fd;
      poll_fd.events = POLLOUT;

      const int SEL = poll(&poll_fd, 1, connect_timeout_ms);
      if (SEL <= 0) {
        close(socket_fd);
        MaybeLogDisconnectWarn("connect timeout while reconnecting");
        return false;
      }

      int so_error = 0;
      socklen_t so_len = sizeof(so_error);
      if (getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, &so_error, &so_len) != 0 || so_error != 0) {
        close(socket_fd);
        MaybeLogDisconnectWarn("connect SO_ERROR while reconnecting");
        return false;
      }
    }

    if (!SetNonBlocking(socket_fd, false)) {
      MV_LOG_WARN("HAL.Serial.Sim", "restore blocking mode failed");
    }

    timeval rcv_tv{};
    rcv_tv.tv_sec = recv_timeout_ms / 1000;
    rcv_tv.tv_usec = static_cast<decltype(rcv_tv.tv_usec)>((recv_timeout_ms % 1000) * 1000);
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof(rcv_tv));

    sock_fd = socket_fd;
    MV_LOG_INFO("HAL.Serial.Sim", "connected to {}", endpoint);
    return true;
  }

  void MaybeReconnect() {
    if (Connected()) {
      return;
    }

    const auto NOW = std::chrono::steady_clock::now();
    if (last_reconnect_try != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration_cast<std::chrono::milliseconds>(NOW - last_reconnect_try).count() <
            reconnect_interval_ms) {
      return;
    }

    last_reconnect_try = NOW;
    (void)ConnectNow();
  }
};

SimSerial::SimSerial() : impl_(std::make_unique<Impl>()) {}

SimSerial::~SimSerial() {
  Close();
}

SimSerial::SimSerial(SimSerial&&) noexcept = default;
SimSerial& SimSerial::operator=(SimSerial&&) noexcept = default;

bool SimSerial::Open(const YAML::Node& config) {
  if (impl_->logical_open) {
    return true;
  }

  impl_->endpoint = config["endpoint"].as<std::string>("127.0.0.1:19091");
  impl_->connect_timeout_ms = config["connect_timeout_ms"].as<int>(300);
  impl_->recv_timeout_ms = config["recv_timeout_ms"].as<int>(100);
  impl_->reconnect_interval_ms = config["reconnect_interval_ms"].as<int>(100);

  if (impl_->connect_timeout_ms < 0 || impl_->recv_timeout_ms < 0 ||
      impl_->reconnect_interval_ms < 0) {
    MV_LOG_ERROR("HAL.Serial.Sim", "timeouts and reconnect interval must be non-negative");
    return false;
  }

  if (!ParseEndpoint(impl_->endpoint, impl_->host, impl_->port)) {
    MV_LOG_ERROR("HAL.Serial.Sim", "invalid endpoint '{}', expected host:port", impl_->endpoint);
    return false;
  }

  impl_->logical_open = true;
  impl_->last_reconnect_try = std::chrono::steady_clock::time_point{};
  impl_->MaybeReconnect();
  if (!impl_->Connected()) {
    MV_LOG_WARN("HAL.Serial.Sim",
                "initial connect to {} failed, entering soft-degrade reconnect mode",
                impl_->endpoint);
  }
  return true;
}

void SimSerial::Close() {
  impl_->logical_open = false;
  impl_->CloseSocket();
}

bool SimSerial::Send(const uint8_t* data, std::size_t len) {
  if (!impl_->logical_open) {
    return false;
  }
  if (data == nullptr || len == 0) {
    return false;
  }

  impl_->MaybeReconnect();
  if (!impl_->Connected()) {
    return true;
  }

  std::size_t total_written = 0;
  while (total_written < len) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const ssize_t WRITTEN = send(impl_->sock_fd, data + total_written, len - total_written, 0);
    if (WRITTEN <= 0) {
      impl_->CloseSocket();
      impl_->MaybeLogDisconnectWarn("send failed, switch to soft-degrade mode and reconnect later");
      return true;
    }
    total_written += static_cast<std::size_t>(WRITTEN);
  }

  return true;
}

bool SimSerial::Recv(uint8_t* buf, std::size_t len, std::size_t& received) {
  received = 0;
  if (!impl_->logical_open) {
    return false;
  }
  if (buf == nullptr || len == 0) {
    return false;
  }

  impl_->MaybeReconnect();
  if (!impl_->Connected()) {
    return false;
  }

  const ssize_t READ_SIZE = recv(impl_->sock_fd, buf, len, 0);
  if (READ_SIZE > 0) {
    received = static_cast<std::size_t>(READ_SIZE);
    return true;
  }

  if (READ_SIZE == 0) {
    impl_->CloseSocket();
    impl_->MaybeLogDisconnectWarn("remote closed sim serial connection");
    return false;
  }

  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    return false;
  }

  impl_->CloseSocket();
  impl_->MaybeLogDisconnectWarn("recv failed, switch to soft-degrade mode and reconnect later");
  return false;
}

bool SimSerial::IsOpen() const {
  return impl_->logical_open;
}

}  // namespace mv::hal
