/**
 * @file sim_camera.cpp
 * @brief SimCamera 实现：消费仿真器流并输出 BGR 图像
 */

#include "sim_camera.hpp"

#include "../../core/logger.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <opencv2/imgcodecs.hpp>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace mv::hal {
namespace {

constexpr std::uint8_t kMagic0 = 0x53;  // 'S'
constexpr std::uint8_t kMagic1 = 0x4D;  // 'M'
constexpr std::uint8_t kVersion = 0x01;
constexpr std::uint8_t kMsgImage = 0x01;
// 协议头固定 24 字节：magic(2) + version(1) + type(1) + seq(4) + ts_ns(8) + size(4) + crc32(4)
constexpr std::size_t kHeaderSize = 24;
// 图像元数据固定 8 字节：width(2) + height(2) + channels(1) + encoding(1) + quality(1) +
// reserved(1)
constexpr std::size_t kImageMetaSize = 8;

/// @brief 从小端字节序读取 uint32（协议字段默认小端）
std::uint32_t ReadLeU32(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8U) |
         (static_cast<std::uint32_t>(p[2]) << 16U) | (static_cast<std::uint32_t>(p[3]) << 24U);
}

/// @brief 从小端字节序读取 uint16（用于图像宽高字段）
std::uint16_t ReadLeU16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1]) << 8U);
}

/// @brief 计算 payload 的 CRC32（与 AT 侧保持一致，用于链路完整性校验）
std::uint32_t Crc32(const std::uint8_t* data, std::size_t len) {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      const std::uint32_t mask = static_cast<std::uint32_t>(-(static_cast<int>(crc & 1U)));
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

/// @brief 解析 endpoint 为 host + port
/// @note endpoint 格式固定为 "host:port"，port 范围 [1, 65535]
bool ParseEndpoint(const std::string& endpoint, std::string& host, int& port) {
  const auto pos = endpoint.rfind(':');
  if (pos == std::string::npos || pos == 0 || pos + 1 >= endpoint.size()) {
    return false;
  }

  host = endpoint.substr(0, pos);
  try {
    port = std::stoi(endpoint.substr(pos + 1));
  } catch (...) {
    return false;
  }

  return port > 0 && port <= 65535;
}

/// @brief 切换 socket 阻塞模式
/// @why 连接阶段用非阻塞 + select 做超时控制；连接建立后恢复阻塞读以简化收包逻辑
bool SetNonBlocking(int fd, bool enable) {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  const int new_flags = enable ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
  return fcntl(fd, F_SETFL, new_flags) == 0;
}

/// @brief 精确读取 len 字节，直到收满或错误
/// @note EINTR 可重试；EAGAIN/EWOULDBLOCK 视为本轮失败，由上层决定重连
bool ReadExact(int fd, std::uint8_t* dst, std::size_t len) {
  std::size_t offset = 0;
  while (offset < len) {
    const ssize_t n = recv(fd, dst + offset, len - offset, 0);
    if (n == 0) {
      return false;
    }
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;
      }
      return false;
    }
    offset += static_cast<std::size_t>(n);
  }
  return true;
}

}  // namespace

struct SimCamera::Impl {
  // socket 资源与连接状态
  int sock_fd{-1};
  bool is_open{false};

  // endpoint 解析结果
  std::string endpoint{"127.0.0.1:19090"};
  std::string host{"127.0.0.1"};
  int port{19090};

  // 超时与重连参数（单位：ms）
  int connect_timeout_ms{2000};
  int recv_timeout_ms{500};
  int reconnect_interval_ms{200};
  // 防御性上限（单位：byte），用于限制单包内存分配
  std::size_t max_payload_bytes{8U * 1024U * 1024U};

  // 上次尝试连接的时刻（用于重连节流）
  std::chrono::steady_clock::time_point last_connect_try{};
};

SimCamera::SimCamera() : impl_(std::make_unique<Impl>()) {}

SimCamera::~SimCamera() {
  Close();
}

SimCamera::SimCamera(SimCamera&&) noexcept = default;
SimCamera& SimCamera::operator=(SimCamera&&) noexcept = default;

bool SimCamera::Open(const YAML::Node& config) {
  if (impl_->is_open) {
    return true;
  }

  impl_->endpoint = config["endpoint"].as<std::string>("127.0.0.1:19090");
  impl_->connect_timeout_ms = config["connect_timeout_ms"].as<int>(2000);
  impl_->recv_timeout_ms = config["recv_timeout_ms"].as<int>(500);
  impl_->reconnect_interval_ms = config["reconnect_interval_ms"].as<int>(200);
  impl_->max_payload_bytes =
      static_cast<std::size_t>(config["max_payload_bytes"].as<int>(8 * 1024 * 1024));

  if (!ParseEndpoint(impl_->endpoint, impl_->host, impl_->port)) {
    MV_LOG_ERROR("HAL.Camera.Sim", "invalid endpoint '{}'", impl_->endpoint);
    return false;
  }

  impl_->last_connect_try = std::chrono::steady_clock::time_point{};

  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    MV_LOG_ERROR("HAL.Camera.Sim", "socket() failed: {}", std::strerror(errno));
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(impl_->port));
  if (inet_pton(AF_INET, impl_->host.c_str(), &addr.sin_addr) != 1) {
    MV_LOG_ERROR("HAL.Camera.Sim", "inet_pton failed for host '{}'", impl_->host);
    close(fd);
    return false;
  }

  if (!SetNonBlocking(fd, true)) {
    MV_LOG_WARN("HAL.Camera.Sim", "failed to set nonblocking before connect");
  }

  const int ret = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (ret < 0 && errno != EINPROGRESS) {
    MV_LOG_ERROR("HAL.Camera.Sim", "connect() failed: {}", std::strerror(errno));
    close(fd);
    return false;
  }

  if (ret < 0) {
    // 非阻塞 connect 返回 EINPROGRESS 时，用 select 进行超时等待，避免主线程长时间卡死。
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);

    timeval tv{};
    tv.tv_sec = impl_->connect_timeout_ms / 1000;
    tv.tv_usec = (impl_->connect_timeout_ms % 1000) * 1000;

    const int sel = select(fd + 1, nullptr, &wfds, nullptr, &tv);
    if (sel <= 0) {
      MV_LOG_ERROR("HAL.Camera.Sim", "connect timeout to {}", impl_->endpoint);
      close(fd);
      return false;
    }

    // 仅 select 可写不足以判定成功，仍需通过 SO_ERROR 获取最终连接结果。
    int so_error = 0;
    socklen_t so_len = sizeof(so_error);
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &so_len) != 0 || so_error != 0) {
      MV_LOG_ERROR("HAL.Camera.Sim", "connect SO_ERROR={} ({})", so_error, std::strerror(so_error));
      close(fd);
      return false;
    }
  }

  if (!SetNonBlocking(fd, false)) {
    MV_LOG_WARN("HAL.Camera.Sim", "failed to restore blocking mode");
  }

  timeval rcv_tv{};
  rcv_tv.tv_sec = impl_->recv_timeout_ms / 1000;
  rcv_tv.tv_usec = (impl_->recv_timeout_ms % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof(rcv_tv));

  impl_->sock_fd = fd;
  impl_->is_open = true;
  impl_->last_connect_try = std::chrono::steady_clock::now();

  MV_LOG_INFO("HAL.Camera.Sim", "connected to {}", impl_->endpoint);
  return true;
}

void SimCamera::Close() {
  if (impl_->sock_fd >= 0) {
    close(impl_->sock_fd);
    impl_->sock_fd = -1;
  }
  impl_->is_open = false;
}

bool SimCamera::Grab(cv::Mat& frame) {
  if (!impl_->is_open) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - impl_->last_connect_try)
            .count();
    if (impl_->last_connect_try == std::chrono::steady_clock::time_point{} ||
        elapsed_ms >= impl_->reconnect_interval_ms) {
      // 重连节流：未达到重连间隔直接返回 false，避免高频重试打爆日志和网络栈。
      impl_->last_connect_try = now;
      YAML::Node cfg;
      cfg["endpoint"] = impl_->endpoint;
      cfg["connect_timeout_ms"] = impl_->connect_timeout_ms;
      cfg["recv_timeout_ms"] = impl_->recv_timeout_ms;
      cfg["reconnect_interval_ms"] = impl_->reconnect_interval_ms;
      cfg["max_payload_bytes"] = static_cast<int>(impl_->max_payload_bytes);
      if (!Open(cfg)) {
        return false;
      }
    } else {
      return false;
    }
  }

  // 最多扫描 8 个包：允许跳过姿态类消息与坏包，避免长期阻塞在非图像消息流上。
  for (int i = 0; i < 8; ++i) {
    std::array<std::uint8_t, kHeaderSize> hdr{};
    if (!ReadExact(impl_->sock_fd, hdr.data(), hdr.size())) {
      MV_LOG_WARN("HAL.Camera.Sim", "stream closed while reading header");
      Close();
      return false;
    }

    if (hdr[0] != kMagic0 || hdr[1] != kMagic1 || hdr[2] != kVersion) {
      MV_LOG_WARN("HAL.Camera.Sim", "invalid packet header, reconnecting");
      Close();
      return false;
    }

    const std::uint8_t msg_type = hdr[3];
    const std::uint32_t payload_size = ReadLeU32(hdr.data() + 16);
    const std::uint32_t payload_crc = ReadLeU32(hdr.data() + 20);

    // 防止异常包触发超大内存分配（OOM 风险）。
    if (payload_size > impl_->max_payload_bytes) {
      MV_LOG_WARN("HAL.Camera.Sim", "payload too large: {}", payload_size);
      Close();
      return false;
    }

    std::vector<std::uint8_t> payload(payload_size);
    if (payload_size > 0 && !ReadExact(impl_->sock_fd, payload.data(), payload.size())) {
      MV_LOG_WARN("HAL.Camera.Sim", "stream closed while reading payload");
      Close();
      return false;
    }

    if (Crc32(payload.data(), payload.size()) != payload_crc) {
      MV_LOG_WARN("HAL.Camera.Sim", "crc mismatch, dropping packet");
      continue;
    }

    if (msg_type != kMsgImage) {
      continue;
    }

    if (payload.size() < kImageMetaSize) {
      MV_LOG_WARN("HAL.Camera.Sim", "image payload too short: {}", payload.size());
      continue;
    }

    const std::uint16_t width = ReadLeU16(payload.data());
    const std::uint16_t height = ReadLeU16(payload.data() + 2);
    const std::uint8_t channels = payload[4];
    const std::uint8_t encoding = payload[5];
    (void)payload[6];  // quality
    (void)payload[7];  // reserved

    if (width == 0 || height == 0 || channels != 3 || encoding != 1) {
      MV_LOG_WARN("HAL.Camera.Sim", "invalid image meta w={} h={} c={} enc={}", width, height,
                  channels, encoding);
      continue;
    }

    const std::vector<std::uint8_t> jpeg(payload.begin() + static_cast<long>(kImageMetaSize),
                                         payload.end());

    cv::Mat decoded = cv::imdecode(jpeg, cv::IMREAD_COLOR);
    if (decoded.empty()) {
      MV_LOG_WARN("HAL.Camera.Sim", "jpeg decode failed");
      continue;
    }

    frame = std::move(decoded);
    return true;
  }

  return false;
}

bool SimCamera::IsOpen() const {
  return impl_->is_open;
}

}  // namespace mv::hal
