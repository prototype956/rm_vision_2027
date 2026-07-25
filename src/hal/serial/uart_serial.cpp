/**
 * @file uart_serial.cpp
 * @brief Linux UART 串口 Pimpl 实现（termios）
 *
 * 【termios 参数配置说明】
 *
 *   我们用"原始模式"（raw mode）：
 *   - c_lflag 不设置 ECHO / ICANON / ISIG：禁止回显、行缓冲、信号字符
 *   - c_iflag 不设置 IXON / ICRNL 等：禁止软流控、CR→LF 转换
 *   - c_cflag 设置 CS8 | CLOCAL | CREAD：8位数据、忽略调制解调器状态、允许读
 *   - VTIME=0, VMIN=0：read() 立即返回（非阻塞模式）
 *
 *   之所以用非阻塞 read 而不是阻塞等待：
 *   Recv() 调用方（Pipeline 线程）需要定期检查退出标志，
 *   如果 read() 永远阻塞，线程无法干净退出。
 *   上层用 Recv 超时逻辑来凑齐一帧，而不是依赖 termios VMIN 阻塞。
 *
 * 【波特率映射】
 *   termios 只接受 Bxxx 宏，不接受整数。
 *   我们在 Open() 里做一次映射，未知波特率打印警告并降级到 B115200。
 *
 * 【设备遍历逻辑】
 *   先尝试 config["device"]，再依次尝试 config["fallback_devices"] 列表。
 *   这样比原代码硬编码 /dev/ttyUSB0..3 / ttyACM0..1 更灵活，
 *   且设备路径完全由配置文件控制，不需要重编代码。
 */

#include "uart_serial.hpp"

#include "../../core/logger.hpp"

#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace mv::hal {

// ============================================================================
// 内部工具函数
// ============================================================================

namespace {

/**
 * @brief 将整数波特率转换为 termios speed_t 宏
 *
 * 只支持机器人常用的几档，未知值降级到 B115200 并打印警告。
 * termios 只接受 Bxxx 宏，不能直接传整数，需要这层映射。
 */
speed_t BaudrateToTermios(int baudrate) {
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      MV_LOG_WARN("HAL.Serial.UART", "unsupported baudrate {}, falling back to 115200", baudrate);
      return B115200;
  }
}

/**
 * @brief 打开单个设备节点，失败时返回 -1
 */
int OpenDevice(const std::string& path) {
  // O_NOCTTY：不让串口成为进程的控制终端（避免串口发来的特殊字符杀死进程）
  // O_NDELAY：非阻塞 open（设备离线时不挂起）
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) — POSIX open() 是 C vararg，无法替换
  const int RAW_FD = open(path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (RAW_FD == -1) {
    MV_LOG_WARN("HAL.Serial.UART", "cannot open '{}': {}", path, strerror(errno));
  } else {
    MV_LOG_INFO("HAL.Serial.UART", "opened '{}'", path);
  }
  return RAW_FD;
}

}  // namespace

// ============================================================================
// Impl 定义
// ============================================================================

struct UartSerial::Impl {
  int fd{-1};
  bool is_open{false};
};

// ============================================================================
// 构造 / 析构 / 移动
// ============================================================================

UartSerial::UartSerial() : impl_(std::make_unique<Impl>()) {}

UartSerial::~UartSerial() {
  Close();
}

UartSerial::UartSerial(UartSerial&&) noexcept = default;
UartSerial& UartSerial::operator=(UartSerial&&) noexcept = default;

// ============================================================================
// ISerial 接口实现
// ============================================================================

bool UartSerial::Open(const YAML::Node& config) {
  if (impl_->is_open) {
    return true;
  }

  // 收集要尝试的设备路径列表
  std::vector<std::string> devices;
  if (config["device"]) {
    devices.push_back(config["device"].as<std::string>());
  }
  if (config["fallback_devices"] && config["fallback_devices"].IsSequence()) {
    for (const auto& node : config["fallback_devices"]) {
      devices.push_back(node.as<std::string>());
    }
  }
  if (devices.empty()) {
    MV_LOG_ERROR("HAL.Serial.UART", "no device paths configured");
    return false;
  }

  // 依次尝试打开
  int port_fd = -1;
  for (const auto& path : devices) {
    port_fd = OpenDevice(path);
    if (port_fd != -1) {
      break;
    }
  }
  if (port_fd == -1) {
    MV_LOG_ERROR("HAL.Serial.UART", "all device paths failed");
    return false;
  }

  // 配置 termios（原始模式，非阻塞读）
  struct termios tio {};
  if (tcgetattr(port_fd, &tio) != 0) {
    MV_LOG_ERROR("HAL.Serial.UART", "tcgetattr failed: {}", strerror(errno));
    close(port_fd);
    return false;
  }

  const int BAUDRATE = config["baudrate"].as<int>(115200);
  const speed_t BAUD_SPEED = BaudrateToTermios(BAUDRATE);
  cfsetispeed(&tio, BAUD_SPEED);
  cfsetospeed(&tio, BAUD_SPEED);

  // 原始模式：禁用所有处理
  cfmakeraw(&tio);

  // 确保 CLOCAL（忽略调制解调器状态）和 CREAD（启用接收）
  tio.c_cflag |= (CLOCAL | CREAD);

  // 非阻塞 read：VTIME=0, VMIN=0 → read() 立即返回（可能读到 0 字节）
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN] = 0;

  tcflush(port_fd, TCIOFLUSH);
  if (tcsetattr(port_fd, TCSANOW, &tio) != 0) {
    MV_LOG_ERROR("HAL.Serial.UART", "tcsetattr failed: {}", strerror(errno));
    close(port_fd);
    return false;
  }

  impl_->fd = port_fd;
  impl_->is_open = true;
  MV_LOG_INFO("HAL.Serial.UART", "configured at {} baud", BAUDRATE);
  return true;
}

void UartSerial::Close() {
  if (!impl_->is_open) {
    return;  // 幂等
  }
  close(impl_->fd);
  impl_->fd = -1;
  impl_->is_open = false;
  MV_LOG_INFO("HAL.Serial.UART", "closed");
}

bool UartSerial::Send(const uint8_t* data, std::size_t len) {
  if (!impl_->is_open) {
    MV_LOG_WARN("HAL.Serial.UART", "Send called on closed port");
    return false;
  }

  std::size_t total_written = 0;
  while (total_written < len) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const ssize_t BYTES_WRITTEN = write(impl_->fd, data + total_written, len - total_written);
    if (BYTES_WRITTEN <= 0) {
      MV_LOG_ERROR("HAL.Serial.UART", "write error: {}", strerror(errno));
      return false;
    }
    total_written += static_cast<std::size_t>(BYTES_WRITTEN);
  }
  return true;
}

bool UartSerial::Recv(uint8_t* buf, std::size_t len, std::size_t& received) {
  if (!impl_->is_open) {
    received = 0;
    return false;
  }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) — POSIX read() 是 C vararg，无法替换
  const ssize_t BYTES_READ = read(impl_->fd, buf, len);
  if (BYTES_READ < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // 非阻塞模式：当前没有数据，不算错误
      received = 0;
      return false;
    }
    MV_LOG_ERROR("HAL.Serial.UART", "read error: {}", strerror(errno));
    received = 0;
    return false;
  }

  received = static_cast<std::size_t>(BYTES_READ);
  return received > 0;
}

bool UartSerial::IsOpen() const {
  return impl_->is_open;
}

}  // namespace mv::hal
