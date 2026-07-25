/**
 * @file uart_serial.hpp
 * @brief Linux UART 串口的 Pimpl 封装
 *
 * 【与原 uart::SerialPort 的主要区别】
 *
 *   原实现的问题：
 *   1. 协议帧格式（帧头 0x53 / 帧尾 0x45、CRC8、Receive_Data/Write_Data）
 *      和底层 I/O（termios、read/write）混写在同一个类里，无法独立测试；
 *   2. 全局变量 `kal` 定义在 .hpp 中，造成 ODR 违反（每个包含此头文件
 *      的翻译单元都会生成一个副本）；
 *   3. 配置依赖 cv::FileStorage，引入不必要的 OpenCV 依赖。
 *
 *   重构后：
 *   - UartSerial 只负责"打开端口、收发字节流"，不知道任何帧格式；
 *   - 协议层（帧打包/解包/CRC/Kalman 滤波）将在 Stage 3 的
 *     SerialProtocol 模块中单独实现和测试；
 *   - termios 细节完全隐藏在 .cpp 里，头文件只依赖标准库和 yaml-cpp。
 *
 * 【YAML 配置字段】
 * @code
 *   serial:
 *     device:    "/dev/ttyUSB0"   # 首选设备节点
 *     baudrate:  115200           # 115200 / 921600
 *     fallback_devices:           # 可选：依次尝试的备选端口
 *       - "/dev/ttyUSB1"
 *       - "/dev/ttyACM0"
 * @endcode
 */
#pragma once

#include "i_serial.hpp"

#include <memory>

namespace mv::hal {

/**
 * @brief Linux UART 串口实现（Pimpl 模式）
 *
 * 封装 termios API，隐藏所有平台细节。
 * 上层只通过 ISerial 接口交互，移植到其他平台时只需替换此类。
 */
class UartSerial : public ISerial {
 public:
  UartSerial();
  ~UartSerial() override;

  UartSerial(const UartSerial&) = delete;
  UartSerial& operator=(const UartSerial&) = delete;
  UartSerial(UartSerial&&) noexcept;
  UartSerial& operator=(UartSerial&&) noexcept;

  bool Open(const YAML::Node& config) override;
  void Close() override;
  bool Send(const uint8_t* data, std::size_t len) override;
  bool Recv(uint8_t* buf, std::size_t len, std::size_t& received) override;
  [[nodiscard]] bool IsOpen() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::hal
