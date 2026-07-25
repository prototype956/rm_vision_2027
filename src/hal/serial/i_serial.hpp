/**
 * @file i_serial.hpp
 * @brief 串口硬件抽象接口 (ISerial)
 *
 * 【为什么要抽象串口接口？】
 *
 *   原 uart::SerialPort 的问题：
 *   1. 配置读取依赖 cv::FileStorage（OpenCV XML/YAML），与相机模块不必要耦合；
 *   2. 数据协议（帧格式、CRC、数据结构）与底层 I/O 混写在同一个类里，
 *      测试协议解析时必须打开真实串口；
 *   3. 全局变量 kal（Kalman 滤波器）定义在 .hpp 里，多编译单元会 ODR 冲突。
 *
 *   ISerial 将"字节流 I/O"抽象出来：
 *   - 协议层（帧打包/解包/CRC）在 SerialProtocol（后续 Stage 3 实现）中单独测试；
 *   - 传输层（UartSerial）只负责打开端口、收发字节，可独立 mock；
 *   - 上层不感知任何平台 API（termios / Win32 / mock）。
 *
 * 【设计约定】
 *   - Send/Recv 操作是非阻塞友好的：Recv 设置超时，超时返回 false；
 *   - 错误通过返回值 + 日志报告，不抛异常；
 *   - Close() 幂等，析构自动调用。
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <yaml-cpp/yaml.h>

namespace mv::hal {

/**
 * @brief 串口设备的硬件无关抽象接口
 *
 * 具体实现：UartSerial（Linux termios）。
 * 测试时可注入 MockSerial，无需真实硬件。
 */
class ISerial {
 public:
  ISerial() = default;
  virtual ~ISerial() = default;

  // C++ Core Guidelines C.67：多态基类拷贝禁止，移动降为 protected，
  // 防止对象切片，同时保留派生类的移动能力。
  ISerial(const ISerial&) = delete;
  ISerial& operator=(const ISerial&) = delete;

 protected:
  ISerial(ISerial&&) = default;
  ISerial& operator=(ISerial&&) = default;

 public:
  /**
   * @brief 打开串口
   *
   * @param config  YAML 节点，至少需要：
   *                  `device`   (string)  — 设备节点，如 "/dev/ttyUSB0"
   *                  `baudrate` (int)     — 波特率，如 115200 / 921600
   * @return true   串口已就绪
   * @return false  打开失败（设备不存在、权限不足等）
   */
  virtual bool Open(const YAML::Node& config) = 0;

  /**
   * @brief 关闭串口并释放文件描述符
   *
   * 幂等，析构时自动调用。
   */
  virtual void Close() = 0;

  /**
   * @brief 发送字节流
   *
   * @param data  待发送数据的起始地址
   * @param len   待发送字节数
   * @return true   全部字节已写入内核发送缓冲区
   * @return false  写入失败（串口断开等）
   */
  virtual bool Send(const uint8_t* data, std::size_t len) = 0;

  /**
   * @brief 接收字节流（带超时）
   *
   * 尽量读取 `len` 字节，最多等待约 1 帧周期（实现内部决定超时值）。
   * 调用方应循环调用直到凑够一帧，或交由 SerialProtocol 负责成帧。
   *
   * @param[out] buf        接收缓冲区（调用方分配，大小 >= len）
   * @param      len        期望接收的字节数
   * @param[out] received   实际接收到的字节数（可能 < len）
   * @return true   至少接收到 1 字节
   * @return false  超时或错误（received == 0）
   */
  virtual bool Recv(uint8_t* buf, std::size_t len, std::size_t& received) = 0;

  /**
   * @brief 查询串口是否已打开
   */
  [[nodiscard]] virtual bool IsOpen() const = 0;
};

}  // namespace mv::hal
