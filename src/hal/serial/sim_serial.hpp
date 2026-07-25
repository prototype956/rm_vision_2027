/**
 * @file sim_serial.hpp
 * @brief SimSerial: 基于 TCP 的仿真串口实现（用于 at_vision_simulator 联调）
 */
#pragma once

#include "i_serial.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace mv::hal {

/**
 * @brief 仿真串口后端（TCP）
 *
 * 【设计目标】
 * - 对上层保持 ISerial 契约，串口节点无需感知仿真与真机差异；
 * - 断连时软降级：Send() 返回成功，避免 SerialNode 连续计错将 FSM 拉入 ERROR；
 * - 后台按间隔自动重连，恢复后透明继续收发。
 *
 * 【配置键】
 * - endpoint: string, 形如 "127.0.0.1:19091"
 * - connect_timeout_ms: int, 连接超时（ms）
 * - recv_timeout_ms: int, 接收超时（ms）
 * - reconnect_interval_ms: int, 重连间隔（ms）
 */
class SimSerial : public ISerial {
 public:
  SimSerial();
  ~SimSerial() override;

  SimSerial(const SimSerial&) = delete;
  SimSerial& operator=(const SimSerial&) = delete;
  SimSerial(SimSerial&&) noexcept;
  SimSerial& operator=(SimSerial&&) noexcept;

  /**
   * @brief 打开仿真串口并初始化连接参数
   * @param config YAML 配置节点
   * @return true 配置成功并进入可用状态（即使当前未连上远端）
   * @return false 配置严重错误（如参数非法）
   * @thread_safety Not thread-safe
   */
  bool Open(const YAML::Node& config) override;

  /**
   * @brief 关闭连接并重置打开状态（幂等）
   * @thread_safety Not thread-safe
   */
  void Close() override;

  /**
   * @brief 发送字节流到仿真端
   *
   * 断连场景返回 true（软降级），并由内部自动重连机制在后续尝试恢复链路。
   *
   * @param data 待发送数据首地址
   * @param len  待发送字节数
   * @return true 发送成功或软降级吞吐成功
   * @return false 串口未打开或输入参数非法
   * @thread_safety Not thread-safe
   */
  bool Send(const uint8_t* data, std::size_t len) override;

  /**
   * @brief 从仿真端接收字节流
   * @param buf 接收缓冲区
   * @param len 期望接收字节数
   * @param received 实际接收字节数
   * @return true 接收到至少 1 字节
   * @return false 当前无数据或连接异常
   * @thread_safety Not thread-safe
   */
  bool Recv(uint8_t* buf, std::size_t len, std::size_t& received) override;

  /**
   * @brief 查询逻辑打开状态
   * @return true Open() 成功后为真，直到 Close()
   * @thread_safety Not thread-safe
   */
  [[nodiscard]] bool IsOpen() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::hal
