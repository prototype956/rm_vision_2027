/**
 * @file sim_camera.hpp
 * @brief SimCamera: 从 AT 仿真器 TCP 流读取 JPEG 帧并转换为 cv::Mat
 */
#pragma once

#include "i_camera.hpp"

#include <memory>

namespace mv::hal {

/**
 * @brief 基于 TCP 流的仿真相机实现
 *
 * 【职责】
 * - 连接 AT 仿真器 TCP 服务并消费图像流；
 * - 解析协议包，提取 JPEG 负载并解码为 OpenCV BGR 图像。
 *
 * 【配置键语义】（来自 YAML::Node）
 * - endpoint: string，格式 "host:port"，默认 "127.0.0.1:19090"；
 * - connect_timeout_ms: int，连接超时，单位 ms；
 * - recv_timeout_ms: int，接收超时，单位 ms；
 * - reconnect_interval_ms: int，断线重连间隔，单位 ms；
 * - max_payload_bytes: int，单包最大负载，单位 byte。
 *
 * 约定：
 * - 输入协议见 docs/simulator/stream-protocol-spec.md。
 * - Grab() 优先消费 msg_type=0x01 的图像帧；其他消息类型会被跳过。
 * - 输出图像格式固定为 BGR cv::Mat（CV_8UC3）。
 */
class SimCamera : public ICamera {
 public:
  SimCamera();
  ~SimCamera() override;

  SimCamera(const SimCamera&) = delete;
  SimCamera& operator=(const SimCamera&) = delete;
  SimCamera(SimCamera&&) noexcept;
  SimCamera& operator=(SimCamera&&) noexcept;

  /**
   * @brief 打开仿真相机连接并初始化参数
   * @param config YAML 配置节点（见类注释中的配置键）
   * @return true 打开成功（或已打开）
   * @return false 打开失败（格式错误/网络不可达/超时）
   * @thread_safety Not thread-safe
   */
  bool Open(const YAML::Node& config) override;

  /**
   * @brief 关闭连接并释放 socket 资源（幂等）
   * @thread_safety Not thread-safe
   */
  void Close() override;

  /**
   * @brief 读取并解码下一帧图像
   * @param frame 成功时写入 BGR 图像
   * @return true 成功读取并解码
   * @return false 当前无有效图像、超时或连接异常
   * @thread_safety Not thread-safe
   */
  bool Grab(cv::Mat& frame) override;

  /**
   * @brief 查询连接是否处于打开状态
   * @return true 已打开
   * @return false 未打开
   * @thread_safety Not thread-safe
   */
  [[nodiscard]] bool IsOpen() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::hal
