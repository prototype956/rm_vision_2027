/**
 * @file detect_node.hpp
 * @brief 检测+解算节点（Pipeline 第二级）
 *
 * 【职责】
 *   - 从 input_ch_（FramePacket）取帧；
 *   - 调用 IDetector::Detect() 获取 2D 检测结果；
 *   - 对每个 Detection 调用 ISolver::Solve() 完成 PnP 解算；
 *   - 将 DetectPacket 推入 output_ch_；
 *   - enemy_color 与 IMU 姿态来自 Pipeline::SharedState（SerialNode 反向更新）。
 *
 * 【并发设计】
 *   DetectNode 运行在独立线程：
 *   - 与 CaptureNode 解耦（两者速率可独立调整）；
 *   - 若检测耗时 > 帧间隔，Channel 满则丢帧（实时优先策略）；
 *   - 多个 DetectNode 可共享同一 input_ch_（多消费者模式，
 *     用于神经网络推理 GPU 加速场景，目前 1:1 即可）。
 *
 * 【SharedState 注入】
 *   DetectNode 通过构造注入 SharedState&：
 *   - 使用 state.enemy_color 读取敌方颜色；
 *   - 使用 state.gimbal_quat 将 IMU 姿态逐帧注入 ISolver。
 */
#pragma once

#include "channel.hpp"
#include "interfaces/i_detector.hpp"
#include "interfaces/i_solver.hpp"
#include "node.hpp"
#include "packet.hpp"
#include "shared_state.hpp"

#include <memory>

namespace mv::pipeline {

class DetectNode final : public PipelineNode {
 public:
  /**
   * @param detector  检测器（已 Init()）
   * @param solver    PnP 解算器（已 Init()）
   * @param input_ch  输入通道（FramePacket）
   * @param output_ch 输出通道（DetectPacket）
   * @param state     共享状态，由 SerialNode 更新颜色与 IMU 四元数
   */
  DetectNode(std::unique_ptr<IDetector> detector, std::unique_ptr<ISolver> solver,
             std::shared_ptr<Channel<FramePacket>> input_ch,
             std::shared_ptr<Channel<DetectPacket>> output_ch, SharedState& state);

  ~DetectNode() override = default;

  DetectNode(const DetectNode&) = delete;
  DetectNode& operator=(const DetectNode&) = delete;
  DetectNode(DetectNode&&) = delete;
  DetectNode& operator=(DetectNode&&) = delete;

 protected:
  void WorkLoop() override;
  void OnStop() override;

 private:
  std::unique_ptr<IDetector> detector_;
  std::unique_ptr<ISolver> solver_;
  std::shared_ptr<Channel<FramePacket>> input_ch_;
  std::shared_ptr<Channel<DetectPacket>> output_ch_;
  SharedState& state_;
};

}  // namespace mv::pipeline
