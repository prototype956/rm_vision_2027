/**
 * @file predict_node.hpp
 * @brief 预测+投票节点（Pipeline 第三级）
 *
 * 【职责】
 *   - 从 input_ch_（DetectPacket）取数据；
 *   - 调用 IPredictor::Predict() 更新卡尔曼跟踪，输出 GimbalControl；
 *   - 调用 IPredictor::GetTrackTarget() 获取调试状态；
 *   - 调用 IVoter::Vote() 决策是否开火，覆写 control.fire；
 *   - 将 ControlPacket 推入 output_ch_。
 *
 * 【enemy_color 注入】
 *   同 DetectNode，通过 std::atomic<ArmorColor>& 引用获取，
 *   保证 SerialNode 更新后下一帧立刻生效。
 *
 * 【Foxglove 可视化接口（预留）】
 *   ControlPacket 中包含 TrackTarget，后续 FoxgloveNode（可选旁路）
 *   可订阅 output_ch_ 或另开 debug_ch_ 接收诊断数据。
 *
 * 【使用示例】
 * @code
 *   PredictNode pred_node{
 *     std::move(predictor), std::move(voter),
 *     detect_ch, control_ch, color
 *   };
 *   pred_node.Start();
 * @endcode
 */
#pragma once

#include "channel.hpp"
#include "interfaces/i_predictor.hpp"
#include "interfaces/i_voter.hpp"
#include "node.hpp"
#include "packet.hpp"
#include "shared_state.hpp"

#include <memory>

namespace mv::pipeline {

class PredictNode final : public PipelineNode {
 public:
  /**
   * @param predictor   预测器（已 Init()）
   * @param voter       投票器（已 Init()，控制 fire 决策）
   * @param input_ch    输入通道（DetectPacket）
   * @param output_ch   输出通道（ControlPacket）
   * @param state       Pipeline 共享状态（enemy_color + gimbal_quat）
   *
   * 使用 SharedState& 而非单独传 enemy_color：
   *   EkfPredictor 同时需要 gimbal_quat，将两者封装在同一个引用中传入，
   *   避免未来扩展时再改构造函数参数列表。
   */
  PredictNode(std::unique_ptr<IPredictor> predictor, std::unique_ptr<IVoter> voter,
              std::shared_ptr<Channel<DetectPacket>> input_ch,
              std::shared_ptr<Channel<ControlPacket>> output_ch, SharedState& state);

  ~PredictNode() override = default;

  PredictNode(const PredictNode&) = delete;
  PredictNode& operator=(const PredictNode&) = delete;
  PredictNode(PredictNode&&) = delete;
  PredictNode& operator=(PredictNode&&) = delete;

 protected:
  void WorkLoop() override;
  void OnStop() override;

 private:
  std::unique_ptr<IPredictor> predictor_;
  std::unique_ptr<IVoter> voter_;
  std::shared_ptr<Channel<DetectPacket>> input_ch_;
  std::shared_ptr<Channel<ControlPacket>> output_ch_;
  SharedState& state_;  ///< Pipeline 共享状态（enemy_color + gimbal_quat均从此取得）
};

}  // namespace mv::pipeline
