/**
 * @file predict_node.cpp
 * @brief PredictNode 工作线程实现
 */
#include "predict_node.hpp"

#include "core/logger.hpp"

namespace mv::pipeline {

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static constexpr auto POP_TIMEOUT = std::chrono::milliseconds{10};

PredictNode::PredictNode(std::unique_ptr<IPredictor> predictor, std::unique_ptr<IVoter> voter,
                         std::shared_ptr<Channel<DetectPacket>> input_ch,
                         std::shared_ptr<Channel<ControlPacket>> output_ch, SharedState& state)
    : PipelineNode("PredictNode"),
      predictor_(std::move(predictor)),
      voter_(std::move(voter)),
      input_ch_(std::move(input_ch)),
      output_ch_(std::move(output_ch)),
      state_(state) {}

void PredictNode::OnStop() {
  if (input_ch_) {
    input_ch_->Shutdown();
  }
  if (output_ch_) {
    output_ch_->Shutdown();
  }
}

void PredictNode::WorkLoop() {
  MV_LOG_INFO("PredictNode", "Worker started.");

  while (!ShouldStop()) {
    DetectPacket det_pkt;
    if (!input_ch_->Pop(det_pkt, POP_TIMEOUT)) {
      continue;
    }

    const ArmorColor COLOR = state_.enemy_color.load();

    // ── 注入云台姿态（IMU 四元数 → EKF 坐标系变换）──────────────────────
    // EkfPredictor 每帧读取最新四元数，SimplePredictor 的默认实现为空操作。
    predictor_->SetGimbalOrientation(state_.GetGimbalQuat());

    // ── 预测（EKF 跟踪 + 云台角度输出）──────────────────────────────────
    GimbalControl control = predictor_->Predict(det_pkt.detections, det_pkt.timestamp, COLOR);

    // ── 跟踪状态快照（供 Voter 决策 + Foxglove 可视化）────────────────
    const TrackTarget TARGET = predictor_->GetTrackTarget();

    // ── 开火决策（Voter 签字覆写 fire 字段）──────────────────────────
    control.fire = voter_->Vote(TARGET, control);

    ControlPacket out_pkt{
        .control = control,
        .track_target = TARGET,
        .timestamp = det_pkt.timestamp,
        .frame_id = det_pkt.frame_id,
    };

    if (!output_ch_->Push(std::move(out_pkt))) {
      break;  // 通道关闭，退出
    }

    IncrementProcessed();
  }

  MV_LOG_INFO("PredictNode", "Worker stopped. Processed {} packets.", ProcessedCount());
}

}  // namespace mv::pipeline
