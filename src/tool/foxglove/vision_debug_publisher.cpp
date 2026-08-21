#include "tool/foxglove/vision_debug_publisher.hpp"

#include "tool/foxglove/control/control_debug_publisher.hpp"
#include "tool/foxglove/pipeline/vision_debug_pipeline.hpp"
#include "tool/foxglove/runtime/foxglove_session.hpp"

#include <atomic>
#include <mutex>
#include <utility>

#include <optional>

namespace mv::tool::foxglove {

struct VisionDebugPublisher::Impl {
  explicit Impl(const Config& config)
      : session(config), pipeline(config, session), control(config, session) {
    // pipeline 构造期间先创建并注册全部频道，会话启动时才能一次性暴露完整话题集合。
    session.Start();
    pipeline.Start();
    control.Start();
  }

  ~Impl() { Stop(); }

  void Stop() noexcept {
    if (stopped.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    // 先停止生产者、排空队列并关闭频道，再让会话收尾 MCAP 和 WebSocket。
    control.Stop();
    pipeline.Stop();
    session.Stop();
  }

  modules::ArmorSelectionSnapshot SelectionSnapshot() const noexcept {
    std::lock_guard lock(selection_mutex);
    return latest_selection;
  }

  void UpdateSelection(const modules::FireControlResult& result) noexcept {
    std::lock_guard lock(selection_mutex);
    latest_selection = {.valid = true,
                        .source_sequence = result.source_sequence,
                        .tracker_state = result.tracker_state,
                        .tracked_label = result.tracked_label,
                        .tracked_type = result.tracked_type,
                        .selected_slot = result.selected_slot,
                        .pending_slot = result.armor_selection.pending_slot,
                        .pending_duration_s = result.armor_selection.pending_duration_s,
                        .switch_confirmation_s = result.armor_selection.switch_confirmation_s};
  }

  runtime::FoxgloveSession session;
  pipeline::VisionDebugPipeline pipeline;
  control::ControlDebugPublisher control;
  mutable std::mutex selection_mutex;
  modules::ArmorSelectionSnapshot latest_selection;
  std::atomic<bool> stopped{false};
};

VisionDebugPublisher::VisionDebugPublisher(const Config& config)
    : impl_(std::make_unique<Impl>(config)) {}

VisionDebugPublisher::~VisionDebugPublisher() = default;

void VisionDebugPublisher::Publish(
    const hal::CameraFrame& frame, std::span<const modules::ArmorDetection> detections,
    const modules::DetectorStats& detector_stats,
    const modules::LightbarDetectionResult& lightbar_result,
    const modules::ArmorPnpFrameResult& pnp_result,
    const modules::ArmorPredictionResult& prediction_result) noexcept {
  const auto SELECTION = impl_->SelectionSnapshot();
  const bool SEQUENCE_MATCHES = SELECTION.valid &&
                                SELECTION.source_sequence <= prediction_result.sequence &&
                                prediction_result.sequence - SELECTION.source_sequence <= 2;
  const bool IDENTITY_MATCHES = SELECTION.tracked_label == prediction_result.label &&
                                SELECTION.tracked_type == prediction_result.type;
  const bool TRACKER_MATCHES = prediction_result.state != modules::TrackerState::LOST &&
                               SELECTION.tracker_state != modules::TrackerState::LOST &&
                               prediction_result.reset_reason.empty() &&
                               !(prediction_result.state == modules::TrackerState::DETECTING &&
                                 SELECTION.source_sequence != prediction_result.sequence);
  impl_->pipeline.Publish(
      frame, detections, detector_stats, lightbar_result, pnp_result, prediction_result,
      SEQUENCE_MATCHES && IDENTITY_MATCHES && TRACKER_MATCHES ? std::optional(SELECTION)
                                                              : std::nullopt);
}

void VisionDebugPublisher::PublishControl(const modules::FireControlResult& result) noexcept {
  impl_->UpdateSelection(result);
  impl_->control.Publish(result);
}

VisionPublisherStats VisionDebugPublisher::SnapshotStats() const noexcept {
  auto stats = impl_->pipeline.SnapshotStats();
  stats.dropped_control_samples = impl_->control.DroppedSamples();
  return stats;
}

bool VisionDebugPublisher::IsRunning() const noexcept {
  return impl_->pipeline.IsRunning();
}

std::filesystem::path VisionDebugPublisher::RecordingPath() const {
  return impl_->session.RecordingPath();
}

void VisionDebugPublisher::Stop() noexcept {
  impl_->Stop();
}

}  // namespace mv::tool::foxglove
