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

  void UpdateSelection(const ::mv::runtime::ControlCycleOutput& output,
                       const ::mv::runtime::ControlCycleDiagnostics& diagnostics) noexcept {
    std::lock_guard lock(selection_mutex);
    latest_selection = modules::ArmorSelectionSnapshot{
        .valid = true,
        .source_sequence = output.fire_control.source_sequence,
        .tracker_state = output.fire_control.tracker_state,
        .tracked_label = output.fire_control.tracked_label,
        .tracked_type = output.fire_control.tracked_type,
        .selected_slot = output.fire_control.selected_slot,
        .pending_slot = diagnostics.fire_control.armor_selection.pending_slot,
        .pending_duration_s = diagnostics.fire_control.armor_selection.pending_duration_s,
        .switch_confirmation_s = diagnostics.fire_control.armor_selection.switch_confirmation_s};
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
    const frame::FramePacket& packet, const ::mv::runtime::VisionFrameOutput& output,
    const ::mv::runtime::VisionFrameDiagnostics& diagnostics,
    const std::optional<simulation_evaluation::SimulationEvaluationResult>&
        simulation_evaluation) noexcept {
  const auto SELECTION = impl_->SelectionSnapshot();
  const bool SEQUENCE_MATCHES = SELECTION.valid &&
                                SELECTION.source_sequence <= output.prediction.sequence &&
                                output.prediction.sequence - SELECTION.source_sequence <= 2;
  const bool IDENTITY_MATCHES = SELECTION.tracked_label == output.prediction.label &&
                                SELECTION.tracked_type == output.prediction.type;
  const bool TRACKER_MATCHES = output.prediction.state != modules::TrackerState::LOST &&
                               SELECTION.tracker_state != modules::TrackerState::LOST &&
                               diagnostics.prediction.reset_reason.empty() &&
                               !(output.prediction.state == modules::TrackerState::DETECTING &&
                                 SELECTION.source_sequence != output.prediction.sequence);
  impl_->pipeline.Publish(packet, output, diagnostics, simulation_evaluation,
                          SEQUENCE_MATCHES && IDENTITY_MATCHES && TRACKER_MATCHES
                              ? std::optional(SELECTION)
                              : std::nullopt);
}

void VisionDebugPublisher::PublishControl(
    const ::mv::runtime::ControlCycleOutput& output,
    const ::mv::runtime::ControlCycleDiagnostics& diagnostics) noexcept {
  impl_->UpdateSelection(output, diagnostics);
  impl_->pipeline.UpdateImpact(output.fire_control);
  impl_->control.Publish(output, diagnostics);
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
