#include "tool/foxglove/pipeline/vision_debug_pipeline.hpp"

#include "core/logger.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <mutex>
#include <string>
#include <utility>

namespace mv::tool::foxglove::pipeline {

VisionDebugPipeline::VisionDebugPipeline(const Config& config, runtime::FoxgloveSession& session)
    : session_(session), queue_(config.image.max_fps), encoder_(config.image) {
  // 两个 Context 的频道独立创建；一个 sink 失败不会阻止另一 sink 继续工作。
  if (session_.LiveConfigured()) {
    try {
      live_channels_ = std::make_unique<VisionChannelSet>(session_.LiveContext());
      live_channel_ids_ = live_channels_->Ids();
      session_.RegisterLiveChannel(live_channel_ids_.image);
      session_.RegisterLiveChannel(live_channel_ids_.armor_annotations);
      session_.RegisterLiveChannel(live_channel_ids_.armor_stats);
      session_.RegisterLiveChannel(live_channel_ids_.lightbar_annotations);
      session_.RegisterLiveChannel(live_channel_ids_.lightbar_stats);
      session_.RegisterLiveChannel(live_channel_ids_.debug_stats);
      session_.RegisterLiveChannel(live_channel_ids_.calibration);
      session_.RegisterLiveChannel(live_channel_ids_.frustum);
      session_.RegisterLiveChannel(live_channel_ids_.ground_truth);
      session_.RegisterLiveChannel(live_channel_ids_.projectile_stats);
      session_.RegisterLiveChannel(live_channel_ids_.referee_state);
      session_.RegisterLiveChannel(live_channel_ids_.combat_evaluation);
      session_.RegisterLiveChannel(live_channel_ids_.projection_annotations);
      session_.RegisterLiveChannel(live_channel_ids_.pnp_estimates);
      session_.RegisterLiveChannel(live_channel_ids_.pnp_raw_corners);
      session_.RegisterLiveChannel(live_channel_ids_.pnp_final_corners);
      session_.RegisterLiveChannel(live_channel_ids_.pnp_reprojection);
      session_.RegisterLiveChannel(live_channel_ids_.pnp_error_vectors);
      session_.RegisterLiveChannel(live_channel_ids_.corner_refiner_axes);
      session_.RegisterLiveChannel(live_channel_ids_.corner_refiner_candidates);
      session_.RegisterLiveChannel(live_channel_ids_.pnp_stats);
      session_.RegisterLiveChannel(live_channel_ids_.prediction_scene);
      session_.RegisterLiveChannel(live_channel_ids_.impact_scene);
      session_.RegisterLiveChannel(live_channel_ids_.prediction_state);
      session_.RegisterLiveChannel(live_channel_ids_.prediction_truth_overlay);
      session_.RegisterLiveChannel(live_channel_ids_.prediction_current_annotations);
      session_.RegisterLiveChannel(live_channel_ids_.impact_annotations);
      session_.RegisterLiveChannel(live_channel_ids_.selected_armor_annotations);
    } catch (const std::exception& error) {
      live_channels_.reset();
      session_.FailLiveSetup(error.what());
    }
  }

  if (session_.RecordingConfigured()) {
    try {
      recording_channels_ = std::make_unique<VisionChannelSet>(session_.RecordingContext());
    } catch (const std::exception& error) {
      recording_channels_.reset();
      session_.FailRecordingSetup(error.what());
    }
  }
}

VisionDebugPipeline::~VisionDebugPipeline() {
  Stop();
}

void VisionDebugPipeline::Start() noexcept {
  if (!session_.AnyActive()) {
    return;
  }
  accepting_.store(true, std::memory_order_release);
  try {
    worker_ = std::thread([this] { WorkerLoop(); });
  } catch (const std::exception& error) {
    accepting_.store(false, std::memory_order_release);
    const auto COUNT = metrics_.OnEncodingError();
    MV_LOG_ERROR("Foxglove", "failed to start vision debug worker #{}: {}", COUNT, error.what());
  }
}

TopicDemand VisionDebugPipeline::LiveDemand() const noexcept {
  if (!session_.LiveActive() || !live_channels_) {
    return {};
  }
  return {
      .image = session_.Subscription(live_channel_ids_.image).subscribers > 0,
      .armor_annotations =
          session_.Subscription(live_channel_ids_.armor_annotations).subscribers > 0,
      .armor_stats = session_.Subscription(live_channel_ids_.armor_stats).subscribers > 0,
      .lightbar_annotations =
          session_.Subscription(live_channel_ids_.lightbar_annotations).subscribers > 0,
      .lightbar_stats = session_.Subscription(live_channel_ids_.lightbar_stats).subscribers > 0,
      .debug_stats = session_.Subscription(live_channel_ids_.debug_stats).subscribers > 0,
      .calibration = session_.Subscription(live_channel_ids_.calibration).subscribers > 0,
      .frustum = session_.Subscription(live_channel_ids_.frustum).subscribers > 0,
      .ground_truth = session_.Subscription(live_channel_ids_.ground_truth).subscribers > 0,
      .projectile_stats = session_.Subscription(live_channel_ids_.projectile_stats).subscribers > 0,
      .referee_state = session_.Subscription(live_channel_ids_.referee_state).subscribers > 0,
      .combat_evaluation =
          session_.Subscription(live_channel_ids_.combat_evaluation).subscribers > 0,
      .projection_annotations =
          session_.Subscription(live_channel_ids_.projection_annotations).subscribers > 0,
      .pnp_estimates = session_.Subscription(live_channel_ids_.pnp_estimates).subscribers > 0,
      .pnp_raw_corners = session_.Subscription(live_channel_ids_.pnp_raw_corners).subscribers > 0,
      .pnp_final_corners =
          session_.Subscription(live_channel_ids_.pnp_final_corners).subscribers > 0,
      .pnp_reprojection = session_.Subscription(live_channel_ids_.pnp_reprojection).subscribers > 0,
      .pnp_error_vectors =
          session_.Subscription(live_channel_ids_.pnp_error_vectors).subscribers > 0,
      .corner_refiner_axes =
          session_.Subscription(live_channel_ids_.corner_refiner_axes).subscribers > 0,
      .corner_refiner_candidates =
          session_.Subscription(live_channel_ids_.corner_refiner_candidates).subscribers > 0,
      .pnp_stats = session_.Subscription(live_channel_ids_.pnp_stats).subscribers > 0,
      .prediction_scene = session_.Subscription(live_channel_ids_.prediction_scene).subscribers > 0,
      .impact_scene = session_.Subscription(live_channel_ids_.impact_scene).subscribers > 0,
      .prediction_state = session_.Subscription(live_channel_ids_.prediction_state).subscribers > 0,
      .prediction_truth_overlay =
          session_.Subscription(live_channel_ids_.prediction_truth_overlay).subscribers > 0,
      .prediction_current_annotations =
          session_.Subscription(live_channel_ids_.prediction_current_annotations).subscribers > 0,
      .impact_annotations =
          session_.Subscription(live_channel_ids_.impact_annotations).subscribers > 0,
      .selected_armor_annotations =
          session_.Subscription(live_channel_ids_.selected_armor_annotations).subscribers > 0,
  };
}

void VisionDebugPipeline::UpdateImpact(const modules::FireControlOutput& output) noexcept {
  try {
    const modules::ArmorImpactSnapshot SNAPSHOT{.source_sequence = output.source_sequence,
                                                .selected_slot = output.selected_slot,
                                                .ballistic = output.ballistic};
    std::lock_guard lock(impact_mutex_);
    if (impact_stopped_)
      return;
    const auto EXISTING = std::find_if(
        impact_snapshots_.begin(), impact_snapshots_.end(),
        [&](const auto& value) { return value.source_sequence == SNAPSHOT.source_sequence; });
    if (EXISTING != impact_snapshots_.end()) {
      *EXISTING = SNAPSHOT;
    } else {
      impact_snapshots_.push_back(SNAPSHOT);
      if (impact_snapshots_.size() > 8)
        impact_snapshots_.pop_front();
    }
    impact_condition_.notify_all();
  } catch (...) {
    const auto COUNT = metrics_.OnEncodingError();
    if (COUNT == 1 || COUNT % 100 == 0)
      MV_LOG_ERROR("Foxglove", "failed to cache impact snapshot #{}", COUNT);
  }
}

void VisionDebugPipeline::Publish(
    const frame::FramePacket& packet, const ::mv::runtime::VisionFrameOutput& output,
    const ::mv::runtime::VisionFrameDiagnostics& diagnostics,
    const std::optional<simulation_evaluation::SimulationEvaluationResult>& simulation_evaluation,
    std::optional<modules::ArmorSelectionSnapshot> selection) noexcept {
  metrics_.OnSubmitted();
  const auto LIVE_DEMAND = LiveDemand();
  // 无订阅且未录制时不复制图像和检测结果，调试功能保持近似零额外负载。
  if (!accepting_.load(std::memory_order_acquire) ||
      (!LIVE_DEMAND.Any() && !session_.RecordingActive())) {
    return;
  }
  try {
    const auto RESULT = queue_.Push(packet, output, diagnostics, simulation_evaluation, selection);
    if (RESULT.rate_limited) {
      metrics_.OnRateLimited();
    } else if (RESULT.enqueued) {
      metrics_.OnEnqueued(RESULT.overwritten);
    }
  } catch (const std::exception& error) {
    const auto COUNT = metrics_.OnEncodingError();
    if (COUNT == 1 || COUNT % 100 == 0) {
      MV_LOG_ERROR("Foxglove", "failed to enqueue vision debug frame #{}: {}", COUNT, error.what());
    }
  }
}

void VisionDebugPipeline::WorkerLoop() noexcept {
  while (true) {
    auto frame = queue_.WaitPop();
    if (!frame.has_value()) {
      return;
    }
    try {
      ProcessFrame(*frame);
    } catch (const std::exception& error) {
      const auto COUNT = metrics_.OnEncodingError();
      if (COUNT == 1 || COUNT % 100 == 0) {
        MV_LOG_ERROR("Foxglove", "vision debug worker error #{}: {}", COUNT, error.what());
      }
    }
  }
}

void VisionDebugPipeline::ProcessFrame(const VisionDebugFrame& frame) {
  const auto LIVE_DEMAND = LiveDemand();
  const bool RECORD = session_.RecordingActive() && recording_channels_;
  // MCAP 录制请求全部领域；缺少 geometry 时编码器会自然省略空间和仿真消息。
  const TopicDemand RECORDING_DEMAND = RECORD ? TopicDemand{.image = true,
                                                            .armor_annotations = true,
                                                            .armor_stats = true,
                                                            .lightbar_annotations = true,
                                                            .lightbar_stats = true,
                                                            .debug_stats = true,
                                                            .calibration = true,
                                                            .frustum = true,
                                                            .ground_truth = true,
                                                            .projectile_stats = true,
                                                            .referee_state = true,
                                                            .combat_evaluation = true,
                                                            .projection_annotations = true,
                                                            .pnp_estimates = true,
                                                            .pnp_raw_corners = true,
                                                            .pnp_final_corners = true,
                                                            .pnp_reprojection = true,
                                                            .pnp_error_vectors = true,
                                                            .corner_refiner_axes = true,
                                                            .corner_refiner_candidates = true,
                                                            .pnp_stats = true,
                                                            .prediction_scene = true,
                                                            .impact_scene = true,
                                                            .prediction_state = true,
                                                            .prediction_truth_overlay = true,
                                                            .prediction_current_annotations = true,
                                                            .impact_annotations = true,
                                                            .selected_armor_annotations = true}
                                              : TopicDemand{};
  const auto COMBINED_DEMAND = Merge(LIVE_DEMAND, RECORDING_DEMAND);
  if (!COMBINED_DEMAND.Any()) {
    return;
  }

  const auto IMPACT = (COMBINED_DEMAND.impact_annotations || COMBINED_DEMAND.impact_scene)
                          ? WaitForImpact(frame.output.prediction.sequence)
                          : std::nullopt;
  // 合并后只编码一次，同一 PreparedFrame 供实时与录制频道复用。
  const auto FRAME = encoder_.Encode(frame, COMBINED_DEMAND, metrics_.Counts(), IMPACT);
  if (FRAME.jpeg_ms.has_value()) {
    metrics_.OnEncoded();
  }
  std::lock_guard publish_lock(session_.PublishMutex());
  if (LIVE_DEMAND.Any() && session_.LiveActive() && live_channels_) {
    const auto RESULT = live_channels_->Publish(FRAME, LIVE_DEMAND);
    ReportPublishErrors(RESULT, false);
    if (RESULT.attempted && RESULT.success) {
      metrics_.OnLivePublished();
    }
  }
  if (RECORD && session_.RecordingActive() && recording_channels_) {
    const auto RESULT = recording_channels_->Publish(FRAME, RECORDING_DEMAND);
    ReportPublishErrors(RESULT, true);
    if (RESULT.attempted && RESULT.success) {
      metrics_.OnRecorded();
    }
  }
  metrics_.AddLatency(FRAME.jpeg_ms, FRAME.publish_latency_ms);
}

std::optional<modules::ArmorImpactSnapshot> VisionDebugPipeline::WaitForImpact(
    std::uint64_t source_sequence) noexcept {
  try {
    std::unique_lock lock(impact_mutex_);
    const auto FIND = [&]() {
      return std::find_if(impact_snapshots_.begin(), impact_snapshots_.end(),
                          [source_sequence](const auto& value) {
                            return value.source_sequence == source_sequence;
                          });
    };
    impact_condition_.wait_for(lock, std::chrono::milliseconds(20), [&] {
      return impact_stopped_ || FIND() != impact_snapshots_.end() ||
             (!impact_snapshots_.empty() &&
              impact_snapshots_.back().source_sequence > source_sequence);
    });
    const auto MATCH = FIND();
    return MATCH == impact_snapshots_.end() ? std::nullopt : std::optional(*MATCH);
  } catch (...) {
    return std::nullopt;
  }
}

void VisionDebugPipeline::ReportPublishErrors(const ChannelPublishResult& result,
                                              bool recording) noexcept {
  for (std::size_t index = 0; index < result.error_count; ++index) {
    const auto& publish_error = result.errors[index];
    const std::string OPERATION =
        std::string(recording ? "MCAP " : "live ") + TopicName(publish_error.topic);
    if (recording) {
      session_.ReportRecordingError(OPERATION, publish_error.error);
    } else {
      session_.ReportLiveError(OPERATION, publish_error.error);
    }
  }
}

VisionPublisherStats VisionDebugPipeline::SnapshotStats() const noexcept {
  runtime::SubscriptionSnapshot image;
  runtime::SubscriptionSnapshot armor_annotations;
  runtime::SubscriptionSnapshot armor_stats;
  runtime::SubscriptionSnapshot debug_stats;
  if (live_channels_) {
    image = session_.Subscription(live_channel_ids_.image);
    armor_annotations = session_.Subscription(live_channel_ids_.armor_annotations);
    armor_stats = session_.Subscription(live_channel_ids_.armor_stats);
    debug_stats = session_.Subscription(live_channel_ids_.debug_stats);
  }
  return metrics_.Snapshot(session_.Snapshot(), image, armor_annotations, armor_stats, debug_stats);
}

bool VisionDebugPipeline::IsRunning() const noexcept {
  return accepting_.load(std::memory_order_acquire) && session_.AnyActive();
}

void VisionDebugPipeline::Stop() noexcept {
  if (stop_called_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  accepting_.store(false, std::memory_order_release);
  {
    std::lock_guard lock(impact_mutex_);
    impact_stopped_ = true;
  }
  impact_condition_.notify_all();
  queue_.Stop();
  if (worker_.joinable()) {
    worker_.join();
  }
  if (live_channels_) {
    live_channels_->Close();
  }
  if (recording_channels_) {
    recording_channels_->Close();
  }
}

}  // namespace mv::tool::foxglove::pipeline
