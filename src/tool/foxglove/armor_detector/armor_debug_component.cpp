#include "tool/foxglove/armor_detector/armor_debug_component.hpp"

#include "core/logger.hpp"

#include <exception>
#include <string>
#include <utility>

namespace mv::tool::foxglove::armor_detector {

ArmorDebugComponent::ArmorDebugComponent(const Config& config, runtime::FoxgloveSession& session)
    : session_(session), queue_(config.image.max_fps), encoder_(config.image) {
  // 两套频道结构相同但绑定独立 Context，使实时与录制故障互不传播。
  if (session_.LiveConfigured()) {
    try {
      live_channels_ = std::make_unique<ArmorChannelSet>(session_.LiveContext());
      live_channel_ids_ = live_channels_->Ids();
      session_.RegisterLiveChannel(live_channel_ids_.image);
      session_.RegisterLiveChannel(live_channel_ids_.annotations);
      session_.RegisterLiveChannel(live_channel_ids_.stats);
    } catch (const std::exception& error) {
      live_channels_.reset();
      session_.FailLiveSetup(error.what());
    }
  }

  if (session_.RecordingConfigured()) {
    try {
      recording_channels_ = std::make_unique<ArmorChannelSet>(session_.RecordingContext());
    } catch (const std::exception& error) {
      recording_channels_.reset();
      session_.FailRecordingSetup(error.what());
    }
  }
}

ArmorDebugComponent::~ArmorDebugComponent() {
  Stop();
}

void ArmorDebugComponent::Start() noexcept {
  if (!session_.AnyActive()) {
    return;
  }
  accepting_.store(true, std::memory_order_release);
  try {
    worker_ = std::thread([this] { WorkerLoop(); });
  } catch (const std::exception& error) {
    accepting_.store(false, std::memory_order_release);
    const auto COUNT = metrics_.OnEncodingError();
    MV_LOG_ERROR("Foxglove", "failed to start armor debug worker #{}: {}", COUNT, error.what());
  }
}

TopicDemand ArmorDebugComponent::LiveDemand() const noexcept {
  if (!session_.LiveActive() || !live_channels_) {
    return {};
  }
  return {
      .image = session_.Subscription(live_channel_ids_.image).subscribers > 0,
      .annotations = session_.Subscription(live_channel_ids_.annotations).subscribers > 0,
      .stats = session_.Subscription(live_channel_ids_.stats).subscribers > 0,
  };
}

void ArmorDebugComponent::Publish(const hal::CameraFrame& frame,
                                  std::span<const modules::ArmorDetection> detections,
                                  const modules::DetectorStats& detector_stats) noexcept {
  metrics_.OnSubmitted();
  const auto LIVE_DEMAND = LiveDemand();
  // 没有实时订阅且未录制时不进入队列，从源头跳过 JPEG 与消息构造。
  if (!accepting_.load(std::memory_order_acquire) ||
      (!LIVE_DEMAND.Any() && !session_.RecordingActive())) {
    return;
  }
  try {
    const auto RESULT = queue_.Push(frame, detections, detector_stats);
    if (RESULT.rate_limited) {
      metrics_.OnRateLimited();
    } else if (RESULT.enqueued) {
      metrics_.OnEnqueued(RESULT.overwritten);
    }
  } catch (const std::exception& error) {
    const auto COUNT = metrics_.OnEncodingError();
    if (COUNT == 1 || COUNT % 100 == 0) {
      MV_LOG_ERROR("Foxglove", "failed to enqueue armor debug frame #{}: {}", COUNT, error.what());
    }
  }
}

void ArmorDebugComponent::WorkerLoop() noexcept {
  while (true) {
    auto item = queue_.WaitPop();
    if (!item.has_value()) {
      return;
    }
    try {
      ProcessFrame(*item);
    } catch (const std::exception& error) {
      const auto COUNT = metrics_.OnEncodingError();
      if (COUNT == 1 || COUNT % 100 == 0) {
        MV_LOG_ERROR("Foxglove", "armor debug worker error #{}: {}", COUNT, error.what());
      }
    }
  }
}

void ArmorDebugComponent::ProcessFrame(const FrameItem& item) {
  const auto LIVE_DEMAND = LiveDemand();
  const bool RECORD = session_.RecordingActive() && recording_channels_;
  const TopicDemand RECORDING_DEMAND =
      RECORD ? TopicDemand{.image = true, .annotations = true, .stats = true} : TopicDemand{};
  const auto COMBINED_DEMAND = Merge(LIVE_DEMAND, RECORDING_DEMAND);
  if (!COMBINED_DEMAND.Any()) {
    return;
  }

  // 合并两个 sink 的需求后只编码一次，生成的消息批次随后分别写入两个 Context。
  const auto FRAME = encoder_.Encode(item, COMBINED_DEMAND, metrics_.Counts());
  if (FRAME.jpeg_ms.has_value()) {
    metrics_.OnEncoded();
  }

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

void ArmorDebugComponent::ReportPublishErrors(const ChannelPublishResult& result,
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

PublisherStats ArmorDebugComponent::SnapshotStats() const noexcept {
  runtime::SubscriptionSnapshot image;
  runtime::SubscriptionSnapshot annotations;
  runtime::SubscriptionSnapshot stats;
  if (live_channels_) {
    image = session_.Subscription(live_channel_ids_.image);
    annotations = session_.Subscription(live_channel_ids_.annotations);
    stats = session_.Subscription(live_channel_ids_.stats);
  }
  return metrics_.Snapshot(session_.Snapshot(), image, annotations, stats);
}

bool ArmorDebugComponent::IsRunning() const noexcept {
  return accepting_.load(std::memory_order_acquire) && session_.AnyActive();
}

void ArmorDebugComponent::Stop() noexcept {
  if (stop_called_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  accepting_.store(false, std::memory_order_release);
  // Stop() 唤醒消费者但保留队列内容，工作线程会先处理最后一帧再退出。
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

}  // namespace mv::tool::foxglove::armor_detector
