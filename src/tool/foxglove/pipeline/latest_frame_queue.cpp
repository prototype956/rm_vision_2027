#include "tool/foxglove/pipeline/latest_frame_queue.hpp"

#include <algorithm>
#include <utility>

namespace mv::tool::foxglove::pipeline {

LatestFrameQueue::LatestFrameQueue(double max_fps)
    : PERIOD(std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(1.0 / max_fps))),
      JITTER_TOLERANCE(
          std::min(std::chrono::duration_cast<SteadyClock::duration>(std::chrono::milliseconds(2)),
                   PERIOD / 10)) {}

QueuePushResult LatestFrameQueue::Push(const hal::CameraFrame& frame,
                                       std::span<const modules::ArmorDetection> detections,
                                       const modules::DetectorStats& detector_stats,
                                       const modules::LightbarDetectionResult& lightbar_result,
                                       const modules::ArmorPnpFrameResult& pnp_result,
                                       const modules::ArmorPredictionResult& prediction_result,
                                       std::optional<modules::ArmorSelectionSnapshot> selection) {
  std::lock_guard lock(mutex_);
  if (stopped_) {
    return {};
  }
  if (next_publish_timestamp_ != SteadyClock::time_point{} &&
      frame.receive_steady_time + JITTER_TOLERANCE < next_publish_timestamp_) {
    return {.rate_limited = true};
  }
  if (next_publish_timestamp_ == SteadyClock::time_point{} ||
      frame.receive_steady_time > next_publish_timestamp_ + PERIOD) {
    next_publish_timestamp_ = frame.receive_steady_time + PERIOD;
  } else {
    next_publish_timestamp_ += PERIOD;
  }

  VisionDebugFrame item;
  item.image = frame.image;
  item.receive_steady_time = frame.receive_steady_time;
  item.capture_timestamp_ns = frame.capture_timestamp_ns;
  item.geometry = frame.geometry;
  item.sequence = frame.sequence;
  item.source_invalid_frames = frame.source_invalid_frames;
  item.detections.assign(detections.begin(), detections.end());
  item.detector_stats = detector_stats;
  item.lightbar_result = lightbar_result;
  item.pnp_result = pnp_result;
  item.prediction_result = prediction_result;
  item.armor_selection = selection;

  const bool OVERWRITTEN = queued_frame_.has_value();
  queued_frame_ = std::move(item);
  condition_.notify_one();
  return {.enqueued = true, .overwritten = OVERWRITTEN};
}

std::optional<VisionDebugFrame> LatestFrameQueue::WaitPop() noexcept {
  try {
    std::unique_lock lock(mutex_);
    condition_.wait(lock, [this] { return stopped_ || queued_frame_.has_value(); });
    if (!queued_frame_.has_value()) {
      return std::nullopt;
    }
    auto result = std::move(queued_frame_);
    queued_frame_.reset();
    return result;
  } catch (...) {
    return std::nullopt;
  }
}

void LatestFrameQueue::Stop() noexcept {
  std::lock_guard lock(mutex_);
  stopped_ = true;
  condition_.notify_one();
}

}  // namespace mv::tool::foxglove::pipeline
