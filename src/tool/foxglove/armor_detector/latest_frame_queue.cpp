#include "tool/foxglove/armor_detector/latest_frame_queue.hpp"

#include <algorithm>
#include <utility>

namespace mv::tool::foxglove::armor_detector {

LatestFrameQueue::LatestFrameQueue(double max_fps)
    : PERIOD(std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(1.0 / max_fps))),
      JITTER_TOLERANCE(
          std::min(std::chrono::duration_cast<SteadyClock::duration>(std::chrono::milliseconds(2)),
                   PERIOD / 10)) {}

QueuePushResult LatestFrameQueue::Push(const hal::CameraFrame& frame,
                                       std::span<const modules::ArmorDetection> detections,
                                       const modules::DetectorStats& detector_stats) {
  std::lock_guard lock(mutex_);
  if (stopped_) {
    return {};
  }
  if (next_publish_timestamp_ != SteadyClock::time_point{} &&
      frame.timestamp + JITTER_TOLERANCE < next_publish_timestamp_) {
    return {.rate_limited = true};
  }
  // 小抖动沿用原调度节拍；长暂停后以当前帧重新锚定，避免连续追赶历史时隙。
  if (next_publish_timestamp_ == SteadyClock::time_point{} ||
      frame.timestamp > next_publish_timestamp_ + PERIOD) {
    next_publish_timestamp_ = frame.timestamp + PERIOD;
  } else {
    next_publish_timestamp_ += PERIOD;
  }

  FrameItem item;
  item.image = frame.image;
  item.timestamp = frame.timestamp;
  item.sequence = frame.sequence;
  item.detections.assign(detections.begin(), detections.end());
  item.detector_stats = detector_stats;

  const bool OVERWRITTEN = queued_frame_.has_value();
  // 队列容量固定为 1，生产者用新帧覆盖旧帧而不是等待编码线程。
  queued_frame_ = std::move(item);
  condition_.notify_one();
  return {.enqueued = true, .overwritten = OVERWRITTEN};
}

std::optional<FrameItem> LatestFrameQueue::WaitPop() noexcept {
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

}  // namespace mv::tool::foxglove::armor_detector
