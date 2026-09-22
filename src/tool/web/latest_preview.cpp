#include "tool/web/latest_preview.hpp"

#include "core/logger.hpp"
#include "tool/debug/armor_detection_overlay.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::tool::web {

LatestPreview::LatestPreview(PreviewConfig config)
    : config_(config),
      period_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / config.max_fps))) {}

LatestPreview::~LatestPreview() {
  Stop();
}

void LatestPreview::Start() {
  std::lock_guard lock(mutex_);
  if (running_) {
    return;
  }
  stopped_ = false;
  running_ = true;
  worker_ = std::thread(&LatestPreview::WorkerLoop, this);
}

void LatestPreview::Stop() noexcept {
  {
    std::lock_guard lock(mutex_);
    if (!running_) {
      return;
    }
    stopped_ = true;
    condition_.notify_one();
  }
  if (worker_.joinable()) {
    worker_.join();
  }
  std::lock_guard lock(mutex_);
  running_ = false;
  pending_.reset();
}

void LatestPreview::Push(const frame::FramePacket& packet, const runtime::VisionFrameOutput& output,
                         const runtime::VisionFrameDiagnostics& diagnostics) noexcept {
  try {
    std::lock_guard lock(mutex_);
    if (!running_ || stopped_) {
      return;
    }
    const auto NOW = packet.capture.stamp.receive_steady_time;
    if (next_frame_time_ != std::chrono::steady_clock::time_point{} && NOW < next_frame_time_) {
      return;
    }
    next_frame_time_ = NOW + period_;
    PendingFrame frame{.image = packet.capture.image,
                       .detections = output.detections,
                       .detector = diagnostics.detector,
                       .sequence = packet.capture.stamp.sequence,
                       .geometry_status = diagnostics.real_geometry_status};
    if (pending_) {
      ++overwritten_frames_;
    }
    pending_ = std::move(frame);
    condition_.notify_one();
  } catch (...) {
    std::lock_guard lock(mutex_);
    ++encoding_errors_;
  }
}

PreviewSnapshot LatestPreview::Snapshot() const noexcept {
  try {
    std::lock_guard lock(mutex_);
    return {.jpeg = latest_jpeg_,
            .source_sequence = latest_sequence_,
            .overwritten_frames = overwritten_frames_,
            .encoding_errors = encoding_errors_};
  } catch (...) {
    return {};
  }
}

void LatestPreview::WorkerLoop() noexcept {
  for (;;) {
    std::optional<PendingFrame> frame;
    {
      std::unique_lock lock(mutex_);
      condition_.wait(lock, [this] { return stopped_ || pending_.has_value(); });
      if (!pending_) {
        if (stopped_) {
          return;
        }
        continue;
      }
      frame = std::move(pending_);
      pending_.reset();
    }

    try {
      cv::Mat image = frame->image.clone();
      DrawArmorDetections(image, frame->detections);
      const auto TEXT = fmt::format("seq={} detections={} candidates={} {:.2f} ms", frame->sequence,
                                    frame->detections.size(), frame->detector.threshold_candidates,
                                    frame->detector.total_ms);
      cv::putText(image, TEXT, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2,
                  cv::LINE_AA);
      if (!frame->geometry_status.empty())
        cv::putText(image, frame->geometry_status, {10, 54}, cv::FONT_HERSHEY_SIMPLEX,
                    0.5, cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
      std::vector<unsigned char> encoded;
      if (!cv::imencode(".jpg", image, encoded, {cv::IMWRITE_JPEG_QUALITY, config_.jpeg_quality})) {
        throw std::runtime_error("OpenCV JPEG encoder returned false");
      }
      auto bytes = std::make_shared<const std::string>(
          reinterpret_cast<const char*>(encoded.data()), encoded.size());
      std::lock_guard lock(mutex_);
      latest_jpeg_ = std::move(bytes);
      latest_sequence_ = frame->sequence;
    } catch (const std::exception& error) {
      std::lock_guard lock(mutex_);
      ++encoding_errors_;
      MV_LOG_WARN("WebDebug", "preview encoding failed: {}", error.what());
    } catch (...) {
      std::lock_guard lock(mutex_);
      ++encoding_errors_;
      MV_LOG_WARN("WebDebug", "preview encoding failed: unknown exception");
    }
  }
}

}  // namespace mv::tool::web
