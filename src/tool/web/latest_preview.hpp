#pragma once

#include "frame/frame_packet.hpp"
#include "runtime/vision_frame_diagnostics.hpp"
#include "runtime/vision_frame_output.hpp"
#include "tool/web/web_debug_config.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <optional>

namespace mv::tool::web {

struct PreviewSnapshot {
  std::shared_ptr<const std::string> jpeg;
  std::uint64_t source_sequence{0};
  std::uint64_t overwritten_frames{0};
  std::uint64_t encoding_errors{0};
};

/** @brief 容量为一的非阻塞预览队列和后台 JPEG 编码器。 */
class LatestPreview final {
 public:
  explicit LatestPreview(PreviewConfig config);
  ~LatestPreview();

  LatestPreview(const LatestPreview&) = delete;
  LatestPreview& operator=(const LatestPreview&) = delete;

  void Start();
  void Stop() noexcept;
  void Push(const frame::FramePacket& packet, const runtime::VisionFrameOutput& output,
            const runtime::VisionFrameDiagnostics& diagnostics) noexcept;
  [[nodiscard]] PreviewSnapshot Snapshot() const noexcept;

 private:
  struct PendingFrame {
    cv::Mat image;
    std::vector<modules::ArmorDetection> detections;
    modules::ArmorDetectorDiagnostics detector;
    std::uint64_t sequence{0};
    std::string geometry_status;
  };

  void WorkerLoop() noexcept;

  PreviewConfig config_;
  std::chrono::steady_clock::duration period_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<PendingFrame> pending_;
  std::shared_ptr<const std::string> latest_jpeg_;
  std::chrono::steady_clock::time_point next_frame_time_{};
  std::thread worker_;
  bool running_{false};
  bool stopped_{false};
  std::uint64_t latest_sequence_{0};
  std::uint64_t overwritten_frames_{0};
  std::uint64_t encoding_errors_{0};
};

}  // namespace mv::tool::web
