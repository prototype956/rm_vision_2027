#include "tool/foxglove/vision_debug_publisher.hpp"

#include "tool/foxglove/pipeline/vision_debug_pipeline.hpp"
#include "tool/foxglove/runtime/foxglove_session.hpp"

#include <atomic>
#include <utility>

namespace mv::tool::foxglove {

struct VisionDebugPublisher::Impl {
  explicit Impl(Config config) : session(config), pipeline(config, session) {
    // pipeline 构造期间先创建并注册全部频道，会话启动时才能一次性暴露完整话题集合。
    session.Start();
    pipeline.Start();
  }

  ~Impl() { Stop(); }

  void Stop() noexcept {
    if (stopped.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    // 先停止生产者、排空队列并关闭频道，再让会话收尾 MCAP 和 WebSocket。
    pipeline.Stop();
    session.Stop();
  }

  runtime::FoxgloveSession session;
  pipeline::VisionDebugPipeline pipeline;
  std::atomic<bool> stopped{false};
};

VisionDebugPublisher::VisionDebugPublisher(Config config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

VisionDebugPublisher::~VisionDebugPublisher() = default;

void VisionDebugPublisher::Publish(
    const hal::CameraFrame& frame, std::span<const modules::ArmorDetection> detections,
    const modules::DetectorStats& detector_stats, const modules::ArmorPnpFrameResult& pnp_result,
    const modules::ArmorPredictionResult& prediction_result) noexcept {
  impl_->pipeline.Publish(frame, detections, detector_stats, pnp_result, prediction_result);
}

VisionPublisherStats VisionDebugPublisher::SnapshotStats() const noexcept {
  return impl_->pipeline.SnapshotStats();
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
