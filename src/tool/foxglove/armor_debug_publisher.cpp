#include "tool/foxglove/armor_debug_publisher.hpp"

#include "tool/foxglove/armor_detector/armor_debug_component.hpp"
#include "tool/foxglove/runtime/foxglove_session.hpp"

#include <atomic>
#include <utility>

namespace mv::tool::foxglove {

struct ArmorDebugPublisher::Impl {
  explicit Impl(Config config) : session(config), armor_component(config, session) {
    // 频道必须先绑定 Context 并完成订阅注册，Server/Writer 才能看到完整频道集合。
    session.Start();
    armor_component.Start();
  }

  ~Impl() { Stop(); }

  void Stop() noexcept {
    if (stopped.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    // 先停止生产者并关闭频道，再由会话写入 MCAP 尾部和停止 WebSocket。
    armor_component.Stop();
    session.Stop();
  }

  runtime::FoxgloveSession session;
  armor_detector::ArmorDebugComponent armor_component;
  std::atomic<bool> stopped{false};
};

ArmorDebugPublisher::ArmorDebugPublisher(Config config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

ArmorDebugPublisher::~ArmorDebugPublisher() = default;

void ArmorDebugPublisher::Publish(const hal::CameraFrame& frame,
                                  std::span<const modules::ArmorDetection> detections,
                                  const modules::DetectorStats& detector_stats) noexcept {
  impl_->armor_component.Publish(frame, detections, detector_stats);
}

PublisherStats ArmorDebugPublisher::SnapshotStats() const noexcept {
  return impl_->armor_component.SnapshotStats();
}

bool ArmorDebugPublisher::IsRunning() const noexcept {
  return impl_->armor_component.IsRunning();
}

std::filesystem::path ArmorDebugPublisher::RecordingPath() const {
  return impl_->session.RecordingPath();
}

void ArmorDebugPublisher::Stop() noexcept {
  impl_->Stop();
}

}  // namespace mv::tool::foxglove
