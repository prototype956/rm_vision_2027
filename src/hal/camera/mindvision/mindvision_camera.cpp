#include "mindvision_camera.hpp"

#include "core/logger.hpp"
#include "mindvision_config.hpp"
#include "mindvision_device.hpp"

#include <exception>
#include <utility>

namespace mv::hal {

struct MindVisionCamera::Impl {
  detail::MindVisionDevice device;
};

MindVisionCamera::MindVisionCamera() : impl_(std::make_unique<Impl>()) {}

MindVisionCamera::~MindVisionCamera() = default;

MindVisionCamera::MindVisionCamera(MindVisionCamera&& other) : impl_(std::make_unique<Impl>()) {
  impl_.swap(other.impl_);
}

MindVisionCamera& MindVisionCamera::operator=(MindVisionCamera&& other) noexcept {
  if (this == &other)
    return *this;
  Close();
  impl_.swap(other.impl_);
  return *this;
}

bool MindVisionCamera::Open(const YAML::Node& config) {
  if (impl_->device.IsOpen())
    return true;
  try {
    return impl_->device.Open(detail::ParseMindVisionConfig(config));
  } catch (const std::exception& error) {
    MV_LOG_ERROR("HAL.Camera.MV", "invalid config: {}", error.what());
    return false;
  }
}

void MindVisionCamera::Close() {
  impl_->device.Close();
}

GrabStatus MindVisionCamera::Grab(CameraFrame& frame) {
  return impl_->device.Grab(frame);
}

CameraInfo MindVisionCamera::Info() const {
  return impl_->device.Info();
}

bool MindVisionCamera::IsOpen() const {
  return impl_->device.IsOpen();
}

}  // namespace mv::hal
