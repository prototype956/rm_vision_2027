#include "hal/camera/talos/talos_camera.hpp"

#include "core/logger.hpp"
#include "talos_config.hpp"
#include "talos_device.hpp"

#include <exception>
#include <utility>

namespace mv::hal {

struct TalosCamera::Impl {
  detail::TalosDevice device;
};

TalosCamera::TalosCamera() : impl_(std::make_unique<Impl>()) {}

TalosCamera::~TalosCamera() = default;

TalosCamera::TalosCamera(TalosCamera&& other) noexcept : impl_(std::make_unique<Impl>()) {
  impl_.swap(other.impl_);
}

TalosCamera& TalosCamera::operator=(TalosCamera&& other) noexcept {
  if (this != &other) {
    Close();
    impl_.swap(other.impl_);
  }
  return *this;
}

bool TalosCamera::Open(const YAML::Node& config) {
  if (impl_->device.IsOpen()) {
    return true;
  }
  try {
    return impl_->device.Open(detail::ParseTalosConfig(config));
  } catch (const std::exception& error) {
    MV_LOG_ERROR("HAL.Camera.Talos", "invalid config: {}", error.what());
    return false;
  }
}

void TalosCamera::Close() {
  impl_->device.Close();
}

GrabStatus TalosCamera::Grab(CameraFrame& frame) {
  return impl_->device.Grab(frame);
}

CameraInfo TalosCamera::Info() const {
  return impl_->device.Info();
}

bool TalosCamera::IsOpen() const {
  return impl_->device.IsOpen();
}

}  // namespace mv::hal
