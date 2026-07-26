#pragma once

#include "hal/camera/i_camera.hpp"
#include "mindvision_config.hpp"

#include <memory>

namespace mv::hal::detail {

/**
 * @brief 独占 MindVision SDK 句柄及采集资源的内部设备对象。
 */
class MindVisionDevice final {
 public:
  MindVisionDevice();
  ~MindVisionDevice();

  MindVisionDevice(const MindVisionDevice&) = delete;
  MindVisionDevice& operator=(const MindVisionDevice&) = delete;
  MindVisionDevice(MindVisionDevice&&) = delete;
  MindVisionDevice& operator=(MindVisionDevice&&) = delete;

  bool Open(const MindVisionConfig& config);
  void Close() noexcept;
  GrabStatus Grab(CameraFrame& frame);

  [[nodiscard]] CameraInfo Info() const;
  [[nodiscard]] bool IsOpen() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::hal::detail
