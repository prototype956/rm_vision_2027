#pragma once

#include "i_camera.hpp"

#include <memory>

namespace mv::hal {

class MindVisionCamera : public ICamera {
 public:
  MindVisionCamera();
  ~MindVisionCamera() override;

  // Pimpl 持有 unique_ptr，不可拷贝，可移动
  MindVisionCamera(const MindVisionCamera&) = delete;
  MindVisionCamera& operator=(const MindVisionCamera&) = delete;
  MindVisionCamera(MindVisionCamera&&) noexcept;
  MindVisionCamera& operator=(MindVisionCamera&&) noexcept;

  bool Open(const YAML::Node& config) override;
  void Close() override;
  GrabStatus Grab(CameraFrame& frame) override;
  [[nodiscard]] CameraInfo Info() const override;
  [[nodiscard]] bool IsOpen() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;  // 所有 SDK 相关成员都在 Impl 里
};

}  // namespace mv::hal
