#pragma once

#include "hal/camera/i_camera.hpp"

#include <memory>

namespace mv::hal {

/**
 * @brief 基于 MindVision SDK 的工业相机 HAL 适配器。
 *
 * 配置解析和 SDK 资源管理由内部实现负责；对上层保持统一的 ICamera 生命周期。
 */
class MindVisionCamera : public ICamera {
 public:
  MindVisionCamera();
  ~MindVisionCamera() override;

  MindVisionCamera(const MindVisionCamera&) = delete;
  MindVisionCamera& operator=(const MindVisionCamera&) = delete;
  MindVisionCamera(MindVisionCamera&& other);
  MindVisionCamera& operator=(MindVisionCamera&& other) noexcept;

  bool Open(const YAML::Node& config) override;
  void Close() override;
  GrabStatus Grab(CameraFrame& frame) override;
  [[nodiscard]] CameraInfo Info() const override;
  [[nodiscard]] bool IsOpen() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::hal
