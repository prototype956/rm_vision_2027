#pragma once

#include "hal/camera/i_camera.hpp"

#include <memory>

namespace mv::hal {

/**
 * @brief 从 Daedalus/Talos 共享内存读取同步仿真图像。
 *
 * 后端接受 Talos v2 RGB8 和 v3 RGB8/BGR8，向上层统一返回独立持有的 BGR8 图像。
 */
class TalosCamera final : public ICamera {
 public:
  TalosCamera();
  ~TalosCamera() override;

  TalosCamera(const TalosCamera&) = delete;
  TalosCamera& operator=(const TalosCamera&) = delete;
  TalosCamera(TalosCamera&& other) noexcept;
  TalosCamera& operator=(TalosCamera&& other) noexcept;

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
