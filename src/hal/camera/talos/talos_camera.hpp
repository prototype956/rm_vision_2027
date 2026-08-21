#pragma once

#include "hal/camera/i_camera.hpp"

#include <memory>

namespace mv::hal {

/**
 * @brief 从 Daedalus/Talos 共享内存读取同步仿真图像。
 *
 * 后端消费 Talos v6 原子发布的图像、标定、坐标变换、云台遥测、弹丸统计和仿真真值，
 * 接受 RGB8/BGR8 像素并向上层统一返回独立持有的 BGR8 图像。
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
  struct Impl;                  ///< 隔离配置解析和共享内存设备实现。
  std::unique_ptr<Impl> impl_;  ///< 当前相机唯一拥有的设备状态。
};

}  // namespace mv::hal
