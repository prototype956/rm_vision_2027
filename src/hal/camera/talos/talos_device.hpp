#pragma once

#include "hal/camera/i_camera.hpp"
#include "talos_config.hpp"

#include <memory>

namespace mv::hal::detail {

/**
 * @brief 独占 Talos 共享内存映射及三缓冲消费状态的内部设备对象。
 */
class TalosDevice final {
 public:
  TalosDevice();
  ~TalosDevice();

  TalosDevice(const TalosDevice&) = delete;
  TalosDevice& operator=(const TalosDevice&) = delete;
  TalosDevice(TalosDevice&&) = delete;
  TalosDevice& operator=(TalosDevice&&) = delete;

  bool Open(const TalosConfig& config);
  void Close() noexcept;
  GrabStatus Grab(CameraFrame& frame);

  [[nodiscard]] CameraInfo Info() const;
  [[nodiscard]] bool IsOpen() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::hal::detail
