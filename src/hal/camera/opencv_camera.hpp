#pragma once

#include "i_camera.hpp"

#include <memory>

namespace mv::hal {

class OpenCvCamera : public ICamera {
 public:
  OpenCvCamera();
  ~OpenCvCamera() override;

  OpenCvCamera(const OpenCvCamera&) = delete;
  OpenCvCamera& operator=(const OpenCvCamera&) = delete;
  OpenCvCamera(OpenCvCamera&&) noexcept;
  OpenCvCamera& operator=(OpenCvCamera&&) noexcept;

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
