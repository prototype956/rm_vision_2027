#pragma once

#include "hal/camera/i_camera.hpp"

#include <memory>

namespace mv::hal {

/**
 * @brief 基于 cv::VideoCapture 的通用相机后端。
 *
 * source 可以是 V4L2 设备索引，也可以是视频文件或流地址。Open() 会请求配置中的
 * 分辨率和帧率，并拒绝后端未采用目标分辨率的设备。输出固定为 BGR8。
 */
class OpenCvCamera : public ICamera {
 public:
  OpenCvCamera();
  ~OpenCvCamera() override;

  OpenCvCamera(const OpenCvCamera&) = delete;
  OpenCvCamera& operator=(const OpenCvCamera&) = delete;
  OpenCvCamera(OpenCvCamera&& other);
  OpenCvCamera& operator=(OpenCvCamera&& other) noexcept;

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
