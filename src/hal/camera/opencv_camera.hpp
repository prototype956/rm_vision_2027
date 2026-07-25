/**
 * @file opencv_camera.hpp
 * @brief 基于 OpenCV VideoCapture 的相机封装（Pimpl 模式）
 *
 * 【适用场景】
 *   - USB 摄像头（V4L2 设备，如 /dev/video0）
 *   - 本地视频文件（用于离线调试和回放）
 *   - RTSP 网络摄像头
 *
 * 【与 MindVisionCamera 的区别】
 *   MindVisionCamera 直接使用 SDK DMA buffer，帧延迟更低，适合比赛环境；
 *   OpenCvCamera 通过 V4L2 → OpenCV 路径，兼容性更强，适合调试和仿真。
 *   两者对上层暴露完全相同的 ICamera 接口，切换时业务代码零改动。
 *
 * 【YAML 配置字段（OpenCV 模式）】
 * @code
 *   camera:
 *     source: 0           # 设备索引（int）或视频路径（string）
 *     width:  1280        # 期望分辨率（硬件可能不支持，会降级）
 *     height: 720
 *     fps:    60          # 期望帧率（仅作 hint，实际由 V4L2 决定）
 * @endcode
 */
#pragma once

#include "i_camera.hpp"

#include <memory>

namespace mv::hal {

/**
 * @brief OpenCV VideoCapture 的 ICamera 包装
 *
 * 持有 `std::unique_ptr<Impl>`，cv::VideoCapture 完全隐藏在 .cpp 里，
 * 头文件不引入任何 OpenCV 大型头文件，编译时间不受影响。
 */
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
  bool Grab(cv::Mat& frame) override;
  [[nodiscard]] bool IsOpen() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::hal
