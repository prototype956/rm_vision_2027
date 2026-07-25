/**
 * @file mindvision_camera.hpp
 * @brief MindVision 工业相机的 Pimpl 封装
 *
 * 【为什么用 Pimpl 而不是继承后暴露所有成员？】
 *
 *   MindVision SDK 头文件（CameraApi.h）包含大量平台相关类型（tSdkXxx）。
 *   如果这些类型出现在 .hpp 的 private 区，所有 include 此头文件的翻译单元
 *   都必须能看到 CameraApi.h —— 哪怕它们根本不关心工业相机。
 *
 *   Pimpl 把 SDK 类型完全隔离在 .cpp 里：
 *   - 编译时间：修改 SDK 使用方式只需重编 mindvision_camera.cpp，
 *     而不是所有包含此头文件的模块；
 *   - 可测试性：在没有 MindVision SDK 的 CI 机器上，
 *     只要不链接 mindvision_camera.cpp，其他模块仍可编译；
 *   - 干净依赖：HAL 接口头文件对 SDK 零依赖。
 *
 * 【YAML 配置字段（MindVision）】
 * @code
 *   camera:
 *     exposure_us:  5000       # 曝光时间（微秒）
 *     resolution:
 *       width:  1280
 *       height: 800
 *     channel: 3               # 图像通道数（3=BGR，1=灰度）
 * @endcode
 */
#pragma once

#include "i_camera.hpp"

#include <memory>

namespace mv::hal {

/**
 * @brief MindVision 工业相机实现（Pimpl 模式）
 *
 * 持有 `std::unique_ptr<Impl>`，所有 SDK 调用均在 Impl 中完成，
 * 头文件对 CameraApi.h 零依赖。
 *
 * 使用示例：
 * @code
 *   auto cam = std::make_unique<MindVisionCamera>();
 *   if (cam->Open(cfg.Subtree("camera"))) {
 *     cv::Mat frame;
 *     while (running && cam->Grab(frame)) {
 *       process(frame);
 *     }
 *   }
 * @endcode
 */
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
  bool Grab(cv::Mat& frame) override;
  [[nodiscard]] bool IsOpen() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;  // 所有 SDK 相关成员都在 Impl 里
};

}  // namespace mv::hal
