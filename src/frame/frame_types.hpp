#pragma once

#include "geometry/rigid_transform.hpp"

#include <array>
#include <chrono>
#include <cstdint>

#include <opencv2/core.hpp>
#include <optional>

namespace mv::frame {

/** @brief 一帧图像的采集时间、顺序和数据源健康标识。 */
struct FrameStamp {
  std::chrono::steady_clock::time_point receive_steady_time{};  ///< HAL 收帧单调时钟。
  std::optional<std::uint64_t> capture_timestamp_ns;  ///< 数据源采集 Unix epoch 纳秒时间。
  std::uint64_t sequence{0};               ///< 本次数据源 Open() 后递增的帧序号。
  std::uint64_t source_invalid_frames{0};  ///< 数据源累计拒绝的无效帧数。
};

/** @brief 独立持有像素数据及其采集标识的一帧图像。 */
struct CapturedFrame {
  cv::Mat image;     ///< 不依赖相机驱动 DMA 缓冲区生命周期的 OpenCV 图像。
  FrameStamp stamp;  ///< 与 image 对应的采集时间和顺序。
};

/** @brief 与当前图像对应的针孔相机内参和 plumb_bob 畸变参数。 */
struct CameraModel {
  std::uint32_t width{0};              ///< 标定适用的图像宽度，单位为像素。
  std::uint32_t height{0};             ///< 标定适用的图像高度，单位为像素。
  double fx{0.0};                      ///< 水平方向焦距，单位为像素。
  double fy{0.0};                      ///< 垂直方向焦距，单位为像素。
  double cx{0.0};                      ///< 主点横坐标，单位为像素。
  double cy{0.0};                      ///< 主点纵坐标，单位为像素。
  std::array<double, 5> distortion{};  ///< 依次为 k1、k2、p1、p2、k3。
};

/** @brief 与图像同帧的 world、gimbal、camera_optical 和 muzzle 变换。 */
struct FrameKinematics {
  geometry::RigidTransform world_t_gimbal;           ///< gimbal 到 world 的变换。
  geometry::RigidTransform gimbal_t_camera_optical;  ///< camera_optical 到 gimbal 的变换。
  geometry::RigidTransform gimbal_t_muzzle;          ///< muzzle 到 gimbal 的变换。
};

/** @brief 算法消费的完整同帧相机模型和平台运动学只读视图。 */
struct SpatialFrameView {
  const CameraModel& calibration;                  ///< camera_optical 对应的相机模型。
  const geometry::RigidTransform& world_t_gimbal;  ///< gimbal 到 world 的变换。
  const geometry::RigidTransform& gimbal_t_camera_optical;  ///< camera_optical 到 gimbal。
  const geometry::RigidTransform& gimbal_t_muzzle;          ///< muzzle 到 gimbal 的变换。
};

}  // namespace mv::frame
