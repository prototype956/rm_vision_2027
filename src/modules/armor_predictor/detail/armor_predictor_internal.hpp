#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"

#include <cstddef>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

namespace mv::modules::detail {

inline constexpr int K_STATE_SIZE = 11;       ///< EKF 状态维数。
inline constexpr int K_MEASUREMENT_SIZE = 4;  ///< 单块装甲量测维数。

/** @brief x,vx,y,vy,z,vz,yaw,vyaw,r,dr,dz 顺序的状态向量。 */
using StateVector = Eigen::Matrix<double, K_STATE_SIZE, 1>;
using StateMatrix = Eigen::Matrix<double, K_STATE_SIZE, K_STATE_SIZE>;

/** @brief 从正式 PnP 结果提取并转换到 world 坐标系的内部观测。 */
struct Observation {
  std::size_t input_index{0};            ///< 原始检测索引。
  ArmorLabel label{ArmorLabel::SENTRY};  ///< 网络识别的机器人标签。
  hal::CameraFrame::ArmorType type{hal::CameraFrame::ArmorType::SMALL};  ///< 装甲物理尺寸。
  geometry::RigidTransform world_t_armor;  ///< armor 到 world 的 PnP 观测变换。
  cv::Point2f image_center{};  ///< 初始化时用于主点距离排序的图像中心。
  double yaw{0.0};             ///< 装甲法向在 world XY 平面的航向角。
};

}  // namespace mv::modules::detail
