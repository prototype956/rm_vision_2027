#pragma once

#include "modules/armor_predictor/detail/armor_predictor_internal.hpp"

namespace mv::modules::detail {

/** @brief 单槽位在方位、俯仰、距离和装甲 yaw 空间中的非线性量测模型。 */
struct MeasurementModel {
  Eigen::Matrix<double, K_MEASUREMENT_SIZE, 1> value;  ///< 当前状态对应的预测量测。
  Eigen::Matrix<double, K_MEASUREMENT_SIZE, K_STATE_SIZE> jacobian;  ///< 对状态的 Jacobian。
};

/** @brief 将角度归一化到 [-pi, pi]。 */
[[nodiscard]] double WrapAngle(double angle) noexcept;
/** @brief 由车辆中心状态计算指定四装甲槽位的世界坐标。 */
[[nodiscard]] geometry::Vector3 WorldArmorPosition(const StateVector& state, int slot);
/** @brief 由车辆状态和安装滚转角生成指定槽位的完整世界系位姿。 */
[[nodiscard]] geometry::RigidTransform WorldArmorPose(const StateVector& state, int slot,
                                                      double armor_roll_rad);
/** @brief 在 gimbal 球坐标量测空间计算指定槽位的预测值与 Jacobian。 */
[[nodiscard]] MeasurementModel ModelMeasurement(const StateVector& state, int slot,
                                                const hal::CameraFrame::FrameGeometry& geometry);
/** @brief 将世界系装甲位姿转换为方位、俯仰、距离和 yaw 实际量测。 */
[[nodiscard]] Eigen::Matrix<double, K_MEASUREMENT_SIZE, 1> ObservePose(
    const geometry::RigidTransform& world_t_armor, const hal::CameraFrame::FrameGeometry& geometry);

}  // namespace mv::modules::detail
