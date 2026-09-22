#pragma once

#include "geometry/rigid_transform.hpp"

#include <cmath>

namespace mv::geometry {

/** @brief 最终 pitch 刚体的固定安装外参；平移单位为米，旋转方向为 child 到 gimbal。 */
struct GimbalExtrinsics {
  Quaternion gimbal_q_imu{Quaternion::Identity()};  ///< IMU 解算机体系到云台系的安装旋转。
  RigidTransform gimbal_t_camera_optical;          ///< 相机光学系到云台系的安装变换。
  RigidTransform gimbal_t_muzzle;                  ///< 枪口系到云台系的安装变换。
};

/**
 * @brief 把 IMU 解算机体系角速度转换到云台前、左、上坐标系，单位保持 rad/s。
 * @param imu_angular_velocity 原始机体系角速度；不能传入已经转换过的云台角速度。
 * @param extrinsics 已校验的固定安装外参，与姿态转换使用同一个 R_GI。
 */
[[nodiscard]] inline Vector3 ResolveGimbalAngularVelocity(
    const Vector3& imu_angular_velocity, const GimbalExtrinsics& extrinsics) noexcept {
  return extrinsics.gimbal_q_imu * imu_angular_velocity;
}

/** @brief 检查矩阵是否为有限、正交且行列式为 +1 的三维旋转矩阵。 */
[[nodiscard]] inline bool IsRotationMatrix(const Eigen::Matrix3d& rotation) noexcept {
  return rotation.allFinite() &&
         (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(rotation.determinant() - 1.0) <= 1e-6;
}

/**
 * @brief 根据 IMU 姿态和固定安装旋转，计算仅补偿旋转的云台到 world 变换。
 * @param world_q_imu 已校验的单位四元数，将 IMU 解算机体系旋转到惯性参考系。
 * @param extrinsics 已校验的固定安装外参。
 * @return 平移为零的云台变换，旋转满足 R_WG = R_WI * R_GI^T。
 *
 * IMU 与相机固定在最终 pitch 刚体上，姿态已包含串联关节的共同转动，不再叠加关节角。
 * 此处 world 只有惯性方向，原点采用随云台参考点移动的近似，不表示导航位置。
 */
[[nodiscard]] inline RigidTransform ResolveRotationOnlyWorldTGimbal(
    const Quaternion& world_q_imu, const GimbalExtrinsics& extrinsics) noexcept {
  return {.translation = Vector3::Zero(),
          .rotation = (world_q_imu * extrinsics.gimbal_q_imu.conjugate()).normalized()};
}

}  // namespace mv::geometry
