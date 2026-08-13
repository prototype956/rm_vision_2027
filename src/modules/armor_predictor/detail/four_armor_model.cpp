#include "modules/armor_predictor/detail/four_armor_model.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>
#include <numbers>

namespace mv::modules::detail {
namespace {

Eigen::Matrix<double, 3, K_STATE_SIZE> ArmorPositionJacobian(const StateVector& state, int slot) {
  Eigen::Matrix<double, 3, K_STATE_SIZE> jacobian = Eigen::Matrix<double, 3, K_STATE_SIZE>::Zero();
  const bool alternate = (slot % 2) != 0;
  const double angle = WrapAngle(state[6] + static_cast<double>(slot) * std::numbers::pi / 2.0);
  const double radius = state[8] + (alternate ? state[9] : 0.0);
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  jacobian(0, 0) = 1.0;
  jacobian(1, 2) = 1.0;
  jacobian(2, 4) = 1.0;
  jacobian(0, 6) = -radius * sine;
  jacobian(1, 6) = radius * cosine;
  jacobian(0, 8) = cosine;
  jacobian(1, 8) = sine;
  if (alternate) {
    jacobian(0, 9) = cosine;
    jacobian(1, 9) = sine;
    jacobian(2, 10) = 1.0;
  }
  return jacobian;
}

}  // namespace

double WrapAngle(double angle) noexcept {
  return std::remainder(angle, 2.0 * std::numbers::pi);
}

geometry::Vector3 WorldArmorPosition(const StateVector& state, int slot) {
  const bool alternate = (slot % 2) != 0;
  const double angle = WrapAngle(state[6] + static_cast<double>(slot) * std::numbers::pi / 2.0);
  // 奇偶槽位分别使用 r 和 r+dr，并允许奇数槽位存在共同高度差 dz。
  const double radius = state[8] + (alternate ? state[9] : 0.0);
  const geometry::Vector3 center(state[0], state[2], state[4]);
  const geometry::Vector3 normal(std::cos(angle), std::sin(angle), 0.0);
  return center + radius * normal + geometry::Vector3(0.0, 0.0, alternate ? state[10] : 0.0);
}

geometry::RigidTransform WorldArmorPose(const StateVector& state, int slot, double armor_roll_rad) {
  const double angle = WrapAngle(state[6] + static_cast<double>(slot) * std::numbers::pi / 2.0);
  const geometry::Vector3 normal(std::cos(angle), std::sin(angle), 0.0);
  const geometry::Vector3 up = geometry::Vector3::UnitZ();
  const geometry::Vector3 x_axis = up.cross(normal).normalized();
  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = up;
  rotation.col(2) = normal;
  const geometry::Quaternion base_rotation(rotation);
  const geometry::Quaternion mount_roll(
      Eigen::AngleAxisd(armor_roll_rad, geometry::Vector3::UnitX()));
  return {.translation = WorldArmorPosition(state, slot),
          .rotation = (base_rotation * mount_roll).normalized()};
}

MeasurementModel ModelMeasurement(const StateVector& state, int slot,
                                  const hal::CameraFrame::FrameGeometry& geometry) {
  const auto ARMOR_POSITION_WORLD = WorldArmorPosition(state, slot);
  const auto GIMBAL_T_WORLD = geometry::Inverse(geometry.world_t_gimbal);
  const auto POSITION_GIMBAL = geometry::TransformPoint(GIMBAL_T_WORLD, ARMOR_POSITION_WORLD);
  const double x = POSITION_GIMBAL.x();
  const double y = POSITION_GIMBAL.y();
  const double z = POSITION_GIMBAL.z();
  const double horizontal_squared = x * x + y * y;
  const double horizontal = std::sqrt(horizontal_squared);
  const double distance_squared = horizontal_squared + z * z;
  const double distance = std::sqrt(distance_squared);
  if (horizontal < 1.0e-8 || distance < 1.0e-8) {
    throw std::runtime_error("predicted armor is at singular gimbal spherical coordinate");
  }

  MeasurementModel result;
  result.value << std::atan2(y, x), std::atan2(z, horizontal), distance,
      WrapAngle(state[6] + static_cast<double>(slot) * std::numbers::pi / 2.0);

  // 链式求导：world 装甲位置 -> gimbal 笛卡尔坐标 -> 球坐标量测。
  Eigen::Matrix3d spherical_jacobian;
  spherical_jacobian << -y / horizontal_squared, x / horizontal_squared, 0.0,
      -x * z / (horizontal * distance_squared), -y * z / (horizontal * distance_squared),
      horizontal / distance_squared, x / distance, y / distance, z / distance;
  result.jacobian.setZero();
  result.jacobian.template topRows<3>() = spherical_jacobian *
                                          GIMBAL_T_WORLD.rotation.toRotationMatrix() *
                                          ArmorPositionJacobian(state, slot);
  result.jacobian(3, 6) = 1.0;
  return result;
}

Eigen::Matrix<double, K_MEASUREMENT_SIZE, 1> ObservePose(
    const geometry::RigidTransform& world_t_armor,
    const hal::CameraFrame::FrameGeometry& geometry) {
  const auto GIMBAL_T_WORLD = geometry::Inverse(geometry.world_t_gimbal);
  const auto POSITION = geometry::TransformPoint(GIMBAL_T_WORLD, world_t_armor.translation);
  const double horizontal = std::hypot(POSITION.x(), POSITION.y());
  const auto NORMAL = geometry::TransformVector(world_t_armor, geometry::Vector3::UnitZ());
  return {std::atan2(POSITION.y(), POSITION.x()), std::atan2(POSITION.z(), horizontal),
          POSITION.norm(), std::atan2(NORMAL.y(), NORMAL.x())};
}

}  // namespace mv::modules::detail
