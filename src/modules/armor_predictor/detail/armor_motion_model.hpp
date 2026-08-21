#pragma once

#include "geometry/rigid_transform.hpp"
#include "hal/camera/i_camera.hpp"

#include <array>
#include <cmath>
#include <cstddef>

#include <ceres/jet.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

namespace mv::modules::detail {

inline constexpr int K_STATE_SIZE = 13;

namespace state_index {
inline constexpr int CX = 0;
inline constexpr int VX = 1;
inline constexpr int CY = 2;
inline constexpr int VY = 3;
inline constexpr int CZ = 4;
inline constexpr int VZ = 5;
inline constexpr int ROT_X = 6;
inline constexpr int ROT_Y = 7;
inline constexpr int ROT_Z = 8;
inline constexpr int VYAW = 9;
inline constexpr int LOG_R1 = 10;
inline constexpr int LOG_R2 = 11;
inline constexpr int H = 12;
}  // namespace state_index

using ErrorVector = Eigen::Matrix<double, K_STATE_SIZE, 1>;
using StateMatrix = Eigen::Matrix<double, K_STATE_SIZE, K_STATE_SIZE>;

/** @brief 四装甲整车的强类型名义状态；姿态将 car 系向量旋转到 world 系。 */
struct NominalState {
  geometry::Vector3 position_world{geometry::Vector3::Zero()};
  geometry::Vector3 velocity_world{geometry::Vector3::Zero()};
  geometry::Quaternion world_q_car{geometry::Quaternion::Identity()};
  double yaw_velocity_rad_s{0.0};
  double log_radius_1{std::log(0.2)};
  double log_radius_2{std::log(0.2)};
  double height_offset_m{0.0};
};

/** @brief 固定槽位编号及该标签对应的装甲安装倾角。 */
struct ArmorMount {
  int slot{0};
  double tilt_rad{0.0};
};

template <typename T>
struct StateValue {
  Eigen::Matrix<T, 3, 1> position_world{Eigen::Matrix<T, 3, 1>::Zero()};
  Eigen::Matrix<T, 3, 1> velocity_world{Eigen::Matrix<T, 3, 1>::Zero()};
  Eigen::Quaternion<T> world_q_car{Eigen::Quaternion<T>::Identity()};
  T yaw_velocity_rad_s{0.0};
  T log_radius_1{0.0};
  T log_radius_2{0.0};
  T height_offset_m{0.0};
};

template <typename T>
struct TransformValue {
  Eigen::Matrix<T, 3, 1> translation{Eigen::Matrix<T, 3, 1>::Zero()};
  Eigen::Quaternion<T> rotation{Eigen::Quaternion<T>::Identity()};
};

template <typename T>
[[nodiscard]] inline double ScalarPart(const T& value) {
  return static_cast<double>(value);
}

template <int N>
[[nodiscard]] inline double ScalarPart(const ceres::Jet<double, N>& value) {
  return value.a;
}

template <typename T>
[[nodiscard]] inline Eigen::Quaternion<T> So3Exp(const Eigen::Matrix<T, 3, 1>& rotation) {
  const T SQUARED_ANGLE = rotation.squaredNorm();
  T real;
  T scale;
  if (ScalarPart(SQUARED_ANGLE) < 1.0e-12) {
    real = T(1.0) - SQUARED_ANGLE / T(8.0);
    scale = T(0.5) - SQUARED_ANGLE / T(48.0);
  } else {
    using ceres::cos;
    using ceres::sin;
    using ceres::sqrt;
    using std::cos;
    using std::sin;
    using std::sqrt;
    const T ANGLE = sqrt(SQUARED_ANGLE);
    const T HALF_ANGLE = ANGLE / T(2.0);
    real = cos(HALF_ANGLE);
    scale = sin(HALF_ANGLE) / ANGLE;
  }
  return Eigen::Quaternion<T>(real, scale * rotation.x(), scale * rotation.y(),
                              scale * rotation.z());
}

template <typename T>
[[nodiscard]] inline Eigen::Matrix<T, 3, 1> So3Log(Eigen::Quaternion<T> rotation) {
  rotation.normalize();
  if (ScalarPart(rotation.w()) < 0.0)
    rotation.coeffs() *= T(-1.0);
  const Eigen::Matrix<T, 3, 1> VECTOR = rotation.vec();
  const T NORM = VECTOR.norm();
  if (ScalarPart(NORM) < 1.0e-8)
    return T(2.0) * VECTOR;
  using ceres::atan2;
  using std::atan2;
  return (T(2.0) * atan2(NORM, rotation.w()) / NORM) * VECTOR;
}

template <typename T>
[[nodiscard]] inline StateValue<T> CastState(const NominalState& state) {
  StateValue<T> result;
  result.position_world = state.position_world.template cast<T>();
  result.velocity_world = state.velocity_world.template cast<T>();
  result.world_q_car = state.world_q_car.template cast<T>();
  result.yaw_velocity_rad_s = T(state.yaw_velocity_rad_s);
  result.log_radius_1 = T(state.log_radius_1);
  result.log_radius_2 = T(state.log_radius_2);
  result.height_offset_m = T(state.height_offset_m);
  return result;
}

template <typename T>
inline void Inject(const Eigen::Matrix<T, K_STATE_SIZE, 1>& error, StateValue<T>& state) {
  state.position_world += Eigen::Matrix<T, 3, 1>(error[state_index::CX], error[state_index::CY],
                                                 error[state_index::CZ]);
  state.velocity_world += Eigen::Matrix<T, 3, 1>(error[state_index::VX], error[state_index::VY],
                                                 error[state_index::VZ]);
  const Eigen::Matrix<T, 3, 1> ROTATION_ERROR(error[state_index::ROT_X], error[state_index::ROT_Y],
                                              error[state_index::ROT_Z]);
  state.world_q_car = state.world_q_car * So3Exp(ROTATION_ERROR);
  state.world_q_car.normalize();
  state.yaw_velocity_rad_s += error[state_index::VYAW];
  state.log_radius_1 += error[state_index::LOG_R1];
  state.log_radius_2 += error[state_index::LOG_R2];
  state.height_offset_m += error[state_index::H];
}

/** @brief 将误差状态右乘注入 double 名义状态。 */
void Inject(const ErrorVector& error, NominalState& state);

template <typename T>
[[nodiscard]] inline Eigen::Matrix<T, K_STATE_SIZE, 1> BoxMinus(const StateValue<T>& reference,
                                                                const StateValue<T>& value) {
  Eigen::Matrix<T, K_STATE_SIZE, 1> result;
  result.setZero();
  const auto POSITION = value.position_world - reference.position_world;
  const auto VELOCITY = value.velocity_world - reference.velocity_world;
  result[state_index::CX] = POSITION.x();
  result[state_index::VX] = VELOCITY.x();
  result[state_index::CY] = POSITION.y();
  result[state_index::VY] = VELOCITY.y();
  result[state_index::CZ] = POSITION.z();
  result[state_index::VZ] = VELOCITY.z();
  const auto ROTATION = So3Log(reference.world_q_car.conjugate() * value.world_q_car);
  result[state_index::ROT_X] = ROTATION.x();
  result[state_index::ROT_Y] = ROTATION.y();
  result[state_index::ROT_Z] = ROTATION.z();
  result[state_index::VYAW] = value.yaw_velocity_rad_s - reference.yaw_velocity_rad_s;
  result[state_index::LOG_R1] = value.log_radius_1 - reference.log_radius_1;
  result[state_index::LOG_R2] = value.log_radius_2 - reference.log_radius_2;
  result[state_index::H] = value.height_offset_m - reference.height_offset_m;
  return result;
}

/** @brief 计算 value 相对 reference 的13维局部误差。 */
[[nodiscard]] ErrorVector BoxMinus(const NominalState& reference, const NominalState& value);

template <typename T>
[[nodiscard]] inline StateValue<T> PredictState(const StateValue<T>& state, double dt) {
  StateValue<T> result = state;
  result.position_world += state.velocity_world * T(dt);
  result.world_q_car = state.world_q_car * So3Exp(Eigen::Matrix<T, 3, 1>(
                                               T(0.0), T(0.0), state.yaw_velocity_rad_s * T(dt)));
  result.world_q_car.normalize();
  return result;
}

/** @brief 按匀速/匀角速模型外推名义状态。 */
[[nodiscard]] NominalState PredictState(const NominalState& state, double dt);

template <typename T>
[[nodiscard]] inline T RadiusForSlot(const StateValue<T>& state, int slot) {
  using ceres::exp;
  using std::exp;
  return exp((slot & 1) == 0 ? state.log_radius_1 : state.log_radius_2);
}

template <typename T>
[[nodiscard]] inline TransformValue<T> CarTArmor(const StateValue<T>& state, ArmorMount mount) {
  constexpr double HALF_PI = 1.57079632679489661923;
  const T ANGLE = T(static_cast<double>(mount.slot) * HALF_PI);
  using ceres::cos;
  using ceres::sin;
  using std::cos;
  using std::sin;
  const T RADIUS = RadiusForSlot(state, mount.slot);
  TransformValue<T> result;
  result.translation = {RADIUS * cos(ANGLE), RADIUS * sin(ANGLE),
                        (mount.slot & 1) == 0 ? T(0.0) : state.height_offset_m};

  Eigen::Matrix<T, 3, 3> base_rotation;
  base_rotation.col(0) = Eigen::Matrix<T, 3, 1>(T(0.0), T(1.0), T(0.0));
  base_rotation.col(1) = Eigen::Matrix<T, 3, 1>(T(0.0), T(0.0), T(1.0));
  base_rotation.col(2) = Eigen::Matrix<T, 3, 1>(T(1.0), T(0.0), T(0.0));
  const auto SLOT_ROTATION = So3Exp(Eigen::Matrix<T, 3, 1>(T(0.0), T(0.0), ANGLE));
  const auto TILT_ROTATION = So3Exp(Eigen::Matrix<T, 3, 1>(T(mount.tilt_rad), T(0.0), T(0.0)));
  result.rotation = SLOT_ROTATION * Eigen::Quaternion<T>(base_rotation) * TILT_ROTATION;
  result.rotation.normalize();
  return result;
}

template <typename T>
[[nodiscard]] inline TransformValue<T> WorldTArmor(const StateValue<T>& state, ArmorMount mount) {
  const auto CAR_T_ARMOR = CarTArmor(state, mount);
  return {.translation = state.position_world + state.world_q_car * CAR_T_ARMOR.translation,
          .rotation = state.world_q_car * CAR_T_ARMOR.rotation};
}

template <typename T>
[[nodiscard]] inline TransformValue<T> InverseTransform(const TransformValue<T>& transform) {
  const auto INVERSE_ROTATION = transform.rotation.conjugate();
  return {.translation = -(INVERSE_ROTATION * transform.translation), .rotation = INVERSE_ROTATION};
}

template <typename T>
[[nodiscard]] inline TransformValue<T> ComposeTransform(const TransformValue<T>& parent_t_middle,
                                                        const TransformValue<T>& middle_t_child) {
  return {.translation =
              parent_t_middle.translation + parent_t_middle.rotation * middle_t_child.translation,
          .rotation = parent_t_middle.rotation * middle_t_child.rotation};
}

template <typename T>
[[nodiscard]] inline TransformValue<T> CastTransform(const geometry::RigidTransform& transform) {
  return {.translation = transform.translation.template cast<T>(),
          .rotation = transform.rotation.template cast<T>()};
}

template <typename T>
[[nodiscard]] inline Eigen::Matrix<T, 2, 1> ProjectPoint(
    const Eigen::Matrix<T, 3, 1>& point_camera, const hal::CameraFrame::Calibration& calibration) {
  const T X = point_camera.x() / point_camera.z();
  const T Y = point_camera.y() / point_camera.z();
  const T R2 = X * X + Y * Y;
  const T R4 = R2 * R2;
  const T R6 = R4 * R2;
  const T RADIAL = T(1.0) + T(calibration.distortion[0]) * R2 + T(calibration.distortion[1]) * R4 +
                   T(calibration.distortion[4]) * R6;
  const T DISTORTED_X = X * RADIAL + T(2.0 * calibration.distortion[2]) * X * Y +
                        T(calibration.distortion[3]) * (R2 + T(2.0) * X * X);
  const T DISTORTED_Y = Y * RADIAL + T(calibration.distortion[2]) * (R2 + T(2.0) * Y * Y) +
                        T(2.0 * calibration.distortion[3]) * X * Y;
  return {T(calibration.fx) * DISTORTED_X + T(calibration.cx),
          T(calibration.fy) * DISTORTED_Y + T(calibration.cy)};
}

template <typename T>
[[nodiscard]] inline std::array<Eigen::Matrix<T, 2, 1>, 4> ProjectArmorCorners(
    const StateValue<T>& state, ArmorMount mount, hal::CameraFrame::ArmorType type,
    const hal::CameraFrame::FrameGeometry& geometry) {
  const double WIDTH = type == hal::CameraFrame::ArmorType::LARGE ? 0.225 : 0.135;
  constexpr double HEIGHT = 0.055;
  const std::array<Eigen::Matrix<T, 3, 1>, 4> CORNERS{
      Eigen::Matrix<T, 3, 1>(T(-0.5 * WIDTH), T(0.5 * HEIGHT), T(0.0)),
      Eigen::Matrix<T, 3, 1>(T(0.5 * WIDTH), T(0.5 * HEIGHT), T(0.0)),
      Eigen::Matrix<T, 3, 1>(T(0.5 * WIDTH), T(-0.5 * HEIGHT), T(0.0)),
      Eigen::Matrix<T, 3, 1>(T(-0.5 * WIDTH), T(-0.5 * HEIGHT), T(0.0))};
  const auto WORLD_T_CAMERA = ComposeTransform(CastTransform<T>(geometry.world_t_gimbal),
                                               CastTransform<T>(geometry.gimbal_t_camera_optical));
  const auto CAMERA_T_WORLD = InverseTransform(WORLD_T_CAMERA);
  const auto CAMERA_T_ARMOR = ComposeTransform(CAMERA_T_WORLD, WorldTArmor(state, mount));
  std::array<Eigen::Matrix<T, 2, 1>, 4> result{};
  for (std::size_t index = 0; index < CORNERS.size(); ++index) {
    const Eigen::Matrix<T, 3, 1> POINT_CAMERA =
        CAMERA_T_ARMOR.translation + CAMERA_T_ARMOR.rotation * CORNERS[index];
    result[index] = ProjectPoint(POINT_CAMERA, geometry.calibration);
  }
  return result;
}

/** @brief 将名义状态序列化为稳定的13维诊断顺序。 */
[[nodiscard]] std::array<double, K_STATE_SIZE> DiagnosticState(const NominalState& state);

/** @brief 生成指定槽位的 world_T_armor。 */
[[nodiscard]] geometry::RigidTransform WorldArmorPose(const NominalState& state, ArmorMount mount);

/** @brief 返回车体 +X 轴在 world XY 平面的航向角。 */
[[nodiscard]] double HeadingYaw(const NominalState& state) noexcept;

}  // namespace mv::modules::detail
