#include "modules/armor_predictor/detail/armor_ekf.hpp"

#include "modules/armor_predictor/detail/four_armor_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <Eigen/Cholesky>

namespace mv::modules::detail {

void ArmorEkf::Reset() noexcept {
  state_.setZero();
  covariance_.setZero();
}

void ArmorEkf::Initialize(const Observation& observation, const ArmorPredictorConfig& config) {
  const double radius = observation.label == ArmorLabel::ONE ? config.hero_initial_radius_m
                                                             : config.vehicle_initial_radius_m;
  state_.setZero();
  state_[0] = observation.world_t_armor.translation.x() - radius * std::cos(observation.yaw);
  state_[2] = observation.world_t_armor.translation.y() - radius * std::sin(observation.yaw);
  state_[4] = observation.world_t_armor.translation.z();
  state_[6] = WrapAngle(observation.yaw);
  state_[8] = radius;
  covariance_.setZero();
  for (int index = 0; index < K_STATE_SIZE; ++index)
    covariance_(index, index) = config.initial_covariance_diagonal[index];
}

void ArmorEkf::Predict(double dt, const ArmorPredictorConfig& config) {
  StateMatrix transition = StateMatrix::Identity();
  transition(0, 1) = dt;
  transition(2, 3) = dt;
  transition(4, 5) = dt;
  transition(6, 7) = dt;
  StateMatrix process_noise = StateMatrix::Zero();
  const double a = dt * dt * dt * dt / 4.0;
  const double b = dt * dt * dt / 2.0;
  const double c = dt * dt;
  // 白噪声加速度离散化后的 Q 块同时覆盖位置、速度及其互协方差。
  for (const auto& [position, velocity] :
       std::array<std::pair<int, int>, 3>{{{0, 1}, {2, 3}, {4, 5}}}) {
    process_noise(position, position) = a * config.linear_acceleration_variance;
    process_noise(position, velocity) = b * config.linear_acceleration_variance;
    process_noise(velocity, position) = b * config.linear_acceleration_variance;
    process_noise(velocity, velocity) = c * config.linear_acceleration_variance;
  }
  process_noise(6, 6) = a * config.angular_acceleration_variance;
  process_noise(6, 7) = b * config.angular_acceleration_variance;
  process_noise(7, 6) = b * config.angular_acceleration_variance;
  process_noise(7, 7) = c * config.angular_acceleration_variance;
  state_ = transition * state_;
  state_[6] = WrapAngle(state_[6]);
  covariance_ = transition * covariance_ * transition.transpose() + process_noise;
}

void ArmorEkf::Update(std::span<const Observation> observations, std::span<const int> slots,
                      const hal::CameraFrame::FrameGeometry& geometry,
                      const ArmorPredictorConfig& config, ArmorPredictionResult& result) {
  const std::size_t MATCHED = static_cast<std::size_t>(
      std::count_if(slots.begin(), slots.end(), [](int slot) { return slot >= 0; }));
  if (MATCHED == 0)
    return;
  const Eigen::Index rows = static_cast<Eigen::Index>(MATCHED * K_MEASUREMENT_SIZE);
  Eigen::MatrixXd measurement_jacobian = Eigen::MatrixXd::Zero(rows, K_STATE_SIZE);
  Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Zero(rows, rows);
  Eigen::VectorXd innovation = Eigen::VectorXd::Zero(rows);
  Eigen::Index offset = 0;
  // 多块装甲共享同一车辆状态，拼成一次联合更新可保留量测间的状态相关性。
  for (std::size_t index = 0; index < observations.size(); ++index) {
    if (slots[index] < 0)
      continue;
    const auto MODEL = ModelMeasurement(state_, slots[index], geometry);
    const auto MEASURED = ObservePose(observations[index].world_t_armor, geometry);
    Eigen::Matrix<double, K_MEASUREMENT_SIZE, 1> residual = MEASURED - MODEL.value;
    residual[0] = WrapAngle(residual[0]);
    residual[1] = WrapAngle(residual[1]);
    residual[3] = WrapAngle(residual[3]);
    measurement_jacobian.block<K_MEASUREMENT_SIZE, K_STATE_SIZE>(offset, 0) = MODEL.jacobian;
    innovation.segment<K_MEASUREMENT_SIZE>(offset) = residual;
    const double center_yaw = std::atan2(state_[2] - geometry.world_t_gimbal.translation.y(),
                                         state_[0] - geometry.world_t_gimbal.translation.x());
    // 斜视会放大距离不确定性，远距离会降低装甲朝向量测可信度。
    const double view_angle = std::abs(WrapAngle(observations[index].yaw - center_yaw));
    const double distance_variance = config.measurement_variance_base[2] +
                                     config.distance_variance_angle_scale * std::log1p(view_angle);
    const double yaw_variance = config.measurement_variance_base[3] +
                                config.armor_yaw_variance_distance_scale * std::log1p(MEASURED[2]);
    measurement_noise.block<K_MEASUREMENT_SIZE, K_MEASUREMENT_SIZE>(offset, offset) =
        (Eigen::Matrix<double, 4, 1>() << config.measurement_variance_base[0],
         config.measurement_variance_base[1], distance_variance, yaw_variance)
            .finished()
            .asDiagonal();
    offset += K_MEASUREMENT_SIZE;
  }
  const Eigen::MatrixXd innovation_covariance =
      measurement_jacobian * covariance_ * measurement_jacobian.transpose() + measurement_noise;
  const Eigen::LDLT<Eigen::MatrixXd> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success) {
    throw std::runtime_error("armor predictor innovation covariance decomposition failed");
  }
  const Eigen::MatrixXd gain = decomposition.solve(measurement_jacobian * covariance_).transpose();
  result.nis = innovation.dot(decomposition.solve(innovation));
  state_ += gain * innovation;
  state_[6] = WrapAngle(state_[6]);
  // Joseph 形式在有限精度下比简化式更好地保持协方差半正定和对称。
  const StateMatrix identity = StateMatrix::Identity();
  const StateMatrix correction = identity - gain * measurement_jacobian;
  covariance_ = correction * covariance_ * correction.transpose() +
                gain * measurement_noise * gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  result.innovation.assign(innovation.data(), innovation.data() + innovation.size());
}

bool ArmorEkf::Diverged(const ArmorPredictorConfig& config) const noexcept {
  const double primary_radius = state_[8];
  const double alternate_radius = state_[8] + state_[9];
  return !state_.allFinite() || !covariance_.allFinite() || primary_radius <= config.min_radius_m ||
         primary_radius >= config.max_radius_m || alternate_radius <= config.min_radius_m ||
         alternate_radius >= config.max_radius_m;
}

}  // namespace mv::modules::detail
