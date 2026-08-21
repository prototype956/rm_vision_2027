#include "modules/armor_predictor/detail/error_state_ekf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <optional>

namespace mv::modules::detail {
namespace {

using Jet = ceres::Jet<double, K_STATE_SIZE>;

Eigen::Matrix3d Skew(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d result;
  result << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(), -vector.y(), vector.x(),
      0.0;
  return result;
}

bool FiniteState(const NominalState& state) {
  return state.position_world.allFinite() && state.velocity_world.allFinite() &&
         state.world_q_car.coeffs().allFinite() && std::isfinite(state.yaw_velocity_rad_s) &&
         std::isfinite(state.log_radius_1) && std::isfinite(state.log_radius_2) &&
         std::isfinite(state.height_offset_m);
}

StateMatrix PredictionJacobian(const NominalState& state, double dt,
                               const NominalState& predicted) {
  Eigen::Matrix<Jet, K_STATE_SIZE, 1> error;
  for (int index = 0; index < K_STATE_SIZE; ++index) {
    error[index] = Jet(0.0);
    error[index].v[index] = 1.0;
  }
  auto perturbed = CastState<Jet>(state);
  Inject(error, perturbed);
  const auto PREDICTED_PERTURBED = PredictState(perturbed, dt);
  const auto LOCAL_ERROR = BoxMinus(CastState<Jet>(predicted), PREDICTED_PERTURBED);
  StateMatrix jacobian;
  for (int row = 0; row < K_STATE_SIZE; ++row)
    jacobian.row(row) = LOCAL_ERROR[row].v.transpose();
  return jacobian;
}

StateMatrix YawAccelerationNoise(double dt, double variance) {
  StateMatrix noise = StateMatrix::Zero();
  noise(state_index::ROT_Z, state_index::ROT_Z) = 0.25 * dt * dt * dt * dt * variance;
  noise(state_index::ROT_Z, state_index::VYAW) = 0.5 * dt * dt * dt * variance;
  noise(state_index::VYAW, state_index::ROT_Z) = 0.5 * dt * dt * dt * variance;
  noise(state_index::VYAW, state_index::VYAW) = dt * dt * variance;
  return noise;
}

StateMatrix ProcessNoise(const NominalState& state, double dt, const ArmorPredictorConfig& config,
                         double yaw_acceleration_variance) {
  StateMatrix noise = StateMatrix::Zero();
  const Eigen::Matrix3d BODY_VARIANCE =
      Eigen::Vector3d(config.body_acceleration_variance[0], config.body_acceleration_variance[1],
                      config.body_acceleration_variance[2])
          .asDiagonal();
  const Eigen::Matrix3d ROTATION = state.world_q_car.toRotationMatrix();
  const Eigen::Matrix3d WORLD_VARIANCE = ROTATION * BODY_VARIANCE * ROTATION.transpose();
  const std::array<int, 3> POSITION_INDICES{state_index::CX, state_index::CY, state_index::CZ};
  const std::array<int, 3> VELOCITY_INDICES{state_index::VX, state_index::VY, state_index::VZ};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const double VARIANCE = WORLD_VARIANCE(row, column);
      noise(POSITION_INDICES[row], POSITION_INDICES[column]) += 0.25 * dt * dt * dt * dt * VARIANCE;
      noise(POSITION_INDICES[row], VELOCITY_INDICES[column]) += 0.5 * dt * dt * dt * VARIANCE;
      noise(VELOCITY_INDICES[row], POSITION_INDICES[column]) += 0.5 * dt * dt * dt * VARIANCE;
      noise(VELOCITY_INDICES[row], VELOCITY_INDICES[column]) += dt * dt * VARIANCE;
    }
  }
  noise += YawAccelerationNoise(dt, yaw_acceleration_variance);
  noise(state_index::ROT_X, state_index::ROT_X) +=
      dt * config.roll_pitch_random_walk_variance_per_s;
  noise(state_index::ROT_Y, state_index::ROT_Y) +=
      dt * config.roll_pitch_random_walk_variance_per_s;
  const double RADIUS_1 = std::exp(state.log_radius_1);
  const double RADIUS_2 = std::exp(state.log_radius_2);
  noise(state_index::LOG_R1, state_index::LOG_R1) +=
      dt * config.radius_random_walk_variance_m2_per_s / (RADIUS_1 * RADIUS_1);
  noise(state_index::LOG_R2, state_index::LOG_R2) +=
      dt * config.radius_random_walk_variance_m2_per_s / (RADIUS_2 * RADIUS_2);
  noise(state_index::H, state_index::H) += dt * config.height_random_walk_variance_m2_per_s;
  return noise;
}

struct StackedObservation {
  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd noise;
};

std::optional<StackedObservation> StackObservations(std::span<const ImageObservation> observations,
                                                    const NominalState& state,
                                                    const ArmorPredictorConfig& config) {
  std::vector<LinearizedObservation> values;
  values.reserve(observations.size());
  Eigen::Index dimension = 0;
  for (const auto& observation : observations) {
    auto value = LinearizeObservation(observation, state, config);
    if (!value.residual.allFinite() || !value.jacobian.allFinite() ||
        !value.covariance.allFinite()) {
      return std::nullopt;
    }
    dimension += value.residual.size();
    values.push_back(std::move(value));
  }
  if (dimension == 0)
    return std::nullopt;
  StackedObservation result;
  result.residual.resize(dimension);
  result.jacobian.resize(dimension, K_STATE_SIZE);
  result.noise = Eigen::MatrixXd::Zero(dimension, dimension);
  Eigen::Index offset = 0;
  for (const auto& value : values) {
    const Eigen::Index SIZE = value.residual.size();
    result.residual.segment(offset, SIZE) = value.residual;
    result.jacobian.middleRows(offset, SIZE) = value.jacobian;
    result.noise.block(offset, offset, SIZE, SIZE) = value.covariance;
    offset += SIZE;
  }
  return result;
}

}  // namespace

void ArmorEsekf::Reset() noexcept {
  initialized_ = false;
  state_ = {};
  covariance_.setIdentity();
}

void ArmorEsekf::Initialize(const NominalState& state, const ArmorPredictorConfig& config) {
  state_ = state;
  state_.world_q_car.normalize();
  covariance_.setZero();
  for (int index = 0; index < K_STATE_SIZE; ++index)
    covariance_(index, index) = config.initial_covariance_diagonal[index];
  initialized_ = FiniteState(state_) && covariance_.allFinite();
}

bool ArmorEsekf::Predict(double dt, const ArmorPredictorConfig& config,
                         double yaw_acceleration_variance) {
  if (!initialized_ || !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(yaw_acceleration_variance) || yaw_acceleration_variance < 0.0) {
    return false;
  }
  const NominalState PREDICTED = PredictState(state_, dt);
  const StateMatrix TRANSITION = PredictionJacobian(state_, dt, PREDICTED);
  covariance_ = TRANSITION * covariance_ * TRANSITION.transpose() +
                ProcessNoise(state_, dt, config, yaw_acceleration_variance);
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  state_ = PREDICTED;
  return !Diverged(config);
}

bool ArmorEsekf::AddYawAccelerationNoise(double dt, double variance) {
  if (!initialized_ || !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(variance) ||
      variance < 0.0) {
    return false;
  }
  covariance_ += YawAccelerationNoise(dt, variance);
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  return covariance_.allFinite() && covariance_.diagonal().minCoeff() >= -1.0e-9;
}

bool ArmorEsekf::Update(std::span<const ImageObservation> observations,
                        const ArmorPredictorConfig& config, EsekfUpdateDiagnostic& diagnostic) {
  diagnostic = {};
  if (!initialized_ || observations.empty())
    return false;

  const NominalState PRIOR = state_;
  const StateMatrix PRIOR_COVARIANCE = covariance_;
  const auto PRIOR_OBSERVATION = StackObservations(observations, PRIOR, config);
  if (!PRIOR_OBSERVATION) {
    return false;
  }
  const Eigen::MatrixXd PRIOR_INNOVATION_COVARIANCE =
      PRIOR_OBSERVATION->jacobian * PRIOR_COVARIANCE * PRIOR_OBSERVATION->jacobian.transpose() +
      PRIOR_OBSERVATION->noise;
  Eigen::LDLT<Eigen::MatrixXd> prior_decomposition(PRIOR_INNOVATION_COVARIANCE);
  if (prior_decomposition.info() != Eigen::Success || !prior_decomposition.isPositive()) {
    return false;
  }
  const Eigen::VectorXd PRIOR_SOLVED = prior_decomposition.solve(PRIOR_OBSERVATION->residual);
  const double PRIOR_NIS = PRIOR_OBSERVATION->residual.dot(PRIOR_SOLVED);
  if (!PRIOR_SOLVED.allFinite() || !std::isfinite(PRIOR_NIS)) {
    return false;
  }
  diagnostic.innovation.assign(
      PRIOR_OBSERVATION->residual.data(),
      PRIOR_OBSERVATION->residual.data() + PRIOR_OBSERVATION->residual.size());
  diagnostic.residual_dimension = static_cast<int>(PRIOR_OBSERVATION->residual.size());
  diagnostic.nis = PRIOR_NIS;

  ErrorVector correction = ErrorVector::Zero();
  std::optional<StackedObservation> stacked;
  Eigen::MatrixXd gain;
  Eigen::LDLT<Eigen::MatrixXd> decomposition;
  for (int iteration = 0; iteration < config.esekf_iterations; ++iteration) {
    NominalState iterate = PRIOR;
    Inject(correction, iterate);
    stacked = StackObservations(observations, iterate, config);
    if (!stacked) {
      return false;
    }
    const Eigen::MatrixXd INNOVATION_COVARIANCE =
        stacked->jacobian * PRIOR_COVARIANCE * stacked->jacobian.transpose() + stacked->noise;
    decomposition.compute(INNOVATION_COVARIANCE);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive())
      return false;
    gain = decomposition.solve(stacked->jacobian * PRIOR_COVARIANCE).transpose();
    if (!gain.allFinite())
      return false;
    const ErrorVector NEXT = gain * (stacked->residual + stacked->jacobian * correction);
    diagnostic.iteration_count = iteration + 1;
    if (!NEXT.allFinite())
      return false;
    const bool CONVERGED = (NEXT - correction).norm() < 1.0e-6;
    correction = NEXT;
    if (CONVERGED)
      break;
  }

  NominalState posterior = PRIOR;
  Inject(correction, posterior);
  if (!FiniteState(posterior))
    return false;

  // 以最终迭代点重新线性化，使诊断、Joseph 更新与最终后验状态一致。
  stacked = StackObservations(observations, posterior, config);
  if (!stacked) {
    return false;
  }
  const Eigen::MatrixXd INNOVATION_COVARIANCE =
      stacked->jacobian * PRIOR_COVARIANCE * stacked->jacobian.transpose() + stacked->noise;
  decomposition.compute(INNOVATION_COVARIANCE);
  if (decomposition.info() != Eigen::Success || !decomposition.isPositive())
    return false;
  gain = decomposition.solve(stacked->jacobian * PRIOR_COVARIANCE).transpose();
  const StateMatrix IDENTITY = StateMatrix::Identity();
  const StateMatrix JOSEPH_LEFT = IDENTITY - gain * stacked->jacobian;
  StateMatrix posterior_covariance = JOSEPH_LEFT * PRIOR_COVARIANCE * JOSEPH_LEFT.transpose() +
                                     gain * stacked->noise * gain.transpose();
  StateMatrix reset_jacobian = IDENTITY;
  reset_jacobian.block<3, 3>(state_index::ROT_X, state_index::ROT_X) -=
      0.5 * Skew(correction.segment<3>(state_index::ROT_X));
  covariance_ = reset_jacobian * posterior_covariance * reset_jacobian.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  state_ = posterior;
  diagnostic.yaw_velocity_update_rad_s = posterior.yaw_velocity_rad_s - PRIOR.yaw_velocity_rad_s;
  return !Diverged(config);
}

bool ArmorEsekf::Diverged(const ArmorPredictorConfig& config) const noexcept {
  if (!initialized_ || !FiniteState(state_) || !covariance_.allFinite())
    return true;
  const double RADIUS_1 = std::exp(state_.log_radius_1);
  const double RADIUS_2 = std::exp(state_.log_radius_2);
  if (!std::isfinite(RADIUS_1) || !std::isfinite(RADIUS_2) || RADIUS_1 < config.min_radius_m ||
      RADIUS_1 > config.max_radius_m || RADIUS_2 < config.min_radius_m ||
      RADIUS_2 > config.max_radius_m ||
      std::abs(state_.height_offset_m) > config.max_height_offset_m ||
      std::abs(state_.yaw_velocity_rad_s) > config.max_yaw_velocity_rad_s) {
    return true;
  }
  return covariance_.diagonal().minCoeff() < -1.0e-9;
}

}  // namespace mv::modules::detail
