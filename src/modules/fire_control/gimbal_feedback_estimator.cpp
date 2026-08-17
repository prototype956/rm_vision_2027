#include "modules/fire_control/gimbal_feedback_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <numbers>

namespace mv::modules {
namespace {

double Wrap(double angle) noexcept {
  return std::remainder(angle, 2.0 * std::numbers::pi);
}
double UnwrapNear(double angle, double reference) noexcept {
  return reference + Wrap(angle - reference);
}

}  // namespace

const char* GimbalFeedbackSourceName(GimbalFeedbackSource source) noexcept {
  switch (source) {
    case GimbalFeedbackSource::NONE:
      return "none";
    case GimbalFeedbackSource::MEASUREMENT_INIT:
      return "measurement_init";
    case GimbalFeedbackSource::PUBLISHED_COMMAND:
      return "published_command";
    case GimbalFeedbackSource::HELD_COMMAND:
      return "held_command";
    case GimbalFeedbackSource::CAMERA_FALLBACK:
      return "camera_fallback";
    case GimbalFeedbackSource::ACTUATOR_RUNTIME_DIRECT:
      return "actuator_runtime_direct";
    case GimbalFeedbackSource::ACTUATOR_RUNTIME_HOLD:
      return "actuator_runtime_hold";
  }
  return "none";
}

GimbalFeedbackEstimator::GimbalFeedbackEstimator(double max_yaw_velocity_rad_s,
                                                 double max_pitch_velocity_rad_s)
    : max_yaw_velocity_rad_s_(max_yaw_velocity_rad_s),
      max_pitch_velocity_rad_s_(max_pitch_velocity_rad_s) {}

void GimbalFeedbackEstimator::ObserveActuatorTelemetry(const hal::GimbalActuatorTelemetry& actuator,
                                                       std::chrono::steady_clock::time_point now,
                                                       std::uint64_t system_now_ns) noexcept {
  constexpr std::uint64_t FUTURE_TOLERANCE_NS = 5'000'000;
  constexpr std::uint64_t MAX_AGE_NS = 100'000'000;
  const bool FINITE = std::isfinite(actuator.actual_yaw) && std::isfinite(actuator.actual_pitch) &&
                      std::isfinite(actuator.yaw_velocity) &&
                      std::isfinite(actuator.pitch_velocity);
  const bool TIMESTAMP_VALID = actuator.state_timestamp_ns != 0 &&
                               actuator.state_timestamp_ns <= system_now_ns + FUTURE_TOLERANCE_NS;
  const std::uint64_t AGE_NS = TIMESTAMP_VALID && actuator.state_timestamp_ns <= system_now_ns
                                   ? system_now_ns - actuator.state_timestamp_ns
                                   : 0;
  // IDEAL 模式可直接跟随命令，无需把遥测作为物理反馈；只有新鲜 PHYSICAL 状态优先。
  const bool PHYSICAL = actuator.valid && actuator.mode == hal::GimbalActuatorMode::PHYSICAL;
  if (!PHYSICAL || !FINITE || !TIMESTAMP_VALID || AGE_NS > MAX_AGE_NS) {
    if (runtime_actuator_active_) {
      runtime_actuator_active_ = false;
      runtime_state_timestamp_ns_ = 0;
      runtime_actuator_age_s_ = std::numeric_limits<double>::infinity();
      if (measurement_.valid) {
        state_ = measurement_;
        source_ = GimbalFeedbackSource::CAMERA_FALLBACK;
      } else {
        state_ = {};
        source_ = GimbalFeedbackSource::NONE;
      }
    }
    return;
  }

  runtime_actuator_age_s_ = static_cast<double>(AGE_NS) * 1.0e-9;
  if (runtime_actuator_active_ && actuator.state_timestamp_ns == runtime_state_timestamp_ns_) {
    source_ = GimbalFeedbackSource::ACTUATOR_RUNTIME_HOLD;
    return;
  }

  const auto SAMPLE_TIME = now - std::chrono::nanoseconds(AGE_NS);
  // Unix 时间戳只用于估算年龄，减到本机 steady_clock 后再进入控制时域。
  const double REFERENCE_YAW = state_.valid         ? state_.yaw
                               : measurement_.valid ? measurement_.yaw
                                                    : actuator.actual_yaw;
  state_ = {.valid = true,
            .source_sequence = measurement_.source_sequence,
            .timestamp = SAMPLE_TIME,
            .yaw = UnwrapNear(actuator.actual_yaw, REFERENCE_YAW),
            .yaw_velocity = std::clamp(actuator.yaw_velocity, -max_yaw_velocity_rad_s_,
                                       max_yaw_velocity_rad_s_),
            .pitch = actuator.actual_pitch,
            .pitch_velocity = std::clamp(actuator.pitch_velocity, -max_pitch_velocity_rad_s_,
                                         max_pitch_velocity_rad_s_)};
  runtime_actuator_active_ = true;
  runtime_state_timestamp_ns_ = actuator.state_timestamp_ns;
  command_projection_active_ = false;
  source_ = GimbalFeedbackSource::ACTUATOR_RUNTIME_DIRECT;
}

void GimbalFeedbackEstimator::ObserveMeasurement(
    std::uint64_t sequence, std::chrono::steady_clock::time_point timestamp,
    const geometry::RigidTransform& world_t_gimbal,
    const std::optional<hal::GimbalActuatorTelemetry>& actuator) noexcept {
  const auto FORWARD = world_t_gimbal.rotation * geometry::Vector3::UnitX();
  if (!FORWARD.allFinite() || FORWARD.norm() < 1.0e-9)
    return;
  const double MEASURED_YAW = std::atan2(FORWARD.y(), FORWARD.x());
  const double MEASURED_PITCH = std::atan2(FORWARD.z(), std::hypot(FORWARD.x(), FORWARD.y()));
  const bool PHYSICAL =
      actuator && actuator->valid && actuator->mode == hal::GimbalActuatorMode::PHYSICAL &&
      std::isfinite(actuator->yaw_velocity) && std::isfinite(actuator->pitch_velocity);
  // 相机位姿始终更新独立 measurement_；是否覆盖融合 state_ 由更高优先级来源决定。
  if (!measurement_.valid) {
    measurement_ = {.valid = true,
                    .source_sequence = sequence,
                    .timestamp = timestamp,
                    .yaw = MEASURED_YAW,
                    .yaw_velocity = 0.0,
                    .pitch = MEASURED_PITCH,
                    .pitch_velocity = 0.0};
    last_observation_time_ = timestamp;
    last_measured_yaw_ = MEASURED_YAW;
    last_measured_pitch_ = MEASURED_PITCH;
    if (PHYSICAL) {
      measurement_.yaw_velocity =
          std::clamp(actuator->yaw_velocity, -max_yaw_velocity_rad_s_, max_yaw_velocity_rad_s_);
      measurement_.pitch_velocity = std::clamp(actuator->pitch_velocity, -max_pitch_velocity_rad_s_,
                                               max_pitch_velocity_rad_s_);
    }
    if (runtime_actuator_active_) {
      state_.source_sequence = sequence;
    } else if (!command_projection_active_) {
      state_ = measurement_;
      source_ =
          PHYSICAL ? GimbalFeedbackSource::CAMERA_FALLBACK : GimbalFeedbackSource::MEASUREMENT_INIT;
    }
    return;
  }
  if (sequence == measurement_.source_sequence || timestamp <= last_observation_time_)
    return;
  const double DT = std::chrono::duration<double>(timestamp - last_observation_time_).count();
  const double YAW = UnwrapNear(MEASURED_YAW, last_measured_yaw_);
  measurement_ = {
      .valid = true,
      .source_sequence = sequence,
      .timestamp = timestamp,
      .yaw = YAW,
      .yaw_velocity = std::clamp((YAW - last_measured_yaw_) / DT, -max_yaw_velocity_rad_s_,
                                 max_yaw_velocity_rad_s_),
      .pitch = MEASURED_PITCH,
      .pitch_velocity = std::clamp((MEASURED_PITCH - last_measured_pitch_) / DT,
                                   -max_pitch_velocity_rad_s_, max_pitch_velocity_rad_s_)};
  if (PHYSICAL) {
    measurement_.yaw_velocity =
        std::clamp(actuator->yaw_velocity, -max_yaw_velocity_rad_s_, max_yaw_velocity_rad_s_);
    measurement_.pitch_velocity =
        std::clamp(actuator->pitch_velocity, -max_pitch_velocity_rad_s_, max_pitch_velocity_rad_s_);
  }
  last_observation_time_ = timestamp;
  last_measured_yaw_ = YAW;
  last_measured_pitch_ = MEASURED_PITCH;
  if (runtime_actuator_active_) {
    state_.source_sequence = sequence;
  } else if (!command_projection_active_) {
    state_ = measurement_;
    source_ =
        PHYSICAL ? GimbalFeedbackSource::CAMERA_FALLBACK : GimbalFeedbackSource::MEASUREMENT_INIT;
  }
}

void GimbalFeedbackEstimator::ObservePublishedCommand(
    const hal::GimbalCommand& command, std::chrono::steady_clock::time_point timestamp,
    bool held_command) noexcept {
  if (runtime_actuator_active_ || !command.valid || !measurement_.valid)
    return;
  state_ = {.valid = true,
            .source_sequence = measurement_.source_sequence,
            .timestamp = timestamp,
            .yaw = UnwrapNear(command.yaw, state_.valid ? state_.yaw : measurement_.yaw),
            .yaw_velocity = held_command
                                ? 0.0
                                : std::clamp(command.yaw_velocity, -max_yaw_velocity_rad_s_,
                                             max_yaw_velocity_rad_s_),
            .pitch = command.pitch,
            .pitch_velocity = held_command
                                  ? 0.0
                                  : std::clamp(command.pitch_velocity, -max_pitch_velocity_rad_s_,
                                               max_pitch_velocity_rad_s_)};
  command_projection_active_ = true;
  source_ =
      held_command ? GimbalFeedbackSource::HELD_COMMAND : GimbalFeedbackSource::PUBLISHED_COMMAND;
}

hal::GimbalFeedback GimbalFeedbackEstimator::Estimate(
    std::chrono::steady_clock::time_point now) const noexcept {
  static_cast<void>(now);
  return state_;
}

void GimbalFeedbackEstimator::ClearCommandProjection() noexcept {
  command_projection_active_ = false;
  if (runtime_actuator_active_) {
    source_ = GimbalFeedbackSource::ACTUATOR_RUNTIME_HOLD;
    return;
  }
  if (measurement_.valid) {
    state_ = measurement_;
    source_ = GimbalFeedbackSource::CAMERA_FALLBACK;
  } else {
    state_ = {};
    source_ = GimbalFeedbackSource::NONE;
  }
}

void GimbalFeedbackEstimator::ClearRuntimeActuator() noexcept {
  runtime_actuator_active_ = false;
  runtime_state_timestamp_ns_ = 0;
  runtime_actuator_age_s_ = std::numeric_limits<double>::infinity();
  if (measurement_.valid) {
    state_ = measurement_;
    source_ = GimbalFeedbackSource::CAMERA_FALLBACK;
  } else {
    state_ = {};
    source_ = GimbalFeedbackSource::NONE;
  }
}

void GimbalFeedbackEstimator::Reset() noexcept {
  state_ = {};
  measurement_ = {};
  last_observation_time_ = {};
  last_measured_yaw_ = 0.0;
  last_measured_pitch_ = 0.0;
  source_ = GimbalFeedbackSource::NONE;
  command_projection_active_ = false;
  runtime_actuator_active_ = false;
  runtime_state_timestamp_ns_ = 0;
  runtime_actuator_age_s_ = std::numeric_limits<double>::infinity();
}

}  // namespace mv::modules
