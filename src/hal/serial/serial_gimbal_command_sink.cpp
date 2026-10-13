#include "hal/serial/serial_gimbal_command_sink.hpp"

#include "core/config.hpp"

#include <cmath>
#include <utility>

namespace mv::hal {
namespace {
std::uint64_t SystemNowNs() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}
}  // namespace

SerialGimbalCommandSinkConfig ParseSerialGimbalCommandSinkConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "serial gimbal transport";
  ConfigLoader::RejectUnknownKeys(
      root, {"schema_version", "enabled", "fire_output_enabled", "max_feedback_age_ms"}, CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1)
    throw ConfigError("serial gimbal transport schema_version must be 1");
  SerialGimbalCommandSinkConfig config;
  config.enabled = ConfigLoader::Require<bool>(root, "enabled", CONTEXT);
  config.fire_output_enabled = ConfigLoader::Require<bool>(root, "fire_output_enabled", CONTEXT);
  config.max_feedback_age_ms = ConfigLoader::Require<double>(root, "max_feedback_age_ms", CONTEXT);
  if (!std::isfinite(config.max_feedback_age_ms) || config.max_feedback_age_ms < 20.0 ||
      config.max_feedback_age_ms > 100.0)
    throw ConfigError("serial gimbal feedback age must be in [20, 100] ms");
  return config;
}

SerialGimbalCommandSink::SerialGimbalCommandSink(SerialImu& imu,
                                                 geometry::GimbalExtrinsics extrinsics,
                                                 SerialGimbalCommandSinkConfig config)
    : imu_(imu), extrinsics_(std::move(extrinsics)), config_(config) {}

bool SerialGimbalCommandSink::IsHealthy() const noexcept {
  if (!config_.enabled)
    return false;
  const auto HEALTH = imu_.Health();
  return HEALTH.connected && HEALTH.synchronized && HEALTH.sample_age_ms >= 0.0 &&
         HEALTH.sample_age_ms <= config_.max_feedback_age_ms;
}

bool SerialGimbalCommandSink::ExternalControlEnabled() const noexcept {
  return config_.enabled && IsHealthy();
}

std::uint64_t SerialGimbalCommandSink::HeartbeatTimestampNs() const noexcept {
  const auto OBSERVATION = imu_.Latest();
  return OBSERVATION ? OBSERVATION->system_timestamp_ns : 0U;
}

GimbalActuatorTelemetry SerialGimbalCommandSink::ActuatorTelemetry() const noexcept {
  GimbalActuatorTelemetry result;
  const auto OBSERVATION = imu_.Latest();
  const auto HEALTH = imu_.Health();
  if (!OBSERVATION || !HEALTH.connected || !HEALTH.synchronized || HEALTH.sample_age_ms < 0.0 ||
      HEALTH.sample_age_ms > config_.max_feedback_age_ms)
    return result;

  const auto WORLD_T_GIMBAL =
      geometry::ResolveRotationOnlyWorldTGimbal(OBSERVATION->world_q_imu, extrinsics_);
  const auto FORWARD = WORLD_T_GIMBAL.rotation * geometry::Vector3::UnitX();
  const auto GYRO = geometry::ResolveGimbalAngularVelocity(OBSERVATION->gyro_rad_s, extrinsics_);
  if (!FORWARD.allFinite() || FORWARD.norm() < 1.0e-9 || !GYRO.allFinite())
    return result;

  result.valid = true;
  result.state_timestamp_ns = OBSERVATION->system_timestamp_ns;
  result.mode = GimbalActuatorMode::PHYSICAL;
  result.actual_yaw = std::atan2(FORWARD.y(), FORWARD.x());
  result.actual_pitch = std::atan2(FORWARD.z(), std::hypot(FORWARD.x(), FORWARD.y()));
  result.yaw_velocity = GYRO.z();
  result.pitch_velocity = -GYRO.y();
  {
    std::lock_guard lock(mutex_);
    result.command_valid = command_valid_;
    result.consumed_command_timestamp_ns = last_command_.timestamp_ns;
    result.consumed_at_timestamp_ns = consumed_at_timestamp_ns_;
    result.target_yaw = last_command_.yaw;
    result.target_pitch = last_command_.pitch;
    result.serial_friction_queued = last_command_.friction_wheel;
    result.serial_feeder_queued = last_command_.fire;
  }
  return result;
}

bool SerialGimbalCommandSink::SendRate(const GimbalCommand& command,
                                       const GimbalRateCommand& rate) noexcept {
  if (!config_.enabled || !IsHealthy()) {
    if (config_.enabled)
      (void)imu_.QueueGimbalCommand(0.0F, 0.0F, false, false, false);
    std::lock_guard lock(mutex_);
    last_command_ = {};
    command_valid_ = false;
    return false;
  }
  if (!command.valid) {
    const bool FRICTION = config_.fire_output_enabled && command.friction_wheel;
    const bool SENT = imu_.QueueGimbalCommand(0.0F, 0.0F, FRICTION, false, FRICTION);
    std::lock_guard lock(mutex_);
    last_command_ = command;
    last_command_.fire = false;
    last_command_.friction_wheel = SENT && FRICTION;
    command_valid_ = false;
    consumed_at_timestamp_ns_ = SystemNowNs();
    return SENT;
  }
  if (!rate.valid || !std::isfinite(rate.yaw_rate_rad_s) || !std::isfinite(rate.pitch_rate_rad_s) ||
      std::abs(rate.yaw_rate_rad_s) > 4.0 || std::abs(rate.pitch_rate_rad_s) > 4.0 ||
      !std::isfinite(command.yaw) || !std::isfinite(command.pitch) ||
      !std::isfinite(command.yaw_velocity) || !std::isfinite(command.pitch_velocity) ||
      !std::isfinite(command.target_distance_m) || command.target_distance_m < 0.0)
    return false;
  const bool SENT = imu_.QueueGimbalCommand(
      static_cast<float>(rate.yaw_rate_rad_s), static_cast<float>(rate.pitch_rate_rad_s), true,
      command.fire && config_.fire_output_enabled,
      (command.friction_wheel || command.fire) && config_.fire_output_enabled);
  if (SENT) {
    std::lock_guard lock(mutex_);
    last_command_ = command;
    last_command_.fire = command.fire && config_.fire_output_enabled;
    last_command_.friction_wheel =
        (command.friction_wheel || command.fire) && config_.fire_output_enabled;
    command_valid_ = true;
    consumed_at_timestamp_ns_ = SystemNowNs();
  }
  return SENT;
}
}  // namespace mv::hal
