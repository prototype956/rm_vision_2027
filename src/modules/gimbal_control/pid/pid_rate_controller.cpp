#include "modules/gimbal_control/pid/pid_rate_controller.hpp"

#include "core/config.hpp"

#include <cmath>

#include <numbers>

namespace mv::modules {
namespace {
VariableIntegralRatePidConfig ParseAxis(const YAML::Node& node) {
  constexpr char CONTEXT[] = "gimbal PID axis";
  ConfigLoader::RequireMap(node, CONTEXT);
  ConfigLoader::RejectUnknownKeys(
      node,
      {"kp", "ki", "kd", "integral_full_error_rad", "integral_stop_error_rad",
       "integral_limit_rad_s", "derivative_filter_tau_s", "deadband_rad"},
      CONTEXT);
  VariableIntegralRatePidConfig config;
  config.kp = ConfigLoader::Require<double>(node, "kp", CONTEXT);
  config.ki = ConfigLoader::Require<double>(node, "ki", CONTEXT);
  config.kd = ConfigLoader::Require<double>(node, "kd", CONTEXT);
  config.integral_full_error_rad =
      ConfigLoader::Require<double>(node, "integral_full_error_rad", CONTEXT);
  config.integral_stop_error_rad =
      ConfigLoader::Require<double>(node, "integral_stop_error_rad", CONTEXT);
  config.integral_limit_rad_s =
      ConfigLoader::Require<double>(node, "integral_limit_rad_s", CONTEXT);
  config.derivative_filter_tau_s =
      ConfigLoader::Require<double>(node, "derivative_filter_tau_s", CONTEXT);
  config.deadband_rad = ConfigLoader::Require<double>(node, "deadband_rad", CONTEXT);
  for (const double VALUE : {config.kp, config.ki, config.kd, config.integral_full_error_rad,
                             config.integral_stop_error_rad, config.integral_limit_rad_s,
                             config.derivative_filter_tau_s, config.deadband_rad}) {
    if (!std::isfinite(VALUE) || VALUE < 0.0)
      throw ConfigError("gimbal PID values must be finite and nonnegative");
  }
  if (config.integral_full_error_rad < config.deadband_rad ||
      config.integral_stop_error_rad <= config.integral_full_error_rad)
    throw ConfigError("gimbal PID requires deadband <= integral_full < integral_stop");
  return config;
}
}  // namespace

PidRateControllerConfig ParsePidRateControllerConfig(const YAML::Node& root) {
  ConfigLoader::RejectUnknownKeys(root, {"schema_version", "yaw", "pitch"}, "gimbal PID");
  if (ConfigLoader::Require<int>(root, "schema_version", "gimbal PID") != 1)
    throw ConfigError("gimbal PID schema_version must be 1");
  return {ParseAxis(root["yaw"]), ParseAxis(root["pitch"])};
}

void PidRateController::Reset() noexcept {
  yaw_.Reset();
  pitch_.Reset();
}

void PidRateController::ApplyTuning(PidRateControllerConfig config,
                                    GimbalRateLimits limits) noexcept {
  if (config.yaw != config_.yaw || limits.max_yaw_rate_rad_s != limits_.max_yaw_rate_rad_s)
    yaw_.Reset();
  if (config.pitch != config_.pitch || limits.max_pitch_rate_rad_s != limits_.max_pitch_rate_rad_s)
    pitch_.Reset();
  config_ = config;
  limits_ = limits;
}

PidRateResult PidRateController::Step(const hal::GimbalCommand& target,
                                      const hal::GimbalActuatorTelemetry& actual,
                                      double dt) noexcept {
  if (!target.valid || !actual.valid || !std::isfinite(target.yaw) ||
      !std::isfinite(target.pitch) || !std::isfinite(actual.actual_yaw) ||
      !std::isfinite(actual.actual_pitch) || !std::isfinite(actual.yaw_velocity) ||
      !std::isfinite(actual.pitch_velocity) || !std::isfinite(dt)) {
    Reset();
    return {};
  }
  PidRateResult result;
  result.yaw = yaw_.Step(std::remainder(target.yaw - actual.actual_yaw, 2.0 * std::numbers::pi),
                         actual.yaw_velocity, dt, limits_.max_yaw_rate_rad_s, config_.yaw);
  result.pitch = pitch_.Step(target.pitch - actual.actual_pitch, actual.pitch_velocity, dt,
                             limits_.max_pitch_rate_rad_s, config_.pitch);
  result.command = {true, result.yaw.rate_command, result.pitch.rate_command};
  return result;
}
}  // namespace mv::modules
