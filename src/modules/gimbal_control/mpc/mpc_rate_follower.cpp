#include "modules/gimbal_control/mpc/mpc_rate_follower.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <cmath>

#include <numbers>

namespace mv::modules {
MpcRateFollowerConfig ParseMpcRateFollowerConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "MPC rate follower";
  ConfigLoader::RejectUnknownKeys(
      root,
      {"yaw_kp", "pitch_kp", "pitch_near_target_kp", "pitch_near_target_error_rad",
       "yaw_velocity_ff_gain", "pitch_velocity_ff_gain", "pitch_velocity_damping_gain"},
      CONTEXT);
  MpcRateFollowerConfig config;
  config.yaw_kp = ConfigLoader::Require<double>(root, "yaw_kp", CONTEXT);
  config.pitch_kp = ConfigLoader::Require<double>(root, "pitch_kp", CONTEXT);
  config.pitch_near_target_kp =
      ConfigLoader::Require<double>(root, "pitch_near_target_kp", CONTEXT);
  config.pitch_near_target_error_rad =
      ConfigLoader::Require<double>(root, "pitch_near_target_error_rad", CONTEXT);
  config.yaw_velocity_ff_gain =
      ConfigLoader::Require<double>(root, "yaw_velocity_ff_gain", CONTEXT);
  config.pitch_velocity_ff_gain =
      ConfigLoader::Require<double>(root, "pitch_velocity_ff_gain", CONTEXT);
  config.pitch_velocity_damping_gain =
      ConfigLoader::Require<double>(root, "pitch_velocity_damping_gain", CONTEXT);
  for (const double VALUE : {config.yaw_kp, config.pitch_kp, config.pitch_near_target_kp,
                             config.pitch_near_target_error_rad, config.yaw_velocity_ff_gain,
                             config.pitch_velocity_ff_gain, config.pitch_velocity_damping_gain}) {
    if (!std::isfinite(VALUE) || VALUE < 0.0)
      throw ConfigError("MPC follower values must be finite and nonnegative");
  }
  if (config.pitch_near_target_kp > config.pitch_kp || config.pitch_near_target_error_rad > 0.1 ||
      config.yaw_velocity_ff_gain > 2.0 || config.pitch_velocity_ff_gain > 2.0 ||
      config.pitch_velocity_damping_gain > 2.0)
    throw ConfigError("MPC follower tuning out of range");
  return config;
}

hal::GimbalRateCommand FollowMpcRate(const hal::GimbalCommand& target,
                                     const hal::GimbalActuatorTelemetry& actual,
                                     const MpcRateFollowerConfig& config,
                                     const GimbalRateLimits& limits) noexcept {
  if (!target.valid || !actual.valid || !std::isfinite(target.yaw) ||
      !std::isfinite(target.pitch) || !std::isfinite(target.yaw_velocity) ||
      !std::isfinite(target.pitch_velocity) || !std::isfinite(actual.actual_yaw) ||
      !std::isfinite(actual.actual_pitch) || !std::isfinite(actual.pitch_velocity))
    return {};
  const double ERROR = target.pitch - actual.actual_pitch;
  const double INNER = std::min(std::abs(ERROR), config.pitch_near_target_error_rad);
  const double PROPORTIONAL = std::copysign(
      config.pitch_near_target_kp * INNER + config.pitch_kp * (std::abs(ERROR) - INNER), ERROR);
  return {true,
          std::clamp(config.yaw_velocity_ff_gain * target.yaw_velocity +
                         config.yaw_kp *
                             std::remainder(target.yaw - actual.actual_yaw, 2.0 * std::numbers::pi),
                     -limits.max_yaw_rate_rad_s, limits.max_yaw_rate_rad_s),
          std::clamp(config.pitch_velocity_ff_gain * target.pitch_velocity + PROPORTIONAL -
                         config.pitch_velocity_damping_gain * actual.pitch_velocity,
                     -limits.max_pitch_rate_rad_s, limits.max_pitch_rate_rad_s)};
}
}  // namespace mv::modules
