#pragma once

#include "hal/gimbal/gimbal_types.hpp"
#include "modules/gimbal_control/gimbal_rate_limits.hpp"

#include <yaml-cpp/yaml.h>

namespace mv::modules {

/** @brief MPC 轨迹到实机速率的跟随环，保留现有前馈、反馈与增益调度。 */
struct MpcRateFollowerConfig {
  bool operator==(const MpcRateFollowerConfig&) const = default;
  double yaw_kp{6.0};
  double pitch_kp{6.0};
  double pitch_near_target_kp{6.0};
  double pitch_near_target_error_rad{0.0};
  double yaw_velocity_ff_gain{1.0};
  double pitch_velocity_ff_gain{1.0};
  double pitch_velocity_damping_gain{0.0};
};

[[nodiscard]] MpcRateFollowerConfig ParseMpcRateFollowerConfig(const YAML::Node& root);
[[nodiscard]] hal::GimbalRateCommand FollowMpcRate(const hal::GimbalCommand& target,
                                                   const hal::GimbalActuatorTelemetry& actual,
                                                   const MpcRateFollowerConfig& config,
                                                   const GimbalRateLimits& limits) noexcept;

}  // namespace mv::modules
