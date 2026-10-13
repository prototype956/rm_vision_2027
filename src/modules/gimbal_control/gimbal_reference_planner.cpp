#include "modules/gimbal_control/gimbal_reference_planner.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <numbers>

namespace mv::modules {
GimbalReferencePlanner::GimbalReferencePlanner(GimbalControlConfig config) : config_(config) {
  if (!std::isfinite(config_.dt_s) || config_.dt_s <= 0.0 || config_.horizon_steps < 2)
    throw ConfigError("invalid gimbal reference timing");
  if (config_.mode == GimbalControlMode::MPC) {
    if (!config_.mpc)
      throw ConfigError("MPC mode requires planner configuration");
    auto mpc_config = *config_.mpc;
    mpc_config.dt_s = config_.dt_s;
    mpc_config.horizon_steps = config_.horizon_steps;
    mpc_ = std::make_unique<MpcTrajectoryPlanner>(mpc_config);
  }
}

void GimbalReferencePlanner::RequestWarmStartRebase() noexcept {
  if (mpc_)
    mpc_->RequestWarmStartRebase();
}

GimbalTrajectoryResult GimbalReferencePlanner::Plan(const hal::GimbalFeedback& feedback,
                                                    std::span<const AimReferencePoint> reference) {
  if (mpc_)
    return mpc_->Plan(feedback, reference);
  GimbalTrajectoryResult result;
  auto& diagnostics = result.diagnostics;
  if (!feedback.valid) {
    diagnostics.failure_reason = GimbalTrajectoryFailureReason::INVALID_FEEDBACK;
    diagnostics.failure_axis = GimbalTrajectoryAxis::BOTH;
    return result;
  }
  if (reference.size() != static_cast<std::size_t>(config_.horizon_steps)) {
    diagnostics.failure_reason = GimbalTrajectoryFailureReason::INVALID_REFERENCE_SIZE;
    diagnostics.failure_axis = GimbalTrajectoryAxis::BOTH;
    return result;
  }
  diagnostics.reference.assign(reference.begin(), reference.end());
  for (std::size_t i = 0; i < reference.size(); ++i) {
    const auto& point = reference[i];
    if (!std::isfinite(point.yaw) || !std::isfinite(point.pitch) ||
        !std::isfinite(point.yaw_velocity) || !std::isfinite(point.pitch_velocity)) {
      diagnostics.failure_reason = GimbalTrajectoryFailureReason::NONFINITE_REFERENCE;
      diagnostics.failure_axis = GimbalTrajectoryAxis::BOTH;
      diagnostics.failure_index = static_cast<int>(i);
      return result;
    }
    if (previous_reference_.size() == reference.size()) {
      diagnostics.reference_horizon_delta_valid = true;
      diagnostics.max_reference_horizon_delta_yaw = std::max(
          diagnostics.max_reference_horizon_delta_yaw,
          std::abs(std::remainder(point.yaw - previous_reference_[i].yaw, 2.0 * std::numbers::pi)));
      diagnostics.max_reference_horizon_delta_pitch =
          std::max(diagnostics.max_reference_horizon_delta_pitch,
                   std::abs(point.pitch - previous_reference_[i].pitch));
    }
  }
  previous_reference_.assign(reference.begin(), reference.end());
  auto& output = result.output;
  for (const auto& point : reference) {
    output.trajectory.push_back({point.yaw, 0.0, 0.0, point.pitch, 0.0, 0.0});
  }
  // Preserve the 10 ms next-reference command and cached sequence for target-loss fallback.
  output.command_index = 1;
  output.command_lookahead_s = config_.dt_s;
  output.command = output.trajectory[1];
  output.command.yaw = std::remainder(output.command.yaw, 2.0 * std::numbers::pi);
  output.valid = true;
  return result;
}
}  // namespace mv::modules
