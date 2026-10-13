#include "modules/gimbal_control/gimbal_trajectory_types.hpp"

namespace mv::modules {
std::string_view GimbalTrajectoryFailureReasonName(GimbalTrajectoryFailureReason reason) noexcept {
  switch (reason) {
    case GimbalTrajectoryFailureReason::NONE:
      return "none";
    case GimbalTrajectoryFailureReason::INVALID_FEEDBACK:
      return "invalid_feedback";
    case GimbalTrajectoryFailureReason::INVALID_REFERENCE_SIZE:
      return "invalid_reference_size";
    case GimbalTrajectoryFailureReason::NONFINITE_REFERENCE:
      return "nonfinite_reference";
    case GimbalTrajectoryFailureReason::YAW_SOLVER_FAILED:
      return "yaw_solver_failed";
    case GimbalTrajectoryFailureReason::PITCH_SOLVER_FAILED:
      return "pitch_solver_failed";
    case GimbalTrajectoryFailureReason::BOTH_SOLVERS_FAILED:
      return "both_solvers_failed";
    case GimbalTrajectoryFailureReason::NONFINITE_SOLUTION:
      return "nonfinite_solution";
    case GimbalTrajectoryFailureReason::VELOCITY_LIMIT_VIOLATION:
      return "velocity_limit_violation";
    case GimbalTrajectoryFailureReason::ACCELERATION_LIMIT_VIOLATION:
      return "acceleration_limit_violation";
  }
  return "unknown";
}

std::string_view GimbalTrajectoryAxisName(GimbalTrajectoryAxis axis) noexcept {
  switch (axis) {
    case GimbalTrajectoryAxis::NONE:
      return "none";
    case GimbalTrajectoryAxis::YAW:
      return "yaw";
    case GimbalTrajectoryAxis::PITCH:
      return "pitch";
    case GimbalTrajectoryAxis::BOTH:
      return "both";
  }
  return "unknown";
}

std::string_view GimbalWarmStartActionName(GimbalWarmStartAction action) noexcept {
  switch (action) {
    case GimbalWarmStartAction::NONE:
      return "none";
    case GimbalWarmStartAction::SHIFT:
      return "shift";
    case GimbalWarmStartAction::REBASE:
      return "rebase";
    case GimbalWarmStartAction::SHIFT_AFTER_FAILURE:
      return "shift_after_failure";
  }
  return "unknown";
}

}  // namespace mv::modules
