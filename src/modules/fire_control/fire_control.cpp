#include "modules/fire_control/fire_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include <numbers>

namespace mv::modules {
namespace {

double Wrap(double angle) noexcept {
  return std::remainder(angle, 2.0 * std::numbers::pi);
}
double UnwrapNear(double angle, double reference) noexcept {
  return reference + Wrap(angle - reference);
}

std::uint64_t SystemNowNs() noexcept {
  const auto COUNT = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
  return COUNT > 0 ? static_cast<std::uint64_t>(COUNT) : 0;
}

double ViewAngle(const PredictedArmorPose& armor, const geometry::Vector3& muzzle) noexcept {
  const auto TO_MUZZLE = muzzle - armor.world_t_armor.translation;
  if (!TO_MUZZLE.allFinite() || TO_MUZZLE.norm() < 1.0e-9) {
    return std::numeric_limits<double>::infinity();
  }
  const auto NORMAL = armor.world_t_armor.rotation * geometry::Vector3::UnitZ();
  return std::acos(std::clamp(NORMAL.normalized().dot(TO_MUZZLE.normalized()), -1.0, 1.0));
}

// 每轴原始残差拆成状态/输入的原始与对偶四项；取最大值作为跨周期收敛趋势标量。
double MaxResidual(const MpcAxisDiagnostics& value) noexcept {
  return std::max({value.primal_residual_state, value.primal_residual_input,
                   value.dual_residual_state, value.dual_residual_input});
}

}  // namespace

std::string_view FireRejectReasonName(FireRejectReason reason) noexcept {
  switch (reason) {
    case FireRejectReason::NONE:
      return "none";
    case FireRejectReason::AUTO_FIRE_DISABLED:
      return "auto_fire_disabled";
    case FireRejectReason::EXTERNAL_CONTROL_DISABLED:
      return "external_control_disabled";
    case FireRejectReason::TRACK_NOT_CONFIRMED:
      return "track_not_confirmed";
    case FireRejectReason::TEMPORARY_LOSS:
      return "temporary_loss";
    case FireRejectReason::STALE_PREDICTION:
      return "stale_prediction";
    case FireRejectReason::INVALID_FEEDBACK:
      return "invalid_feedback";
    case FireRejectReason::STALE_FEEDBACK:
      return "stale_feedback";
    case FireRejectReason::TALOS_UNHEALTHY:
      return "talos_unhealthy";
    case FireRejectReason::NUMERICAL_INVALID:
      return "numerical_invalid";
    case FireRejectReason::HIGH_UNCERTAINTY:
      return "high_uncertainty";
    case FireRejectReason::NO_SHOOTABLE_ARMOR:
      return "no_shootable_armor";
    case FireRejectReason::BALLISTIC_UNSOLVABLE:
      return "ballistic_unsolvable";
    case FireRejectReason::MPC_FAILED:
      return "mpc_failed";
    case FireRejectReason::AIM_ERROR_TOO_LARGE:
      return "aim_error_too_large";
    case FireRejectReason::AIM_NOT_STABLE:
      return "aim_not_stable";
    case FireRejectReason::COOLDOWN:
      return "cooldown";
  }
  return "unknown";
}

std::string_view ArmorSelectionDecisionName(ArmorSelectionDecision decision) noexcept {
  switch (decision) {
    case ArmorSelectionDecision::NONE:
      return "none";
    case ArmorSelectionDecision::ACQUIRED:
      return "acquired";
    case ArmorSelectionDecision::HELD:
      return "held";
    case ArmorSelectionDecision::PENDING_SWITCH:
      return "pending_switch";
    case ArmorSelectionDecision::SWITCHED:
      return "switched";
    case ArmorSelectionDecision::LOST_ANGLE:
      return "lost_angle";
    case ArmorSelectionDecision::NO_CANDIDATE:
      return "no_candidate";
    case ArmorSelectionDecision::TEMP_LOST_HELD:
      return "temp_lost_held";
    case ArmorSelectionDecision::TEMP_LOST_CLEARED:
      return "temp_lost_cleared";
  }
  return "unknown";
}

std::string_view GimbalCommandSourceName(GimbalCommandSource source) noexcept {
  switch (source) {
    case GimbalCommandSource::MPC:
      return "mpc";
    case GimbalCommandSource::TRAJECTORY_FALLBACK:
      return "trajectory_fallback";
    case GimbalCommandSource::STOP:
      return "stop";
  }
  return "stop";
}

FireControl::FireControl(FireControlConfig config, GimbalTrajectoryPlannerConfig planner_config)
    : config_(std::move(config)), planner_(std::move(planner_config)) {}

void FireControl::ResetSelection() noexcept {
  locked_slot_ = -1;
  pending_slot_ = -1;
  pending_since_ = {};
  ResetFireReadiness();
}

void FireControl::ResetFireReadiness() noexcept {
  last_stable_slot_ = -1;
  stable_cycles_ = 0;
  pulse_until_ = {};
}

void FireControl::RequestPlannerRebase(std::string_view reason) noexcept {
  planner_.RequestWarmStartRebase();
  if (pending_solver_rebase_reason_.find(reason) != std::string::npos)
    return;
  if (!pending_solver_rebase_reason_.empty())
    pending_solver_rebase_reason_.push_back(',');
  pending_solver_rebase_reason_.append(reason);
}

int FireControl::SelectSlot(const ControlInputSnapshot& input,
                            const geometry::Vector3& muzzle_world, double horizon_s,
                            const hal::GimbalFeedback& feedback,
                            std::chrono::steady_clock::time_point now,
                            ArmorSelectionDiagnostics& diagnostics) {
  const int ORIGINAL_LOCKED_SLOT = locked_slot_;
  const auto HORIZON = ExtrapolatePrediction(input.prediction, std::max(0.0, horizon_s));
  diagnostics.horizon_s = HORIZON.seconds;
  diagnostics.switch_confirmation_s = config_.slot_switch_confirmation_s;
  for (int slot = 0; slot < 4; ++slot) {
    const auto& armor = HORIZON.armors[slot];
    const auto DELTA = armor.world_t_armor.translation - muzzle_world;
    const double YAW = std::atan2(DELTA.y(), DELTA.x());
    const double PITCH = std::atan2(DELTA.z(), std::hypot(DELTA.x(), DELTA.y()));
    const double SLEW = std::hypot(Wrap(YAW - feedback.yaw), PITCH - feedback.pitch);
    const double VIEW = ViewAngle(armor, muzzle_world);
    diagnostics.candidates[slot] = {
        .slot = slot,
        .predicted_pose = armor,
        .view_angle_rad = VIEW,
        .slew_angle_rad = SLEW,
        .enter_eligible = std::isfinite(VIEW) && VIEW <= config_.armor_enter_angle_rad,
        .leave_eligible = std::isfinite(VIEW) && VIEW <= config_.armor_leave_angle_rad};
  }
  const auto BEST =
      std::min_element(diagnostics.candidates.begin(), diagnostics.candidates.end(),
                       [](const auto& left, const auto& right) {
                         if (std::abs(left.view_angle_rad - right.view_angle_rad) > 1.0e-9)
                           return left.view_angle_rad < right.view_angle_rad;
                         return left.slew_angle_rad < right.slew_angle_rad;
                       });
  const bool BEST_VALID = BEST != diagnostics.candidates.end() && BEST->enter_eligible;

  // 临时丢失期间不发起新切换，仅在较宽的离开角内维持原锁定，避免外推目标抖动。
  if (input.prediction.state == TrackerState::TEMP_LOST) {
    pending_slot_ = -1;
    pending_since_ = {};
    if (locked_slot_ >= 0 && locked_slot_ < 4 &&
        diagnostics.candidates[static_cast<std::size_t>(locked_slot_)].leave_eligible) {
      diagnostics.decision = ArmorSelectionDecision::TEMP_LOST_HELD;
    } else {
      ResetSelection();
      diagnostics.decision = ArmorSelectionDecision::TEMP_LOST_CLEARED;
    }
    diagnostics.locked_slot = locked_slot_;
    return locked_slot_;
  }

  if (locked_slot_ >= 0 && locked_slot_ < 4) {
    const auto& locked = diagnostics.candidates[static_cast<std::size_t>(locked_slot_)];
    if (locked.leave_eligible) {
      // 新槽位必须同时满足观察角改善量和持续时间，抑制相邻装甲交界处来回切换。
      if (BEST_VALID && BEST->slot != locked_slot_ &&
          BEST->view_angle_rad + config_.slot_switch_improvement_rad <= locked.view_angle_rad) {
        if (pending_slot_ != BEST->slot) {
          pending_slot_ = BEST->slot;
          pending_since_ = now;
        }
        diagnostics.pending_duration_s =
            std::max(0.0, std::chrono::duration<double>(now - pending_since_).count());
        if (diagnostics.pending_duration_s >= config_.slot_switch_confirmation_s) {
          locked_slot_ = BEST->slot;
          pending_slot_ = -1;
          pending_since_ = {};
          diagnostics.switched = true;
          diagnostics.decision = ArmorSelectionDecision::SWITCHED;
        } else {
          diagnostics.decision = ArmorSelectionDecision::PENDING_SWITCH;
        }
      } else {
        pending_slot_ = -1;
        pending_since_ = {};
        diagnostics.decision = ArmorSelectionDecision::HELD;
      }
      diagnostics.locked_slot = locked_slot_;
      diagnostics.pending_slot = pending_slot_;
      return locked_slot_;
    }
    locked_slot_ = -1;
    pending_slot_ = -1;
    pending_since_ = {};
    diagnostics.decision = ArmorSelectionDecision::LOST_ANGLE;
  }

  if (BEST_VALID) {
    locked_slot_ = BEST->slot;
    diagnostics.switched = ORIGINAL_LOCKED_SLOT >= 0 && ORIGINAL_LOCKED_SLOT != locked_slot_;
    diagnostics.decision =
        diagnostics.switched ? ArmorSelectionDecision::SWITCHED : ArmorSelectionDecision::ACQUIRED;
  } else {
    diagnostics.decision = ArmorSelectionDecision::NO_CANDIDATE;
  }
  diagnostics.locked_slot = locked_slot_;
  diagnostics.pending_slot = pending_slot_;
  return locked_slot_;
}

BallisticSolution FireControl::SolveBallistic(const ControlInputSnapshot& input, int slot,
                                              const geometry::RigidTransform& world_t_muzzle,
                                              double base_horizon_s) const {
  BallisticSolution result;
  result.slot = slot;
  double fly_time = 0.0;
  // 目标预测位置依赖飞行时间，飞行时间又依赖目标位置，使用定点迭代联合求解。
  for (int iteration = 0; iteration < config_.ballistic_max_iterations; ++iteration) {
    const double HORIZON_S = base_horizon_s + fly_time;
    if (!std::isfinite(HORIZON_S) || HORIZON_S < 0.0 || HORIZON_S > 2.0)
      return result;
    const auto HORIZON = ExtrapolatePrediction(input.prediction, HORIZON_S);
    const auto TARGET = HORIZON.armors[static_cast<std::size_t>(slot)].world_t_armor.translation;
    const auto DELTA = TARGET - world_t_muzzle.translation;
    const double HORIZONTAL = std::hypot(DELTA.x(), DELTA.y());
    const auto MUZZLE_FORWARD = world_t_muzzle.rotation * geometry::Vector3::UnitX();
    if (!DELTA.allFinite() || HORIZONTAL < 1.0e-6 || MUZZLE_FORWARD.dot(DELTA) <= 0.0)
      return result;
    const double V2 = config_.bullet_speed_mps * config_.bullet_speed_mps;
    const double A = config_.gravity_mps2 * HORIZONTAL * HORIZONTAL / (2.0 * V2);
    const double DISCRIMINANT = HORIZONTAL * HORIZONTAL - 4.0 * A * (A + DELTA.z());
    if (!std::isfinite(DISCRIMINANT) || DISCRIMINANT < 0.0 || A <= 0.0)
      return result;
    const double TANGENT = (HORIZONTAL - std::sqrt(DISCRIMINANT)) / (2.0 * A);
    const double PITCH = std::atan(TANGENT);
    const double COSINE = std::cos(PITCH);
    if (!std::isfinite(PITCH) || COSINE <= 1.0e-6)
      return result;
    const double NEXT_FLY_TIME = HORIZONTAL / (config_.bullet_speed_mps * COSINE);
    result.target_world = TARGET;
    result.yaw = std::atan2(DELTA.y(), DELTA.x());
    result.pitch = PITCH;
    result.distance_m = DELTA.norm();
    result.fly_time_s = NEXT_FLY_TIME;
    if (std::abs(NEXT_FLY_TIME - fly_time) < config_.ballistic_time_tolerance_s) {
      result.valid = true;
      return result;
    }
    fly_time = NEXT_FLY_TIME;
  }
  return result;
}

std::vector<AimReferencePoint> FireControl::BuildReference(
    const ControlInputSnapshot& input, int slot, const geometry::RigidTransform& world_t_muzzle,
    double prediction_age_s, double yaw_anchor, BallisticSolution& current) const {
  const auto& planner_config = planner_.Config();
  std::vector<AimReferencePoint> reference(static_cast<std::size_t>(planner_config.horizon_steps));
  double previous_yaw = yaw_anchor;
  for (int index = 0; index < planner_config.horizon_steps; ++index) {
    const double SAMPLE_TIME = static_cast<double>(index) * planner_config.dt_s;
    auto solution = SolveBallistic(input, slot, world_t_muzzle,
                                   prediction_age_s + config_.command_delay_s + SAMPLE_TIME);
    if (!solution.valid)
      return {};
    if (index == 0)
      current = solution;
    const double YAW = UnwrapNear(solution.yaw, previous_yaw);
    reference[static_cast<std::size_t>(index)].yaw = YAW;
    reference[static_cast<std::size_t>(index)].pitch = solution.pitch;
    previous_yaw = YAW;
  }
  // 由离散弹道角生成 MPC 速度参考；端点使用单边差分，中间点使用中心差分。
  for (int index = 0; index < planner_config.horizon_steps; ++index) {
    const int BEFORE = std::max(0, index - 1);
    const int AFTER = std::min(planner_config.horizon_steps - 1, index + 1);
    const double DENOMINATOR = static_cast<double>(AFTER - BEFORE) * planner_config.dt_s;
    auto& point = reference[static_cast<std::size_t>(index)];
    point.yaw_velocity = (reference[static_cast<std::size_t>(AFTER)].yaw -
                          reference[static_cast<std::size_t>(BEFORE)].yaw) /
                         DENOMINATOR;
    point.pitch_velocity = (reference[static_cast<std::size_t>(AFTER)].pitch -
                            reference[static_cast<std::size_t>(BEFORE)].pitch) /
                           DENOMINATOR;
  }
  return reference;
}

FireControlResult FireControl::Step(const ControlInputSnapshot& input,
                                    const hal::GimbalFeedback& feedback,
                                    std::chrono::steady_clock::time_point now) {
  FireControlResult result;
  result.source_sequence = input.prediction.sequence;
  result.source_capture_timestamp_ns = input.prediction.source_capture_timestamp_ns;
  result.command_timestamp_ns = SystemNowNs();
  result.tracker_state = input.prediction.state;
  result.tracked_label = input.prediction.label;
  result.tracked_type = input.prediction.type;
  result.feedback = feedback;
  result.external_control_enabled = input.external_control_enabled;
  result.auto_fire_enabled = config_.auto_fire;
  result.target_linear_speed_mps =
      std::hypot(std::hypot(input.prediction.state_vector[1], input.prediction.state_vector[3]),
                 input.prediction.state_vector[5]);
  result.target_spin_rate_rad_s = std::abs(input.prediction.state_vector[7]);
  const auto& planner_config = planner_.Config();
  result.trajectory_dt_s = planner_config.dt_s;
  result.bullet_speed_mps = config_.bullet_speed_mps;
  result.max_yaw_velocity_rad_s = planner_config.max_yaw_velocity_rad_s;
  result.max_pitch_velocity_rad_s = planner_config.max_pitch_velocity_rad_s;
  result.max_yaw_acceleration_rad_s2 = planner_config.max_yaw_acceleration_rad_s2;
  result.max_pitch_acceleration_rad_s2 = planner_config.max_pitch_acceleration_rad_s2;
  result.prediction_age_s = std::max(
      0.0,
      std::chrono::duration<double>(now - input.prediction.source_receive_steady_time).count());
  result.feedback_age_s =
      feedback.valid
          ? std::max(0.0, std::chrono::duration<double>(now - feedback.timestamp).count())
          : std::numeric_limits<double>::infinity();
  result.command.timestamp_ns = result.command_timestamp_ns;

  // 标签或物理尺寸变化意味着预测器已切换跟踪对象，不能继承槽位和求解器历史。
  if (tracked_label_ != input.prediction.label || tracked_type_ != input.prediction.type) {
    ResetSelection();
    RequestPlannerRebase("tracking_object_changed");
    previous_reference_valid_ = false;
    result.tracking_object_reset = true;
    tracked_label_ = input.prediction.label;
    tracked_type_ = input.prediction.type;
  }

  if (input.prediction.state == TrackerState::TEMP_LOST) {
    if (temp_lost_since_ == std::chrono::steady_clock::time_point{})
      temp_lost_since_ = now;
  } else {
    temp_lost_since_ = {};
  }
  const double TEMP_LOST_TIME = temp_lost_since_ == std::chrono::steady_clock::time_point{}
                                    ? 0.0
                                    : std::chrono::duration<double>(now - temp_lost_since_).count();
  const bool STATE_ALLOWS_CONTROL = input.prediction.state == TrackerState::DETECTING ||
                                    input.prediction.state == TrackerState::TRACKING ||
                                    (input.prediction.state == TrackerState::TEMP_LOST &&
                                     TEMP_LOST_TIME <= config_.max_temp_lost_control_s);
  if (STATE_ALLOWS_CONTROL && !tracking_input_valid_)
    RequestPlannerRebase("tracking_recovered");
  const bool FEEDBACK_FINITE =
      feedback.valid && std::isfinite(feedback.yaw) && std::isfinite(feedback.yaw_velocity) &&
      std::isfinite(feedback.pitch) && std::isfinite(feedback.pitch_velocity);
  const bool PREDICTION_FINITE =
      std::all_of(input.prediction.state_vector.begin(), input.prediction.state_vector.end(),
                  [](double value) { return std::isfinite(value); }) &&
      std::all_of(input.prediction.covariance_diagonal.begin(),
                  input.prediction.covariance_diagonal.end(),
                  [](double value) { return std::isfinite(value) && value >= 0.0; });
  const bool TRANSFORMS_FINITE = input.world_t_gimbal.translation.allFinite() &&
                                 input.world_t_gimbal.rotation.coeffs().allFinite() &&
                                 input.gimbal_t_muzzle.translation.allFinite() &&
                                 input.gimbal_t_muzzle.rotation.coeffs().allFinite();
  // 控制前置校验失败时不生成云台目标；调用方会把默认 command 当作停止命令发布。
  if (!STATE_ALLOWS_CONTROL || !FEEDBACK_FINITE || !PREDICTION_FINITE || !TRANSFORMS_FINITE ||
      result.feedback_age_s > config_.max_prediction_age_s ||
      result.prediction_age_s > config_.max_prediction_age_s) {
    ResetSelection();
    tracking_input_valid_ = false;
    previous_reference_valid_ = false;
    result.reject_reason =
        !FEEDBACK_FINITE                                       ? FireRejectReason::INVALID_FEEDBACK
        : !PREDICTION_FINITE || !TRANSFORMS_FINITE             ? FireRejectReason::NUMERICAL_INVALID
        : result.feedback_age_s > config_.max_prediction_age_s ? FireRejectReason::STALE_FEEDBACK
        : result.prediction_age_s > config_.max_prediction_age_s
            ? FireRejectReason::STALE_PREDICTION
            : FireRejectReason::TRACK_NOT_CONFIRMED;
    return result;
  }
  tracking_input_valid_ = true;

  const auto WORLD_T_MUZZLE = geometry::Compose(input.world_t_gimbal, input.gimbal_t_muzzle);
  result.world_t_muzzle = WORLD_T_MUZZLE;
  result.muzzle_pose_valid = true;
  const auto CENTER = ExtrapolatePrediction(input.prediction, result.prediction_age_s).center_world;
  const double INITIAL_FLY_TIME =
      (CENTER - WORLD_T_MUZZLE.translation).norm() / config_.bullet_speed_mps;
  const int SLOT = SelectSlot(input, WORLD_T_MUZZLE.translation,
                              result.prediction_age_s + config_.command_delay_s + INITIAL_FLY_TIME,
                              feedback, now, result.armor_selection);
  result.selected_slot = SLOT;
  if (result.armor_selection.switched) {
    RequestPlannerRebase("armor_slot_switched");
    last_stable_slot_ = -1;
    stable_cycles_ = 0;
    pulse_until_ = {};
  }
  if (SLOT < 0) {
    stable_cycles_ = 0;
    pulse_until_ = {};
    result.reject_reason = FireRejectReason::NO_SHOOTABLE_ARMOR;
    return result;
  }

  auto reference = BuildReference(input, SLOT, WORLD_T_MUZZLE, result.prediction_age_s,
                                  feedback.yaw, result.ballistic);
  if (reference.empty() || !result.ballistic.valid) {
    stable_cycles_ = 0;
    pulse_until_ = {};
    result.reject_reason = FireRejectReason::BALLISTIC_UNSOLVABLE;
    return result;
  }
  result.target_yaw = reference.front().yaw;
  result.target_pitch = reference.front().pitch;
  if (reference.size() > 1) {
    const auto& next_reference = reference[1];
    if (previous_reference_valid_) {
      result.reference_step_valid = true;
      result.reference_yaw_step = Wrap(next_reference.yaw - previous_reference_yaw_);
      result.reference_pitch_step = next_reference.pitch - previous_reference_pitch_;
    }
    previous_reference_yaw_ = next_reference.yaw;
    previous_reference_pitch_ = next_reference.pitch;
    previous_reference_valid_ = true;
  }
  result.plan = planner_.Plan(feedback, reference);
  // warm-start 重建原因与残差跨周期变化只用于诊断，不参与本周期轨迹有效性判断。
  if (result.plan.warm_start_action == GimbalWarmStartAction::REBASE) {
    result.solver_warm_start_reset = true;
    result.solver_warm_start_reset_reason = std::move(pending_solver_rebase_reason_);
    pending_solver_rebase_reason_.clear();
    solver_cycles_since_reset_ = 0;
    previous_mpc_failed_ = false;
    previous_solver_residual_valid_ = false;
  }
  result.solver_continued_after_failure = previous_mpc_failed_;
  result.solver_cycles_since_reset = solver_cycles_since_reset_;
  if (previous_solver_residual_valid_) {
    result.previous_solver_residual_valid = true;
    result.previous_yaw_solver_residual = previous_yaw_solver_residual_;
    result.previous_pitch_solver_residual = previous_pitch_solver_residual_;
    result.yaw_solver_residual_delta =
        MaxResidual(result.plan.yaw_solver) - previous_yaw_solver_residual_;
    result.pitch_solver_residual_delta =
        MaxResidual(result.plan.pitch_solver) - previous_pitch_solver_residual_;
  }
  ++solver_cycles_since_reset_;
  if (result.plan.trajectory.size() > 1)
    result.raw_mpc_command = result.plan.trajectory[1];
  result.raw_mpc_valid = result.plan.valid;
  if (!result.plan.valid) {
    previous_mpc_failed_ = true;
    previous_solver_residual_valid_ = true;
    previous_yaw_solver_residual_ = MaxResidual(result.plan.yaw_solver);
    previous_pitch_solver_residual_ = MaxResidual(result.plan.pitch_solver);
    stable_cycles_ = 0;
    pulse_until_ = {};
    result.reject_reason = FireRejectReason::MPC_FAILED;
    return result;
  }
  previous_mpc_failed_ = false;
  previous_solver_residual_valid_ = false;

  result.command = {.valid = true,
                    .fire = false,
                    .timestamp_ns = result.command_timestamp_ns,
                    .yaw = result.plan.command.yaw,
                    .yaw_velocity = result.plan.command.yaw_velocity,
                    .yaw_acceleration = result.plan.command.yaw_acceleration,
                    .pitch = result.plan.command.pitch,
                    .pitch_velocity = result.plan.command.pitch_velocity,
                    .pitch_acceleration = result.plan.command.pitch_acceleration,
                    .target_distance_m = result.ballistic.distance_m};
  result.command_source = GimbalCommandSource::MPC;
  result.yaw_error = Wrap(result.target_yaw - feedback.yaw);
  result.pitch_error = result.target_pitch - feedback.pitch;
  const double WIDTH = input.prediction.type == hal::CameraFrame::ArmorType::LARGE ? 0.225 : 0.135;
  constexpr double HEIGHT = 0.055;
  result.fire_yaw_window =
      std::clamp(std::atan2(0.5 * WIDTH * config_.fire_window_scale, result.ballistic.distance_m),
                 config_.min_fire_yaw_rad, config_.max_fire_yaw_rad);
  result.fire_pitch_window =
      std::clamp(std::atan2(0.5 * HEIGHT * config_.fire_window_scale, result.ballistic.distance_m),
                 config_.min_fire_pitch_rad, config_.max_fire_pitch_rad);

  const double POSITION_STD = std::sqrt(
      std::max({0.0, input.prediction.covariance_diagonal[0],
                input.prediction.covariance_diagonal[2], input.prediction.covariance_diagonal[4]}));
  const double YAW_STD = std::sqrt(std::max(0.0, input.prediction.covariance_diagonal[6]));
  const bool UNCERTAINTY_OK =
      POSITION_STD <= config_.max_center_position_std_m && YAW_STD <= config_.max_yaw_std_rad;
  const bool AIM_OK = std::abs(result.yaw_error) <= result.fire_yaw_window &&
                      std::abs(result.pitch_error) <= result.fire_pitch_window;
  // 稳定计数与槽位绑定；换槽或离开窗口都会重新累计，防止切换瞬间误触发。
  if (SLOT == last_stable_slot_ && AIM_OK) {
    ++stable_cycles_;
  } else {
    last_stable_slot_ = SLOT;
    stable_cycles_ = AIM_OK ? 1 : 0;
  }
  result.stable_cycles = stable_cycles_;

  if (!config_.auto_fire) {
    result.reject_reason = FireRejectReason::AUTO_FIRE_DISABLED;
  } else if (!input.external_control_enabled) {
    result.reject_reason = FireRejectReason::EXTERNAL_CONTROL_DISABLED;
  } else if (input.prediction.state == TrackerState::TEMP_LOST) {
    result.reject_reason = FireRejectReason::TEMPORARY_LOSS;
  } else if (input.prediction.state != TrackerState::TRACKING) {
    result.reject_reason = FireRejectReason::TRACK_NOT_CONFIRMED;
  } else if (!UNCERTAINTY_OK) {
    result.reject_reason = FireRejectReason::HIGH_UNCERTAINTY;
  } else if (!AIM_OK) {
    result.reject_reason = FireRejectReason::AIM_ERROR_TOO_LARGE;
  } else if (stable_cycles_ < config_.stable_cycles) {
    result.reject_reason = FireRejectReason::AIM_NOT_STABLE;
  } else {
    result.fire_eligible = true;
    // 开火采用有宽度的电平脉冲，并以脉冲起点限制最小重复间隔。
    const bool PULSE_ACTIVE =
        pulse_until_ != std::chrono::steady_clock::time_point{} && now < pulse_until_;
    const bool INTERVAL_READY =
        last_fire_start_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::duration<double>(now - last_fire_start_).count() >= config_.fire_interval_s;
    if (PULSE_ACTIVE) {
      result.command.fire = true;
      result.reject_reason = FireRejectReason::NONE;
    } else if (INTERVAL_READY) {
      last_fire_start_ = now;
      pulse_until_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(config_.fire_pulse_width_s));
      result.command.fire = true;
      result.reject_reason = FireRejectReason::NONE;
    } else {
      result.reject_reason = FireRejectReason::COOLDOWN;
    }
  }
  if (!result.fire_eligible)
    pulse_until_ = {};
  return result;
}

}  // namespace mv::modules
