#include "modules/fire_control/fire_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
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

// 每轴原始残差拆成状态/输入的原始与对偶四项；取最大值作为跨周期收敛趋势标量。
double MaxResidual(const MpcAxisDiagnostics& value) noexcept {
  return std::max({value.primal_residual_state, value.primal_residual_input,
                   value.dual_residual_state, value.dual_residual_input});
}

}  // namespace

std::string_view FireRejectReasonName(FireRejectReason reason) noexcept {
  switch (reason) {
    case FireRejectReason::INVALID_ACTION:
      return "invalid_action";
    case FireRejectReason::REFEREE_INVALID:
      return "referee_invalid";
    case FireRejectReason::REFEREE_STALE:
      return "referee_stale";
    case FireRejectReason::REFEREE_BLOCKED:
      return "referee_blocked";
    case FireRejectReason::PULSE_BUSY:
      return "pulse_busy";
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
    : rule_(config), config_(config), planner_(planner_config) {}

void FireControl::Reset() {
  *this = FireControl(config_, planner_.Config());
}

void FireControl::ResetSelection() noexcept {
  rule_.ResetSelection();
  external_slot_ = -1;
  ResetFireReadiness();
}

void FireControl::ResetFireReadiness() noexcept {
  rule_.ResetFireReadiness();
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
    // 保存与 target_world 严格对应的时域；收敛后的 NEXT_FLY_TIME 仅用于误差判定。
    result.prediction_horizon_s = HORIZON_S;
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
    double prediction_age_s, BallisticSolution& current, double yaw_anchor) const {
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
                                    std::chrono::steady_clock::time_point now,
                                    std::uint64_t command_timestamp_ns,
                                    std::optional<PolicyDecision> decision,
                                    const ControlPolicy& policy) {
  if (decision && policy)
    throw std::invalid_argument("provide a decision or a policy, not both");
  const bool EXTERNAL_POLICY = decision.has_value() || static_cast<bool>(policy);
  if (round_ && *round_ != input.prediction.source_round_id)
    Reset();
  round_ = input.prediction.source_round_id;
  if (external_mode_ != EXTERNAL_POLICY) {
    ResetSelection();
    RequestPlannerRebase("policy_changed");
  }
  external_mode_ = EXTERNAL_POLICY;
  FireControlResult result;
  result.decision = decision;
  auto& output = result.output;
  auto& diagnostics = result.diagnostics;
  output.shot_requested = decision && decision->RequestsShot();
  output.source_sequence = input.prediction.sequence;
  output.source_capture_timestamp_ns = input.prediction.source_capture_timestamp_ns;
  output.command_timestamp_ns = command_timestamp_ns;
  output.tracker_state = input.prediction.state;
  output.tracked_label = input.prediction.label;
  output.tracked_type = input.prediction.type;
  diagnostics.feedback = feedback;
  diagnostics.referee_age_s =
      input.referee.valid && input.referee.received_at <= now
          ? input.referee.age_at_receive_s +
                std::chrono::duration<double>(now - input.referee.received_at).count()
          : std::numeric_limits<double>::infinity();
  output.external_control_enabled = input.external_control_enabled;
  output.auto_fire_enabled = config_.auto_fire;
  diagnostics.target_linear_speed_mps = input.prediction.velocity_world.norm();
  diagnostics.target_spin_rate_rad_s = std::abs(input.prediction.yaw_velocity_rad_s);
  const auto& planner_config = planner_.Config();
  diagnostics.trajectory_dt_s = planner_config.dt_s;
  diagnostics.bullet_speed_mps = config_.bullet_speed_mps;
  diagnostics.max_yaw_velocity_rad_s = planner_config.max_yaw_velocity_rad_s;
  diagnostics.max_pitch_velocity_rad_s = planner_config.max_pitch_velocity_rad_s;
  diagnostics.max_yaw_acceleration_rad_s2 = planner_config.max_yaw_acceleration_rad_s2;
  diagnostics.max_pitch_acceleration_rad_s2 = planner_config.max_pitch_acceleration_rad_s2;
  const auto SOURCE_TIME = input.prediction.source_steady_time
                               ? input.prediction.source_steady_time
                               : (!input.prediction.source_capture_timestamp_ns
                                      ? std::optional(input.prediction.source_receive_steady_time)
                                      : std::nullopt);
  if (SOURCE_TIME && *SOURCE_TIME <= now) {
    output.prediction_age_s =
        std::max(0.0, std::chrono::duration<double>(now - *SOURCE_TIME).count());
  } else {
    output.prediction_age_s = std::numeric_limits<double>::infinity();
  }
  output.feedback_age_s =
      feedback.valid
          ? std::max(0.0, std::chrono::duration<double>(now - feedback.timestamp).count())
          : std::numeric_limits<double>::infinity();
  output.command.timestamp_ns = output.command_timestamp_ns;

  // 标签或物理尺寸变化意味着预测器已切换跟踪对象，不能继承槽位和求解器历史。
  if (tracked_label_ != input.prediction.label || tracked_type_ != input.prediction.type ||
      generation_ != input.prediction.track_generation) {
    ResetSelection();
    RequestPlannerRebase("tracking_object_changed");
    previous_reference_valid_ = false;
    output.tracking_object_reset = true;
    tracked_label_ = input.prediction.label;
    tracked_type_ = input.prediction.type;
    generation_ = input.prediction.track_generation;
  }

  if (input.prediction.state == TrackerState::TEMP_LOST) {
    if (!temp_lost_since_)
      temp_lost_since_ = now;
  } else {
    temp_lost_since_ = {};
  }
  const double TEMP_LOST_TIME =
      !temp_lost_since_ ? 0.0 : std::chrono::duration<double>(now - *temp_lost_since_).count();
  const bool STATE_ALLOWS_CONTROL = input.prediction.state == TrackerState::DETECTING ||
                                    input.prediction.state == TrackerState::TRACKING ||
                                    (input.prediction.state == TrackerState::TEMP_LOST &&
                                     TEMP_LOST_TIME <= config_.max_temp_lost_control_s);
  if (STATE_ALLOWS_CONTROL && !tracking_input_valid_)
    RequestPlannerRebase("tracking_recovered");
  const bool FEEDBACK_FINITE =
      feedback.valid && feedback.timestamp <= now && std::isfinite(feedback.yaw) &&
      std::isfinite(feedback.yaw_velocity) && std::isfinite(feedback.pitch) &&
      std::isfinite(feedback.pitch_velocity);
  const bool PREDICTION_FINITE = !STATE_ALLOWS_CONTROL || ControlValuesFinite(input);
  // 控制前置校验失败时不生成云台目标；调用方会把默认 command 当作停止命令发布。
  if (!STATE_ALLOWS_CONTROL || !FEEDBACK_FINITE || !PREDICTION_FINITE ||
      output.feedback_age_s > config_.max_prediction_age_s ||
      output.prediction_age_s > config_.max_prediction_age_s) {
    ResetSelection();
    tracking_input_valid_ = false;
    previous_reference_valid_ = false;
    output.reject_reason = !FEEDBACK_FINITE     ? FireRejectReason::INVALID_FEEDBACK
                           : !PREDICTION_FINITE ? FireRejectReason::NUMERICAL_INVALID
                           : output.feedback_age_s > config_.max_prediction_age_s
                               ? FireRejectReason::STALE_FEEDBACK
                           : output.prediction_age_s > config_.max_prediction_age_s
                               ? FireRejectReason::STALE_PREDICTION
                               : FireRejectReason::TRACK_NOT_CONFIRMED;
    return result;
  }
  tracking_input_valid_ = true;

  // 在复位和输入校验后决策，避免策略看到旧目标槽位或旧回合的射击间隔。
  if (policy) {
    try {
      decision = policy(input, Observe(input, feedback, now));
    } catch (...) {
      ResetFireReadiness();
      throw;
    }
    result.decision = decision;
    output.shot_requested = decision->RequestsShot();
  }

  const auto WORLD_T_MUZZLE = geometry::Compose(input.world_t_gimbal, input.gimbal_t_muzzle);
  output.world_t_muzzle = WORLD_T_MUZZLE;
  output.muzzle_pose_valid = true;
  const auto CENTER = ExtrapolatePrediction(input.prediction, output.prediction_age_s).center_world;
  const double INITIAL_FLY_TIME =
      (CENTER - WORLD_T_MUZZLE.translation).norm() / config_.bullet_speed_mps;
  int slot = -1;
  if (decision) {
    if (!decision->Valid()) {
      ResetFireReadiness();
      output.reject_reason = FireRejectReason::INVALID_ACTION;
      return result;
    }
    const int OLD_SLOT = external_slot_;
    if (decision->action != 0)
      external_slot_ = decision->Slot();
    slot = external_slot_;
    diagnostics.armor_selection.switched = OLD_SLOT >= 0 && OLD_SLOT != slot;
    diagnostics.armor_selection.locked_slot = slot;
  } else {
    slot = rule_.SelectSlot(input, WORLD_T_MUZZLE.translation,
                            output.prediction_age_s + config_.command_delay_s + INITIAL_FLY_TIME,
                            feedback, now, diagnostics.armor_selection);
  }
  output.selected_slot = slot;
  if (diagnostics.armor_selection.switched) {
    RequestPlannerRebase("armor_slot_switched");
    rule_.ResetFireReadiness();
    if (!decision)
      pulse_until_ = {};
  }
  if (slot < 0) {
    rule_.ResetFireReadiness();
    pulse_until_ = {};
    output.reject_reason = FireRejectReason::NO_SHOOTABLE_ARMOR;
    return result;
  }

  auto reference = BuildReference(input, slot, WORLD_T_MUZZLE, output.prediction_age_s,
                                  output.ballistic, feedback.yaw);
  if (reference.empty() || !output.ballistic.valid) {
    rule_.ResetFireReadiness();
    pulse_until_ = {};
    output.reject_reason = FireRejectReason::BALLISTIC_UNSOLVABLE;
    return result;
  }
  output.target_yaw = reference.front().yaw;
  output.target_pitch = reference.front().pitch;
  if (reference.size() > 1) {
    const auto& next_reference = reference[1];
    if (previous_reference_valid_) {
      diagnostics.reference_step_valid = true;
      diagnostics.reference_yaw_step = Wrap(next_reference.yaw - previous_reference_yaw_);
      diagnostics.reference_pitch_step = next_reference.pitch - previous_reference_pitch_;
    }
    previous_reference_yaw_ = next_reference.yaw;
    previous_reference_pitch_ = next_reference.pitch;
    previous_reference_valid_ = true;
  }
  auto plan = planner_.Plan(feedback, reference);
  output.plan = std::move(plan.output);
  diagnostics.plan = std::move(plan.diagnostics);
  // warm-start 重建原因与残差跨周期变化只用于诊断，不参与本周期轨迹有效性判断。
  if (diagnostics.plan.warm_start_action == GimbalWarmStartAction::REBASE) {
    diagnostics.solver_warm_start_reset = true;
    diagnostics.solver_warm_start_reset_reason = std::move(pending_solver_rebase_reason_);
    pending_solver_rebase_reason_.clear();
    solver_cycles_since_reset_ = 0;
    previous_mpc_failed_ = false;
    previous_solver_residual_valid_ = false;
  }
  diagnostics.solver_continued_after_failure = previous_mpc_failed_;
  diagnostics.solver_cycles_since_reset = solver_cycles_since_reset_;
  if (previous_solver_residual_valid_) {
    diagnostics.previous_solver_residual_valid = true;
    diagnostics.previous_yaw_solver_residual = previous_yaw_solver_residual_;
    diagnostics.previous_pitch_solver_residual = previous_pitch_solver_residual_;
    diagnostics.yaw_solver_residual_delta =
        MaxResidual(diagnostics.plan.yaw_solver) - previous_yaw_solver_residual_;
    diagnostics.pitch_solver_residual_delta =
        MaxResidual(diagnostics.plan.pitch_solver) - previous_pitch_solver_residual_;
  }
  ++solver_cycles_since_reset_;
  if (output.plan.trajectory.size() > 1)
    output.raw_mpc_command = output.plan.trajectory[1];
  output.raw_mpc_valid = output.plan.valid;
  if (!output.plan.valid) {
    previous_mpc_failed_ = true;
    previous_solver_residual_valid_ = true;
    previous_yaw_solver_residual_ = MaxResidual(diagnostics.plan.yaw_solver);
    previous_pitch_solver_residual_ = MaxResidual(diagnostics.plan.pitch_solver);
    rule_.ResetFireReadiness();
    pulse_until_ = {};
    output.reject_reason = FireRejectReason::MPC_FAILED;
    return result;
  }
  previous_mpc_failed_ = false;
  previous_solver_residual_valid_ = false;

  output.command = {.valid = true,
                    .fire = false,
                    .timestamp_ns = output.command_timestamp_ns,
                    .yaw = output.plan.command.yaw,
                    .yaw_velocity = output.plan.command.yaw_velocity,
                    .yaw_acceleration = output.plan.command.yaw_acceleration,
                    .pitch = output.plan.command.pitch,
                    .pitch_velocity = output.plan.command.pitch_velocity,
                    .pitch_acceleration = output.plan.command.pitch_acceleration,
                    .target_distance_m = output.ballistic.distance_m};
  output.command_source = GimbalCommandSource::MPC;
  const auto HEURISTIC =
      decision ? FireRejectReason::NONE : rule_.Evaluate(input, feedback, result);
  const auto REFEREE_REASON = RefereeRejectReason(input.referee, now, config_.max_referee_age_s);
  if (!config_.auto_fire) {
    output.reject_reason = FireRejectReason::AUTO_FIRE_DISABLED;
  } else if (!input.external_control_enabled) {
    output.reject_reason = FireRejectReason::EXTERNAL_CONTROL_DISABLED;
  } else if (input.prediction.state == TrackerState::TEMP_LOST) {
    output.reject_reason = FireRejectReason::TEMPORARY_LOSS;
  } else if (input.prediction.state != TrackerState::TRACKING) {
    output.reject_reason = FireRejectReason::TRACK_NOT_CONFIRMED;
  } else if (REFEREE_REASON != FireRejectReason::NONE) {
    output.reject_reason = REFEREE_REASON;
  } else if (HEURISTIC != FireRejectReason::NONE) {
    output.reject_reason = HEURISTIC;
  } else {
    output.fire_eligible = true;
    // 开火采用有宽度的电平脉冲，并以脉冲起点限制最小重复间隔。
    const bool PULSE_ACTIVE = pulse_until_ && now < *pulse_until_;
    const bool INTERVAL_READY =
        !last_fire_start_ ||
        std::chrono::duration<double>(now - *last_fire_start_).count() >= config_.fire_interval_s;
    output.shot_requested = decision ? decision->RequestsShot() : !PULSE_ACTIVE;
    if (PULSE_ACTIVE) {
      output.command.fire = true;
      output.reject_reason = decision && decision->RequestsShot() ? FireRejectReason::PULSE_BUSY
                                                                  : FireRejectReason::NONE;
    } else if (INTERVAL_READY && output.shot_requested) {
      output.shot_accepted = true;
      last_fire_start_ = now;
      pulse_until_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(config_.fire_pulse_width_s));
      output.command.fire = true;
      output.reject_reason = FireRejectReason::NONE;
    } else {
      output.reject_reason =
          output.shot_requested ? FireRejectReason::COOLDOWN : FireRejectReason::NONE;
    }
  }
  if (!output.fire_eligible)
    pulse_until_ = {};
  return result;
}

}  // namespace mv::modules
