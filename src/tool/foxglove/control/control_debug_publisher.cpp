#include "tool/foxglove/control/control_debug_publisher.hpp"

#include "core/logger.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <numbers>

namespace mv::tool::foxglove::control {
namespace {

// 高频 state/tracking 使用 JSON 便于 Foxglove Plot 直接选择字段；低频 trajectory
// 携带完整数组，scene 则提供 world 坐标系中的空间关系和颜色状态提示。
constexpr char K_STATE_TOPIC[] = "/vision/control/state";
constexpr char K_TRACKING_TOPIC[] = "/vision/control/tracking";
constexpr char K_TRAJECTORY_TOPIC[] = "/vision/control/trajectory";
constexpr char K_SCENE_TOPIC[] = "/vision/control/scene";
constexpr char K_STATE_SCHEMA[] = R"json({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object"},"source_sequence":{"type":"integer"},
    "source_capture_timestamp_ns":{"type":["integer","null"]},
    "prediction_age_s":{"type":["number","null"]},"feedback_age_s":{"type":["number","null"]},
    "tracker_state":{"type":"string"},"armor_slot":{"type":"integer"},
    "armor_selection":{"type":"object"},
    "ballistics":{"type":"object"},"angles":{"type":"object"},
    "motion":{"type":"object"},"limits":{"type":"object"},"fire_window":{"type":"object"},
    "command":{"type":"object"},"mpc":{"type":"object"},"fire":{"type":"object"},
    "talos":{"type":"object"},"actuator":{"type":"object"},
    "frame_actuator":{"type":["object","null"]},
    "runtime":{"type":"object"}
  },
  "required":["timestamp","source_sequence","prediction_age_s","feedback_age_s","tracker_state",
              "armor_slot","armor_selection","ballistics","angles","motion","limits",
              "fire_window","mpc","fire","talos","actuator","frame_actuator","runtime"]
})json";
constexpr char K_TRACKING_SCHEMA[] = R"json({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object"},"source_sequence":{"type":"integer"},
    "reference_valid":{"type":"boolean"},"mpc_valid":{"type":"boolean"},
    "published_valid":{"type":"boolean"},"estimated_valid":{"type":"boolean"},
    "measured_valid":{"type":"boolean"},"matched_prior_valid":{"type":"boolean"},
    "yaw_reference_next":{"type":["number","null"]},
    "yaw_mpc_next":{"type":["number","null"]},
    "yaw_published":{"type":["number","null"]},
    "yaw_estimated":{"type":["number","null"]},
    "yaw_measured":{"type":["number","null"]},
    "yaw_matched_prior":{"type":["number","null"]},
    "pitch_reference_next":{"type":["number","null"]},
    "pitch_mpc_next":{"type":["number","null"]},
    "pitch_published":{"type":["number","null"]},
    "pitch_estimated":{"type":["number","null"]},
    "pitch_measured":{"type":["number","null"]},
    "pitch_matched_prior":{"type":["number","null"]},
    "yaw_reference_velocity":{"type":["number","null"]},
    "yaw_mpc_velocity":{"type":["number","null"]},
    "yaw_estimated_velocity":{"type":["number","null"]},
    "yaw_measured_velocity":{"type":["number","null"]},
    "pitch_reference_velocity":{"type":["number","null"]},
    "pitch_mpc_velocity":{"type":["number","null"]},
    "pitch_estimated_velocity":{"type":["number","null"]},
    "pitch_measured_velocity":{"type":["number","null"]},
    "yaw_mpc_acceleration":{"type":["number","null"]},
    "pitch_mpc_acceleration":{"type":["number","null"]},
    "yaw_reference_minus_mpc":{"type":["number","null"]},
    "yaw_mpc_minus_estimated":{"type":["number","null"]},
    "yaw_published_minus_measured":{"type":["number","null"]},
    "yaw_estimated_minus_measured":{"type":["number","null"]},
    "yaw_matched_prior_minus_measured":{"type":["number","null"]},
    "pitch_reference_minus_mpc":{"type":["number","null"]},
    "pitch_mpc_minus_estimated":{"type":["number","null"]},
    "pitch_published_minus_measured":{"type":["number","null"]},
    "pitch_estimated_minus_measured":{"type":["number","null"]},
    "pitch_matched_prior_minus_measured":{"type":["number","null"]},
    "yaw_velocity_utilization":{"type":["number","null"]},
    "pitch_velocity_utilization":{"type":["number","null"]},
    "yaw_acceleration_utilization":{"type":["number","null"]},
    "pitch_acceleration_utilization":{"type":["number","null"]},
    "control_period_s":{"type":"number"},"deadline_lateness_us":{"type":"number"},
    "prediction_age_s":{"type":"number"},"measurement_age_s":{"type":["number","null"]},
    "runtime_actuator_age_s":{"type":["number","null"]},
    "frame_actuator_age_s":{"type":["number","null"]},
    "feedback_projection_dt_s":{"type":"number"},
    "feedback_runtime_state_timestamp_ns":{"type":["integer","null"]},
    "yaw_feedback_minus_runtime_actuator":{"type":["number","null"]},
    "pitch_feedback_minus_runtime_actuator":{"type":["number","null"]},
    "yaw_frame_minus_runtime_actuator":{"type":["number","null"]},
    "pitch_frame_minus_runtime_actuator":{"type":["number","null"]},
    "yaw_frame_acceleration_minus_runtime":{"type":["number","null"]},
    "pitch_frame_acceleration_minus_runtime":{"type":["number","null"]},
    "matched_prior_age_at_capture_s":{"type":["number","null"]},
    "sink_send_time_us":{"type":"number"},"mpc_candidate_valid":{"type":"boolean"},
    "command_source":{"type":"string"},"raw_mpc_valid":{"type":"boolean"},
    "fallback_active":{"type":"boolean"},"fallback_age_s":{"type":"number"},
    "fallback_source_slot":{"type":"integer"},
    "fallback_trajectory_index":{"type":"integer"},
    "fallback_remaining_points":{"type":"integer"},
    "mpc_command_index":{"type":"integer"},"command_lookahead_s":{"type":"number"},
    "mpc_residuals_normalized":{"type":"boolean"},
    "consecutive_mpc_failure_cycles":{"type":"integer"},
    "feedback_source":{"type":"string"},"solver_warm_start_reset":{"type":"boolean"},
    "solver_warm_start_reset_reason":{"type":"string"},
    "solver_continued_after_failure":{"type":"boolean"},
    "solver_cycles_since_reset":{"type":"integer"},
    "previous_yaw_solver_residual":{"type":["number","null"]},
    "previous_pitch_solver_residual":{"type":["number","null"]},
    "yaw_solver_residual_delta":{"type":["number","null"]},
    "pitch_solver_residual_delta":{"type":["number","null"]},
    "fallback_expired_this_cycle":{"type":"boolean"},
    "output_projection_cleared":{"type":"boolean"},
    "output_projection_clear_reason":{"type":"string"},
    "reference_step_valid":{"type":"boolean"},
    "reference_yaw_step":{"type":["number","null"]},
    "reference_pitch_step":{"type":["number","null"]},
    "warm_start_action":{"type":"string"},
    "warm_start_rebase_axes":{"type":"string"},
    "solver_retry_attempted":{"type":"boolean"},
    "solver_retry_axes":{"type":"string"},
    "solver_retry_succeeded":{"type":"boolean"},
    "primary_yaw_solver_iterations":{"type":"integer"},
    "primary_pitch_solver_iterations":{"type":"integer"},
    "retry_yaw_solver_iterations":{"type":"integer"},
    "retry_pitch_solver_iterations":{"type":"integer"},
    "primary_yaw_solver_max_residual":{"type":"number"},
    "primary_pitch_solver_max_residual":{"type":"number"},
    "retry_yaw_solver_max_residual":{"type":"number"},
    "retry_pitch_solver_max_residual":{"type":"number"},
    "reference_horizon_delta_valid":{"type":"boolean"},
    "max_reference_horizon_delta_yaw":{"type":["number","null"]},
    "max_reference_horizon_delta_pitch":{"type":["number","null"]},
    "target_linear_speed_mps":{"type":"number"},
    "target_spin_rate_rad_s":{"type":"number"},
    "mpc_failure_reason":{"type":"string"},"mpc_failure_axis":{"type":"string"},
    "mpc_failure_index":{"type":"integer"},"yaw_solver_iterations":{"type":"integer"},
    "pitch_solver_iterations":{"type":"integer"},
    "yaw_solver_max_residual":{"type":"number"},
    "pitch_solver_max_residual":{"type":"number"},
    "actuator_valid":{"type":"boolean"},"actuator_mode":{"type":"string"},
    "actuator_command_valid":{"type":"boolean"},
    "actuator_saturation_flags":{"type":"integer"},
    "actuator_target_yaw":{"type":["number","null"]},
    "actuator_target_pitch":{"type":["number","null"]},
    "actuator_actual_yaw":{"type":["number","null"]},
    "actuator_actual_pitch":{"type":["number","null"]},
    "actuator_yaw_velocity":{"type":["number","null"]},
    "actuator_pitch_velocity":{"type":["number","null"]},
    "actuator_yaw_acceleration":{"type":["number","null"]},
    "actuator_pitch_acceleration":{"type":["number","null"]},
    "actuator_yaw_velocity_utilization":{"type":["number","null"]},
    "actuator_pitch_velocity_utilization":{"type":["number","null"]},
    "actuator_yaw_acceleration_utilization":{"type":["number","null"]},
    "actuator_pitch_acceleration_utilization":{"type":["number","null"]},
    "yaw_published_minus_actuator":{"type":["number","null"]},
    "pitch_published_minus_actuator":{"type":["number","null"]},
    "yaw_actuator_minus_measured":{"type":["number","null"]},
    "pitch_actuator_minus_measured":{"type":["number","null"]},
    "command_consumption_latency_s":{"type":["number","null"]},
    "consumed_command_timestamp_ns":{"type":["integer","null"]},
    "consumed_at_timestamp_ns":{"type":["integer","null"]},
    "matched_prior_approximate":{"type":"boolean"},
    "external_control_enabled":{"type":"boolean"},"command_sink_healthy":{"type":"boolean"}
  },
  "required":["timestamp","source_sequence","reference_valid","mpc_valid",
              "published_valid","estimated_valid","measured_valid","matched_prior_valid",
              "mpc_failure_reason","mpc_failure_axis","external_control_enabled"]
})json";
constexpr char K_TRAJECTORY_SCHEMA[] = R"json({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object"},"source_sequence":{"type":"integer"},"dt_s":{"type":"number"},
    "reference":{"type":"array","items":{"type":"object"}},
    "planned":{"type":"array","items":{"type":"object"}},
    "published":{"type":"object"},
    "estimated_history":{"type":"array","items":{"type":"object"}},
    "measured_history":{"type":"array","items":{"type":"object"}}
  },
  "required":["timestamp","source_sequence","dt_s","reference","planned",
              "estimated_history","measured_history"]
})json";

::foxglove::Schema JsonSchema(const char* name, const char* data, std::size_t size) {
  return {.name = name,
          .encoding = "jsonschema",
          .data = reinterpret_cast<const std::byte*>(data),
          .data_len = size};
}

/** @brief 创建带内嵌 JSON Schema 的原始频道，失败时保留 Foxglove 错误文本。 */
std::unique_ptr<::foxglove::RawChannel> CreateChannel(const char* topic, const char* schema_name,
                                                      const char* schema, std::size_t schema_size,
                                                      const ::foxglove::Context& context) {
  auto channel = ::foxglove::RawChannel::create(
      topic, "json", JsonSchema(schema_name, schema, schema_size), context);
  if (!channel.has_value()) {
    throw std::runtime_error(std::string("create ") + topic + ": " +
                             ::foxglove::strerror(channel.error()));
  }
  return std::make_unique<::foxglove::RawChannel>(std::move(channel).value());
}

// JSON 不支持 NaN 和无穷值；诊断中的非有限数统一编码为 null，保证消息始终合法。
std::string Number(double value) {
  return std::isfinite(value) ? fmt::format("{:.12g}", value) : "null";
}

std::string OptionalTimestamp(const std::optional<std::uint64_t>& value) {
  return value ? std::to_string(*value) : "null";
}

std::string Timestamp(std::uint64_t nanos) {
  return fmt::format("{{\"sec\":{},\"nsec\":{}}}", nanos / 1'000'000'000ULL,
                     nanos % 1'000'000'000ULL);
}

double UnwrapNear(double angle, double reference) noexcept {
  return reference + std::remainder(angle - reference, 2.0 * std::numbers::pi);
}

std::string OptionalNumber(bool valid, double value) {
  return valid ? Number(value) : "null";
}

double MaxResidual(const modules::MpcAxisDiagnostics& value) noexcept {
  return std::max({value.primal_residual_state, value.primal_residual_input,
                   value.dual_residual_state, value.dual_residual_input});
}

const char* ActuatorModeName(hal::GimbalActuatorMode mode) noexcept {
  switch (mode) {
    case hal::GimbalActuatorMode::PHYSICAL:
      return "physical";
    case hal::GimbalActuatorMode::IDEAL:
      return "ideal";
    case hal::GimbalActuatorMode::LEGACY:
      return "legacy";
  }
  return "legacy";
}

/** @brief 编码执行器目标、实际状态、约束标志和可选命令消费延迟。 */
std::string EncodeActuator(const hal::GimbalActuatorTelemetry& value) {
  const bool LATENCY_VALID = value.valid && value.consumed_command_timestamp_ns != 0 &&
                             value.consumed_at_timestamp_ns >= value.consumed_command_timestamp_ns;
  const double LATENCY_S = LATENCY_VALID
                               ? static_cast<double>(value.consumed_at_timestamp_ns -
                                                     value.consumed_command_timestamp_ns) *
                                     1.0e-9
                               : 0.0;
  return fmt::format(
      "{{\"valid\":{},\"mode\":\"{}\",\"command_valid\":{},"
      "\"saturation_flags\":{},\"state_timestamp_ns\":{},"
      "\"consumed_command_timestamp_ns\":{},\"consumed_at_timestamp_ns\":{},"
      "\"command_consumption_latency_s\":{},\"target_yaw\":{},\"target_pitch\":{},"
      "\"actual_yaw\":{},\"actual_pitch\":{},\"yaw_velocity\":{},"
      "\"pitch_velocity\":{},\"yaw_acceleration\":{},\"pitch_acceleration\":{}}}",
      value.valid, ActuatorModeName(value.mode), value.command_valid, value.saturation_flags,
      value.state_timestamp_ns,
      value.consumed_command_timestamp_ns != 0 ? std::to_string(value.consumed_command_timestamp_ns)
                                               : "null",
      value.consumed_at_timestamp_ns != 0 ? std::to_string(value.consumed_at_timestamp_ns) : "null",
      OptionalNumber(LATENCY_VALID, LATENCY_S), OptionalNumber(value.valid, value.target_yaw),
      OptionalNumber(value.valid, value.target_pitch),
      OptionalNumber(value.valid, value.actual_yaw),
      OptionalNumber(value.valid, value.actual_pitch),
      OptionalNumber(value.valid, value.yaw_velocity),
      OptionalNumber(value.valid, value.pitch_velocity),
      OptionalNumber(value.valid, value.yaw_acceleration),
      OptionalNumber(value.valid, value.pitch_acceleration));
}

/** @brief 将一次单轴 TinyMPC 求解诊断编码为可嵌套 JSON 对象。 */
std::string EncodeAxisDiagnostics(const modules::MpcAxisDiagnostics& axis) {
  return fmt::format(
      "{{\"status\":{},\"solved\":{},\"iterations\":{},"
      "\"primal_residual_state\":{},\"primal_residual_input\":{},"
      "\"dual_residual_state\":{},\"dual_residual_input\":{}}}",
      axis.status, axis.solved, axis.iterations, Number(axis.primal_residual_state),
      Number(axis.primal_residual_input), Number(axis.dual_residual_state),
      Number(axis.dual_residual_input));
}

/** @brief 编码四装甲候选、选择时域、滞回状态和切换决策。 */
std::string EncodeArmorSelection(const modules::ArmorSelectionDiagnostics& selection) {
  std::string candidates = "[";
  for (std::size_t index = 0; index < selection.candidates.size(); ++index) {
    if (index != 0)
      candidates.push_back(',');
    const auto& candidate = selection.candidates[index];
    const auto& pose = candidate.predicted_pose.world_t_armor;
    candidates += fmt::format(
        "{{\"slot\":{},\"view_angle_rad\":{},\"slew_angle_rad\":{},"
        "\"enter_eligible\":{},\"leave_eligible\":{},"
        "\"position_world\":[{},{},{}]}}",
        candidate.slot, Number(candidate.view_angle_rad), Number(candidate.slew_angle_rad),
        candidate.enter_eligible, candidate.leave_eligible, Number(pose.translation.x()),
        Number(pose.translation.y()), Number(pose.translation.z()));
  }
  candidates.push_back(']');
  return fmt::format(
      "{{\"horizon_s\":{},\"switch_confirmation_s\":{},\"locked_slot\":{},\"pending_slot\":{},"
      "\"pending_duration_s\":{},\"switched\":{},\"decision\":\"{}\","
      "\"candidates\":{}}}",
      Number(selection.horizon_s), Number(selection.switch_confirmation_s), selection.locked_slot,
      selection.pending_slot, Number(selection.pending_duration_s), selection.switched,
      modules::ArmorSelectionDecisionName(selection.decision), candidates);
}

/** @brief 编码分组后的完整控制状态；字段保持当前控制周期的原始物理单位。 */
std::string EncodeState(const modules::FireControlResult& value, std::uint64_t dropped_samples) {
  const auto& command = value.command;
  const auto& feedback = value.feedback;
  const auto& measured = value.measured_feedback;
  const auto& matched = value.matched_prior_command;
  const auto& ballistic = value.ballistic;
  const auto& plan = value.plan;
  return fmt::format(
      "{{\"timestamp\":{},\"source_sequence\":{},\"source_capture_timestamp_ns\":{},"
      "\"prediction_age_s\":{},\"feedback_age_s\":{},\"tracker_state\":\"{}\","
      "\"armor_slot\":{},"
      "\"armor_selection\":{},"
      "\"ballistics\":{{\"valid\":{},\"bullet_speed_mps\":{},\"distance_m\":{},"
      "\"flight_time_s\":{},\"target_world\":[{},{},{}]}},"
      "\"angles\":{{\"reference_yaw\":{},\"reference_pitch\":{},\"feedback_yaw\":{},"
      "\"feedback_pitch\":{},\"command_yaw\":{},\"command_pitch\":{},"
      "\"measured_yaw\":{},\"measured_pitch\":{},\"matched_prior_yaw\":{},"
      "\"matched_prior_pitch\":{},\"yaw_error\":{},\"pitch_error\":{}}},"
      "\"motion\":{{\"feedback_yaw_velocity\":{},\"feedback_pitch_velocity\":{},"
      "\"measured_yaw_velocity\":{},\"measured_pitch_velocity\":{},"
      "\"command_yaw_velocity\":{},\"command_pitch_velocity\":{},"
      "\"command_yaw_acceleration\":{},\"command_pitch_acceleration\":{},"
      "\"target_linear_speed_mps\":{},\"target_spin_rate_rad_s\":{}}},"
      "\"limits\":{{\"yaw_velocity\":{},\"pitch_velocity\":{},"
      "\"yaw_acceleration\":{},\"pitch_acceleration\":{}}},"
      "\"fire_window\":{{\"yaw\":{},\"pitch\":{}}},"
      "\"command\":{{\"source\":\"{}\",\"raw_mpc_valid\":{},\"published_valid\":{},"
      "\"fallback_active\":{},\"fallback_age_s\":{},\"fallback_source_slot\":{},"
      "\"fallback_trajectory_index\":{},\"fallback_remaining_points\":{},"
      "\"consecutive_mpc_failure_cycles\":{},\"feedback_source\":\"{}\","
      "\"raw_mpc_yaw\":{},\"raw_mpc_pitch\":{},\"published_yaw\":{},\"published_pitch\":{},"
      "\"solver_warm_start_reset\":{},\"solver_warm_start_reset_reason\":\"{}\","
      "\"warm_start_action\":\"{}\",\"warm_start_rebase_axes\":\"{}\","
      "\"solver_continued_after_failure\":{},\"solver_cycles_since_reset\":{},"
      "\"previous_yaw_solver_residual\":{},\"previous_pitch_solver_residual\":{},"
      "\"yaw_solver_residual_delta\":{},\"pitch_solver_residual_delta\":{},"
      "\"fallback_expired_this_cycle\":{},\"output_projection_cleared\":{},"
      "\"output_projection_clear_reason\":\"{}\","
      "\"reference_step_valid\":{},\"reference_yaw_step\":{},\"reference_pitch_step\":{}}},"
      "\"mpc\":{{\"valid\":{},\"command_index\":{},\"command_lookahead_s\":{},"
      "\"residuals_normalized\":{},"
      "\"normalization\":{{\"angle_scale_rad\":{},\"yaw_velocity_scale_rad_s\":{},"
      "\"pitch_velocity_scale_rad_s\":{},\"yaw_acceleration_scale_rad_s2\":{},"
      "\"pitch_acceleration_scale_rad_s2\":{}}},"
      "\"yaw_iterations\":{},\"pitch_iterations\":{},"
      "\"solve_time_us\":{},\"failure_reason\":\"{}\",\"failure_axis\":\"{}\","
      "\"failure_index\":{},\"yaw_solver\":{},\"pitch_solver\":{},"
      "\"solver_retry_attempted\":{},\"solver_retry_axes\":\"{}\","
      "\"solver_retry_succeeded\":{},\"primary_yaw_solver\":{},"
      "\"primary_pitch_solver\":{},\"retry_yaw_solver\":{},\"retry_pitch_solver\":{},"
      "\"reference_horizon_delta_valid\":{},"
      "\"max_reference_horizon_delta_yaw\":{},"
      "\"max_reference_horizon_delta_pitch\":{}}},"
      "\"fire\":{{\"auto_fire_enabled\":{},\"eligible\":{},\"pulse\":{},\"stable_cycles\":{},"
      "\"reject_reason\":\"{}\"}},"
      "\"talos\":{{\"healthy\":{},\"heartbeat_ns\":{},\"external_control_enabled\":{}}},"
      "\"actuator\":{},\"frame_actuator\":{},"
      "\"runtime\":{{\"measurement_fresh\":{},\"measurement_age_s\":{},"
      "\"matched_prior_valid\":{},\"matched_prior_approximate\":{},"
      "\"matched_prior_age_at_capture_s\":{},\"command_publish_succeeded\":{},"
      "\"control_period_s\":{},\"deadline_lateness_us\":{},\"sink_send_time_us\":{},"
      "\"runtime_actuator_age_s\":{},\"frame_actuator_age_s\":{},"
      "\"feedback_projection_dt_s\":{},"
      "\"feedback_runtime_state_timestamp_ns\":{},"
      "\"yaw_feedback_minus_runtime_actuator\":{},"
      "\"pitch_feedback_minus_runtime_actuator\":{},"
      "\"yaw_frame_minus_runtime_actuator\":{},"
      "\"pitch_frame_minus_runtime_actuator\":{},"
      "\"yaw_frame_acceleration_minus_runtime\":{},"
      "\"pitch_frame_acceleration_minus_runtime\":{}}},"
      "\"diagnostics\":{{\"dropped_control_samples\":{}}}}}",
      Timestamp(value.command_timestamp_ns), value.source_sequence,
      OptionalTimestamp(value.source_capture_timestamp_ns), Number(value.prediction_age_s),
      Number(value.feedback_age_s), TrackerStateName(value.tracker_state), value.selected_slot,
      EncodeArmorSelection(value.armor_selection), ballistic.valid, Number(value.bullet_speed_mps),
      Number(ballistic.distance_m), Number(ballistic.fly_time_s),
      Number(ballistic.target_world.x()), Number(ballistic.target_world.y()),
      Number(ballistic.target_world.z()), Number(value.target_yaw), Number(value.target_pitch),
      Number(feedback.yaw), Number(feedback.pitch), Number(command.yaw), Number(command.pitch),
      OptionalNumber(measured.valid, measured.yaw), OptionalNumber(measured.valid, measured.pitch),
      OptionalNumber(matched.valid, matched.command.yaw),
      OptionalNumber(matched.valid, matched.command.pitch), Number(value.yaw_error),
      Number(value.pitch_error), Number(feedback.yaw_velocity), Number(feedback.pitch_velocity),
      OptionalNumber(measured.valid, measured.yaw_velocity),
      OptionalNumber(measured.valid, measured.pitch_velocity), Number(command.yaw_velocity),
      Number(command.pitch_velocity), Number(command.yaw_acceleration),
      Number(command.pitch_acceleration), Number(value.target_linear_speed_mps),
      Number(value.target_spin_rate_rad_s), Number(value.max_yaw_velocity_rad_s),
      Number(value.max_pitch_velocity_rad_s), Number(value.max_yaw_acceleration_rad_s2),
      Number(value.max_pitch_acceleration_rad_s2), Number(value.fire_yaw_window),
      Number(value.fire_pitch_window), modules::GimbalCommandSourceName(value.command_source),
      value.raw_mpc_valid, value.published_valid, value.fallback_active,
      Number(value.fallback_age_s), value.fallback_source_slot, value.fallback_trajectory_index,
      value.fallback_remaining_points, value.consecutive_mpc_failure_cycles,
      modules::GimbalFeedbackSourceName(value.feedback_source),
      OptionalNumber(value.plan.trajectory.size() > 1, value.raw_mpc_command.yaw),
      OptionalNumber(value.plan.trajectory.size() > 1, value.raw_mpc_command.pitch),
      OptionalNumber(value.published_valid, command.yaw),
      OptionalNumber(value.published_valid, command.pitch), value.solver_warm_start_reset,
      value.solver_warm_start_reset_reason,
      modules::GimbalWarmStartActionName(plan.warm_start_action),
      modules::GimbalTrajectoryAxisName(plan.warm_start_rebase_axes),
      value.solver_continued_after_failure, value.solver_cycles_since_reset,
      OptionalNumber(value.previous_solver_residual_valid, value.previous_yaw_solver_residual),
      OptionalNumber(value.previous_solver_residual_valid, value.previous_pitch_solver_residual),
      OptionalNumber(value.previous_solver_residual_valid, value.yaw_solver_residual_delta),
      OptionalNumber(value.previous_solver_residual_valid, value.pitch_solver_residual_delta),
      value.fallback_expired_this_cycle, value.output_projection_cleared,
      value.output_projection_clear_reason, value.reference_step_valid,
      OptionalNumber(value.reference_step_valid, value.reference_yaw_step),
      OptionalNumber(value.reference_step_valid, value.reference_pitch_step), plan.valid,
      plan.command_index, Number(plan.command_lookahead_s), plan.residuals_normalized,
      Number(plan.normalization_angle_scale_rad),
      Number(plan.normalization_yaw_velocity_scale_rad_s),
      Number(plan.normalization_pitch_velocity_scale_rad_s),
      Number(plan.normalization_yaw_acceleration_scale_rad_s2),
      Number(plan.normalization_pitch_acceleration_scale_rad_s2), plan.yaw_iterations,
      plan.pitch_iterations, Number(plan.solve_time_us),
      modules::GimbalTrajectoryFailureReasonName(plan.failure_reason),
      modules::GimbalTrajectoryAxisName(plan.failure_axis), plan.failure_index,
      EncodeAxisDiagnostics(plan.yaw_solver), EncodeAxisDiagnostics(plan.pitch_solver),
      plan.solver_retry_attempted, modules::GimbalTrajectoryAxisName(plan.solver_retry_axes),
      plan.solver_retry_succeeded, EncodeAxisDiagnostics(plan.primary_yaw_solver),
      EncodeAxisDiagnostics(plan.primary_pitch_solver),
      EncodeAxisDiagnostics(plan.retry_yaw_solver), EncodeAxisDiagnostics(plan.retry_pitch_solver),
      plan.reference_horizon_delta_valid,
      OptionalNumber(plan.reference_horizon_delta_valid, plan.max_reference_horizon_delta_yaw),
      OptionalNumber(plan.reference_horizon_delta_valid, plan.max_reference_horizon_delta_pitch),
      value.auto_fire_enabled, value.fire_eligible, command.fire, value.stable_cycles,
      modules::FireRejectReasonName(value.reject_reason), value.command_sink_healthy,
      value.talos_heartbeat_ns, value.external_control_enabled,
      EncodeActuator(value.actuator_telemetry),
      value.frame_actuator_telemetry ? EncodeActuator(*value.frame_actuator_telemetry) : "null",
      value.measurement_fresh, OptionalNumber(measured.valid, value.measurement_age_s),
      matched.valid, matched.approximate, OptionalNumber(matched.valid, matched.age_at_capture_s),
      value.command_publish_succeeded, Number(value.control_period_s),
      Number(value.deadline_lateness_us), Number(value.sink_send_time_us),
      Number(value.runtime_actuator_age_s), Number(value.frame_actuator_age_s),
      Number(value.feedback_projection_dt_s),
      value.feedback_runtime_state_timestamp_ns != 0
          ? std::to_string(value.feedback_runtime_state_timestamp_ns)
          : "null",
      OptionalNumber(value.feedback_runtime_comparison_valid,
                     value.yaw_feedback_minus_runtime_actuator),
      OptionalNumber(value.feedback_runtime_comparison_valid,
                     value.pitch_feedback_minus_runtime_actuator),
      OptionalNumber(value.frame_runtime_comparison_valid, value.yaw_frame_minus_runtime_actuator),
      OptionalNumber(value.frame_runtime_comparison_valid,
                     value.pitch_frame_minus_runtime_actuator),
      OptionalNumber(value.frame_runtime_comparison_valid,
                     value.yaw_frame_acceleration_minus_runtime),
      OptionalNumber(value.frame_runtime_comparison_valid,
                     value.pitch_frame_acceleration_minus_runtime),
      dropped_samples);
}

/**
 * @brief 编码适合 Foxglove Plot 的扁平跟踪误差、约束利用率和时序诊断。
 *
 * 偏航差值计算前会围绕比较基准连续展开，避免 ±pi 边界产生约 2pi 的伪跳变。
 */
std::string EncodeTracking(const modules::FireControlResult& value) {
  const bool REFERENCE_VALID = value.plan.reference.size() > 1;
  const bool MPC_CANDIDATE_VALID = value.plan.trajectory.size() > 1;
  const bool PUBLISHED_VALID = value.command_publish_succeeded && value.command.valid;
  const bool ESTIMATED_VALID = value.feedback.valid;
  const bool MEASURED_VALID = value.measured_feedback.valid;
  const bool MATCHED_VALID = value.matched_prior_command.valid;
  const auto REFERENCE = REFERENCE_VALID ? value.plan.reference[1] : modules::AimReferencePoint{};
  const auto MPC = MPC_CANDIDATE_VALID ? value.plan.trajectory[1] : modules::PlannedGimbalPoint{};
  const double PUBLISHED_YAW = PUBLISHED_VALID && ESTIMATED_VALID
                                   ? UnwrapNear(value.command.yaw, value.feedback.yaw)
                                   : value.command.yaw;
  const double MATCHED_YAW =
      MATCHED_VALID && MEASURED_VALID
          ? UnwrapNear(value.matched_prior_command.command.yaw, value.measured_feedback.yaw)
          : value.matched_prior_command.command.yaw;
  const bool REFERENCE_MPC_VALID = REFERENCE_VALID && MPC_CANDIDATE_VALID;
  const bool MPC_ESTIMATED_VALID = MPC_CANDIDATE_VALID && ESTIMATED_VALID;
  const bool PUBLISHED_MEASURED_VALID = PUBLISHED_VALID && MEASURED_VALID;
  const bool ESTIMATED_MEASURED_VALID = ESTIMATED_VALID && MEASURED_VALID;
  const bool MATCHED_MEASURED_VALID = MATCHED_VALID && MEASURED_VALID;
  const double YAW_VELOCITY_UTILIZATION =
      MPC_CANDIDATE_VALID && value.max_yaw_velocity_rad_s > 0.0
          ? std::abs(MPC.yaw_velocity) / value.max_yaw_velocity_rad_s
          : 0.0;
  const double PITCH_VELOCITY_UTILIZATION =
      MPC_CANDIDATE_VALID && value.max_pitch_velocity_rad_s > 0.0
          ? std::abs(MPC.pitch_velocity) / value.max_pitch_velocity_rad_s
          : 0.0;
  const double YAW_ACCELERATION_UTILIZATION =
      MPC_CANDIDATE_VALID && value.max_yaw_acceleration_rad_s2 > 0.0
          ? std::abs(MPC.yaw_acceleration) / value.max_yaw_acceleration_rad_s2
          : 0.0;
  const double PITCH_ACCELERATION_UTILIZATION =
      MPC_CANDIDATE_VALID && value.max_pitch_acceleration_rad_s2 > 0.0
          ? std::abs(MPC.pitch_acceleration) / value.max_pitch_acceleration_rad_s2
          : 0.0;
  const auto& actuator = value.actuator_telemetry;
  const bool ACTUATOR_VALID = actuator.valid;
  const bool PUBLISHED_ACTUATOR_VALID = PUBLISHED_VALID && ACTUATOR_VALID;
  const bool ACTUATOR_MEASURED_VALID = ACTUATOR_VALID && MEASURED_VALID;
  const bool CONSUMPTION_LATENCY_VALID =
      ACTUATOR_VALID && actuator.consumed_command_timestamp_ns != 0 &&
      actuator.consumed_at_timestamp_ns >= actuator.consumed_command_timestamp_ns;
  const double CONSUMPTION_LATENCY_S =
      CONSUMPTION_LATENCY_VALID ? static_cast<double>(actuator.consumed_at_timestamp_ns -
                                                      actuator.consumed_command_timestamp_ns) *
                                      1.0e-9
                                : 0.0;
  const double PUBLISHED_YAW_AT_ACTUATOR =
      PUBLISHED_ACTUATOR_VALID ? UnwrapNear(value.command.yaw, actuator.actual_yaw) : 0.0;
  const double ACTUATOR_YAW_AT_MEASUREMENT =
      ACTUATOR_MEASURED_VALID ? UnwrapNear(actuator.actual_yaw, value.measured_feedback.yaw) : 0.0;

  return fmt::format(
      "{{\"timestamp\":{},\"source_sequence\":{},"
      "\"reference_valid\":{},\"mpc_valid\":{},\"mpc_candidate_valid\":{},"
      "\"published_valid\":{},\"estimated_valid\":{},\"measured_valid\":{},"
      "\"matched_prior_valid\":{},"
      "\"command_source\":\"{}\",\"raw_mpc_valid\":{},\"fallback_active\":{},"
      "\"fallback_age_s\":{},\"fallback_source_slot\":{},"
      "\"fallback_trajectory_index\":{},\"fallback_remaining_points\":{},"
      "\"mpc_command_index\":{},\"command_lookahead_s\":{},"
      "\"mpc_residuals_normalized\":{},"
      "\"consecutive_mpc_failure_cycles\":{},\"feedback_source\":\"{}\","
      "\"solver_warm_start_reset\":{},\"solver_warm_start_reset_reason\":\"{}\","
      "\"solver_continued_after_failure\":{},\"solver_cycles_since_reset\":{},"
      "\"previous_yaw_solver_residual\":{},\"previous_pitch_solver_residual\":{},"
      "\"yaw_solver_residual_delta\":{},\"pitch_solver_residual_delta\":{},"
      "\"fallback_expired_this_cycle\":{},\"output_projection_cleared\":{},"
      "\"output_projection_clear_reason\":\"{}\","
      "\"reference_step_valid\":{},\"reference_yaw_step\":{},\"reference_pitch_step\":{},"
      "\"warm_start_action\":\"{}\",\"warm_start_rebase_axes\":\"{}\","
      "\"solver_retry_attempted\":{},\"solver_retry_axes\":\"{}\","
      "\"solver_retry_succeeded\":{},"
      "\"primary_yaw_solver_iterations\":{},\"primary_pitch_solver_iterations\":{},"
      "\"retry_yaw_solver_iterations\":{},\"retry_pitch_solver_iterations\":{},"
      "\"primary_yaw_solver_max_residual\":{},"
      "\"primary_pitch_solver_max_residual\":{},"
      "\"retry_yaw_solver_max_residual\":{},\"retry_pitch_solver_max_residual\":{},"
      "\"reference_horizon_delta_valid\":{},"
      "\"max_reference_horizon_delta_yaw\":{},"
      "\"max_reference_horizon_delta_pitch\":{},"
      "\"target_linear_speed_mps\":{},\"target_spin_rate_rad_s\":{},"
      "\"yaw_reference_next\":{},\"yaw_mpc_next\":{},\"yaw_published\":{},"
      "\"yaw_estimated\":{},\"yaw_measured\":{},\"yaw_matched_prior\":{},"
      "\"pitch_reference_next\":{},\"pitch_mpc_next\":{},\"pitch_published\":{},"
      "\"pitch_estimated\":{},\"pitch_measured\":{},\"pitch_matched_prior\":{},"
      "\"yaw_reference_velocity\":{},\"yaw_mpc_velocity\":{},"
      "\"yaw_estimated_velocity\":{},\"yaw_measured_velocity\":{},"
      "\"pitch_reference_velocity\":{},\"pitch_mpc_velocity\":{},"
      "\"pitch_estimated_velocity\":{},\"pitch_measured_velocity\":{},"
      "\"yaw_mpc_acceleration\":{},\"pitch_mpc_acceleration\":{},"
      "\"yaw_reference_minus_mpc\":{},\"yaw_mpc_minus_estimated\":{},"
      "\"yaw_published_minus_measured\":{},\"yaw_estimated_minus_measured\":{},"
      "\"yaw_matched_prior_minus_measured\":{},"
      "\"pitch_reference_minus_mpc\":{},\"pitch_mpc_minus_estimated\":{},"
      "\"pitch_published_minus_measured\":{},\"pitch_estimated_minus_measured\":{},"
      "\"pitch_matched_prior_minus_measured\":{},"
      "\"yaw_velocity_utilization\":{},\"pitch_velocity_utilization\":{},"
      "\"yaw_acceleration_utilization\":{},\"pitch_acceleration_utilization\":{},"
      "\"control_period_s\":{},\"deadline_lateness_us\":{},"
      "\"prediction_age_s\":{},\"measurement_age_s\":{},"
      "\"runtime_actuator_age_s\":{},\"frame_actuator_age_s\":{},"
      "\"feedback_projection_dt_s\":{},"
      "\"feedback_runtime_state_timestamp_ns\":{},"
      "\"yaw_feedback_minus_runtime_actuator\":{},"
      "\"pitch_feedback_minus_runtime_actuator\":{},"
      "\"yaw_frame_minus_runtime_actuator\":{},"
      "\"pitch_frame_minus_runtime_actuator\":{},"
      "\"yaw_frame_acceleration_minus_runtime\":{},"
      "\"pitch_frame_acceleration_minus_runtime\":{},"
      "\"matched_prior_age_at_capture_s\":{},\"sink_send_time_us\":{},"
      "\"actuator_valid\":{},\"actuator_mode\":\"{}\","
      "\"actuator_command_valid\":{},\"actuator_saturation_flags\":{},"
      "\"actuator_target_yaw\":{},\"actuator_target_pitch\":{},"
      "\"actuator_actual_yaw\":{},\"actuator_actual_pitch\":{},"
      "\"actuator_yaw_velocity\":{},\"actuator_pitch_velocity\":{},"
      "\"actuator_yaw_acceleration\":{},\"actuator_pitch_acceleration\":{},"
      "\"actuator_yaw_velocity_utilization\":{},"
      "\"actuator_pitch_velocity_utilization\":{},"
      "\"actuator_yaw_acceleration_utilization\":{},"
      "\"actuator_pitch_acceleration_utilization\":{},"
      "\"yaw_published_minus_actuator\":{},\"pitch_published_minus_actuator\":{},"
      "\"yaw_actuator_minus_measured\":{},\"pitch_actuator_minus_measured\":{},"
      "\"command_consumption_latency_s\":{},"
      "\"consumed_command_timestamp_ns\":{},\"consumed_at_timestamp_ns\":{},"
      "\"matched_prior_approximate\":{},"
      "\"mpc_failure_reason\":\"{}\",\"mpc_failure_axis\":\"{}\","
      "\"mpc_failure_index\":{},\"yaw_solver_iterations\":{},"
      "\"pitch_solver_iterations\":{},\"yaw_solver_max_residual\":{},"
      "\"pitch_solver_max_residual\":{},\"external_control_enabled\":{},"
      "\"command_sink_healthy\":{}}}",
      Timestamp(value.command_timestamp_ns), value.source_sequence, REFERENCE_VALID,
      value.plan.valid, MPC_CANDIDATE_VALID, PUBLISHED_VALID, ESTIMATED_VALID, MEASURED_VALID,
      MATCHED_VALID, modules::GimbalCommandSourceName(value.command_source), value.raw_mpc_valid,
      value.fallback_active, Number(value.fallback_age_s), value.fallback_source_slot,
      value.fallback_trajectory_index, value.fallback_remaining_points, value.plan.command_index,
      Number(value.plan.command_lookahead_s), value.plan.residuals_normalized,
      value.consecutive_mpc_failure_cycles,
      modules::GimbalFeedbackSourceName(value.feedback_source), value.solver_warm_start_reset,
      value.solver_warm_start_reset_reason, value.solver_continued_after_failure,
      value.solver_cycles_since_reset,
      OptionalNumber(value.previous_solver_residual_valid, value.previous_yaw_solver_residual),
      OptionalNumber(value.previous_solver_residual_valid, value.previous_pitch_solver_residual),
      OptionalNumber(value.previous_solver_residual_valid, value.yaw_solver_residual_delta),
      OptionalNumber(value.previous_solver_residual_valid, value.pitch_solver_residual_delta),
      value.fallback_expired_this_cycle, value.output_projection_cleared,
      value.output_projection_clear_reason, value.reference_step_valid,
      OptionalNumber(value.reference_step_valid, value.reference_yaw_step),
      OptionalNumber(value.reference_step_valid, value.reference_pitch_step),
      modules::GimbalWarmStartActionName(value.plan.warm_start_action),
      modules::GimbalTrajectoryAxisName(value.plan.warm_start_rebase_axes),
      value.plan.solver_retry_attempted,
      modules::GimbalTrajectoryAxisName(value.plan.solver_retry_axes),
      value.plan.solver_retry_succeeded, value.plan.primary_yaw_solver.iterations,
      value.plan.primary_pitch_solver.iterations, value.plan.retry_yaw_solver.iterations,
      value.plan.retry_pitch_solver.iterations, Number(MaxResidual(value.plan.primary_yaw_solver)),
      Number(MaxResidual(value.plan.primary_pitch_solver)),
      Number(MaxResidual(value.plan.retry_yaw_solver)),
      Number(MaxResidual(value.plan.retry_pitch_solver)), value.plan.reference_horizon_delta_valid,
      OptionalNumber(value.plan.reference_horizon_delta_valid,
                     value.plan.max_reference_horizon_delta_yaw),
      OptionalNumber(value.plan.reference_horizon_delta_valid,
                     value.plan.max_reference_horizon_delta_pitch),
      Number(value.target_linear_speed_mps), Number(value.target_spin_rate_rad_s),
      OptionalNumber(REFERENCE_VALID, REFERENCE.yaw), OptionalNumber(MPC_CANDIDATE_VALID, MPC.yaw),
      OptionalNumber(PUBLISHED_VALID, PUBLISHED_YAW),
      OptionalNumber(ESTIMATED_VALID, value.feedback.yaw),
      OptionalNumber(MEASURED_VALID, value.measured_feedback.yaw),
      OptionalNumber(MATCHED_VALID, MATCHED_YAW), OptionalNumber(REFERENCE_VALID, REFERENCE.pitch),
      OptionalNumber(MPC_CANDIDATE_VALID, MPC.pitch),
      OptionalNumber(PUBLISHED_VALID, value.command.pitch),
      OptionalNumber(ESTIMATED_VALID, value.feedback.pitch),
      OptionalNumber(MEASURED_VALID, value.measured_feedback.pitch),
      OptionalNumber(MATCHED_VALID, value.matched_prior_command.command.pitch),
      OptionalNumber(REFERENCE_VALID, REFERENCE.yaw_velocity),
      OptionalNumber(MPC_CANDIDATE_VALID, MPC.yaw_velocity),
      OptionalNumber(ESTIMATED_VALID, value.feedback.yaw_velocity),
      OptionalNumber(MEASURED_VALID, value.measured_feedback.yaw_velocity),
      OptionalNumber(REFERENCE_VALID, REFERENCE.pitch_velocity),
      OptionalNumber(MPC_CANDIDATE_VALID, MPC.pitch_velocity),
      OptionalNumber(ESTIMATED_VALID, value.feedback.pitch_velocity),
      OptionalNumber(MEASURED_VALID, value.measured_feedback.pitch_velocity),
      OptionalNumber(MPC_CANDIDATE_VALID, MPC.yaw_acceleration),
      OptionalNumber(MPC_CANDIDATE_VALID, MPC.pitch_acceleration),
      OptionalNumber(REFERENCE_MPC_VALID, REFERENCE.yaw - MPC.yaw),
      OptionalNumber(MPC_ESTIMATED_VALID, MPC.yaw - value.feedback.yaw),
      OptionalNumber(PUBLISHED_MEASURED_VALID, PUBLISHED_YAW - value.measured_feedback.yaw),
      OptionalNumber(ESTIMATED_MEASURED_VALID, value.feedback.yaw - value.measured_feedback.yaw),
      OptionalNumber(MATCHED_MEASURED_VALID, MATCHED_YAW - value.measured_feedback.yaw),
      OptionalNumber(REFERENCE_MPC_VALID, REFERENCE.pitch - MPC.pitch),
      OptionalNumber(MPC_ESTIMATED_VALID, MPC.pitch - value.feedback.pitch),
      OptionalNumber(PUBLISHED_MEASURED_VALID, value.command.pitch - value.measured_feedback.pitch),
      OptionalNumber(ESTIMATED_MEASURED_VALID,
                     value.feedback.pitch - value.measured_feedback.pitch),
      OptionalNumber(MATCHED_MEASURED_VALID,
                     value.matched_prior_command.command.pitch - value.measured_feedback.pitch),
      OptionalNumber(MPC_CANDIDATE_VALID, YAW_VELOCITY_UTILIZATION),
      OptionalNumber(MPC_CANDIDATE_VALID, PITCH_VELOCITY_UTILIZATION),
      OptionalNumber(MPC_CANDIDATE_VALID, YAW_ACCELERATION_UTILIZATION),
      OptionalNumber(MPC_CANDIDATE_VALID, PITCH_ACCELERATION_UTILIZATION),
      Number(value.control_period_s), Number(value.deadline_lateness_us),
      Number(value.prediction_age_s), OptionalNumber(MEASURED_VALID, value.measurement_age_s),
      Number(value.runtime_actuator_age_s), Number(value.frame_actuator_age_s),
      Number(value.feedback_projection_dt_s),
      value.feedback_runtime_state_timestamp_ns != 0
          ? std::to_string(value.feedback_runtime_state_timestamp_ns)
          : "null",
      OptionalNumber(value.feedback_runtime_comparison_valid,
                     value.yaw_feedback_minus_runtime_actuator),
      OptionalNumber(value.feedback_runtime_comparison_valid,
                     value.pitch_feedback_minus_runtime_actuator),
      OptionalNumber(value.frame_runtime_comparison_valid, value.yaw_frame_minus_runtime_actuator),
      OptionalNumber(value.frame_runtime_comparison_valid,
                     value.pitch_frame_minus_runtime_actuator),
      OptionalNumber(value.frame_runtime_comparison_valid,
                     value.yaw_frame_acceleration_minus_runtime),
      OptionalNumber(value.frame_runtime_comparison_valid,
                     value.pitch_frame_acceleration_minus_runtime),
      OptionalNumber(MATCHED_VALID, value.matched_prior_command.age_at_capture_s),
      Number(value.sink_send_time_us), ACTUATOR_VALID, ActuatorModeName(actuator.mode),
      actuator.command_valid, actuator.saturation_flags,
      OptionalNumber(ACTUATOR_VALID, actuator.target_yaw),
      OptionalNumber(ACTUATOR_VALID, actuator.target_pitch),
      OptionalNumber(ACTUATOR_VALID, actuator.actual_yaw),
      OptionalNumber(ACTUATOR_VALID, actuator.actual_pitch),
      OptionalNumber(ACTUATOR_VALID, actuator.yaw_velocity),
      OptionalNumber(ACTUATOR_VALID, actuator.pitch_velocity),
      OptionalNumber(ACTUATOR_VALID, actuator.yaw_acceleration),
      OptionalNumber(ACTUATOR_VALID, actuator.pitch_acceleration),
      OptionalNumber(ACTUATOR_VALID && value.max_yaw_velocity_rad_s > 0.0,
                     std::abs(actuator.yaw_velocity) / value.max_yaw_velocity_rad_s),
      OptionalNumber(ACTUATOR_VALID && value.max_pitch_velocity_rad_s > 0.0,
                     std::abs(actuator.pitch_velocity) / value.max_pitch_velocity_rad_s),
      OptionalNumber(ACTUATOR_VALID && value.max_yaw_acceleration_rad_s2 > 0.0,
                     std::abs(actuator.yaw_acceleration) / value.max_yaw_acceleration_rad_s2),
      OptionalNumber(ACTUATOR_VALID && value.max_pitch_acceleration_rad_s2 > 0.0,
                     std::abs(actuator.pitch_acceleration) / value.max_pitch_acceleration_rad_s2),
      OptionalNumber(PUBLISHED_ACTUATOR_VALID, PUBLISHED_YAW_AT_ACTUATOR - actuator.actual_yaw),
      OptionalNumber(PUBLISHED_ACTUATOR_VALID, value.command.pitch - actuator.actual_pitch),
      OptionalNumber(ACTUATOR_MEASURED_VALID,
                     ACTUATOR_YAW_AT_MEASUREMENT - value.measured_feedback.yaw),
      OptionalNumber(ACTUATOR_MEASURED_VALID,
                     actuator.actual_pitch - value.measured_feedback.pitch),
      OptionalNumber(CONSUMPTION_LATENCY_VALID, CONSUMPTION_LATENCY_S),
      actuator.consumed_command_timestamp_ns != 0
          ? std::to_string(actuator.consumed_command_timestamp_ns)
          : "null",
      actuator.consumed_at_timestamp_ns != 0 ? std::to_string(actuator.consumed_at_timestamp_ns)
                                             : "null",
      value.matched_prior_command.approximate,
      modules::GimbalTrajectoryFailureReasonName(value.plan.failure_reason),
      modules::GimbalTrajectoryAxisName(value.plan.failure_axis), value.plan.failure_index,
      value.plan.yaw_iterations, value.plan.pitch_iterations,
      Number(MaxResidual(value.plan.yaw_solver)), Number(MaxResidual(value.plan.pitch_solver)),
      value.external_control_enabled, value.command_sink_healthy);
}

/** @brief 编码完整 MPC 时域以及相对当前命令时刻的一秒反馈历史。 */
std::string EncodeTrajectory(const modules::FireControlResult& value,
                             const std::deque<FeedbackHistorySample>& estimated_history,
                             const std::deque<FeedbackHistorySample>& measured_history) {
  std::string output = fmt::format(
      "{{\"timestamp\":{},\"source_sequence\":{},\"dt_s\":{},\"command_index\":{},"
      "\"command_lookahead_s\":{},\"fallback_trajectory_index\":{},\"reference\":[",
      Timestamp(value.command_timestamp_ns), value.source_sequence, Number(value.trajectory_dt_s),
      value.plan.command_index, Number(value.plan.command_lookahead_s),
      value.fallback_trajectory_index);
  for (std::size_t index = 0; index < value.plan.reference.size(); ++index) {
    if (index != 0)
      output.push_back(',');
    const auto& point = value.plan.reference[index];
    output += fmt::format(
        "{{\"t_s\":{},\"yaw\":{},\"yaw_velocity\":{},\"pitch\":{},"
        "\"pitch_velocity\":{}}}",
        Number(static_cast<double>(index) * value.trajectory_dt_s), Number(point.yaw),
        Number(point.yaw_velocity), Number(point.pitch), Number(point.pitch_velocity));
  }
  output += "],\"planned\":[";
  for (std::size_t index = 0; index < value.plan.trajectory.size(); ++index) {
    if (index != 0)
      output.push_back(',');
    const auto& point = value.plan.trajectory[index];
    output += fmt::format(
        "{{\"t_s\":{},\"yaw\":{},\"yaw_velocity\":{},\"yaw_acceleration\":{},\"pitch\":{},"
        "\"pitch_velocity\":{},\"pitch_acceleration\":{}}}",
        Number(static_cast<double>(index) * value.trajectory_dt_s), Number(point.yaw),
        Number(point.yaw_velocity), Number(point.yaw_acceleration), Number(point.pitch),
        Number(point.pitch_velocity), Number(point.pitch_acceleration));
  }
  output += fmt::format(
      "],\"published\":{{\"valid\":{},\"source\":\"{}\",\"fallback_active\":{},"
      "\"yaw\":{},\"pitch\":{},\"yaw_velocity\":{},\"pitch_velocity\":{}}},"
      "\"estimated_history\":[",
      value.published_valid, modules::GimbalCommandSourceName(value.command_source),
      value.fallback_active, OptionalNumber(value.published_valid, value.command.yaw),
      OptionalNumber(value.published_valid, value.command.pitch),
      OptionalNumber(value.published_valid, value.command.yaw_velocity),
      OptionalNumber(value.published_valid, value.command.pitch_velocity));
  for (std::size_t index = 0; index < estimated_history.size(); ++index) {
    if (index != 0)
      output.push_back(',');
    const auto& sample = estimated_history[index];
    const double RELATIVE_TIME = (static_cast<double>(sample.timestamp_ns) -
                                  static_cast<double>(value.command_timestamp_ns)) *
                                 1.0e-9;
    output += fmt::format(
        "{{\"t_s\":{},\"yaw\":{},\"yaw_velocity\":{},\"pitch\":{},"
        "\"pitch_velocity\":{}}}",
        Number(RELATIVE_TIME), Number(sample.feedback.yaw), Number(sample.feedback.yaw_velocity),
        Number(sample.feedback.pitch), Number(sample.feedback.pitch_velocity));
  }
  output += "],\"measured_history\":[";
  for (std::size_t index = 0; index < measured_history.size(); ++index) {
    if (index != 0)
      output.push_back(',');
    const auto& sample = measured_history[index];
    const double RELATIVE_TIME = (static_cast<double>(sample.timestamp_ns) -
                                  static_cast<double>(value.command_timestamp_ns)) *
                                 1.0e-9;
    output += fmt::format(
        "{{\"t_s\":{},\"yaw\":{},\"yaw_velocity\":{},\"pitch\":{},"
        "\"pitch_velocity\":{}}}",
        Number(RELATIVE_TIME), Number(sample.feedback.yaw), Number(sample.feedback.yaw_velocity),
        Number(sample.feedback.pitch), Number(sample.feedback.pitch_velocity));
  }
  output += "]}";
  return output;
}

::foxglove::schemas::Point3 Point(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

geometry::Vector3 AimPoint(const geometry::Vector3& muzzle, double yaw, double pitch,
                           double distance) {
  const double HORIZONTAL = std::cos(pitch);
  return muzzle + distance * geometry::Vector3(HORIZONTAL * std::cos(yaw),
                                               HORIZONTAL * std::sin(yaw), std::sin(pitch));
}

/**
 * @brief 构造 world 坐标系控制场景。
 *
 * 场景同时显示四装甲候选、枪口和弹道命中点，以及参考、MPC、融合反馈、实测反馈和
 * 已发布命令射线；颜色含义也写入场景状态文本。
 */
::foxglove::schemas::SceneUpdate EncodeScene(
    const modules::FireControlResult& value,
    const std::deque<FeedbackHistorySample>& estimated_history,
    const std::deque<FeedbackHistorySample>& measured_history) {
  ::foxglove::schemas::SceneUpdate update;
  if (!value.muzzle_pose_valid)
    return update;
  const auto TIMESTAMP = ::foxglove::schemas::Timestamp{
      .sec = static_cast<std::uint32_t>(value.command_timestamp_ns / 1'000'000'000ULL),
      .nsec = static_cast<std::uint32_t>(value.command_timestamp_ns % 1'000'000'000ULL)};
  const ::foxglove::schemas::Duration lifetime{.sec = 0, .nsec = 150'000'000};
  const auto muzzle = value.world_t_muzzle.translation;
  const double distance = value.ballistic.valid ? value.ballistic.distance_m : 3.0;

  ::foxglove::schemas::SceneEntity entity;
  entity.timestamp = TIMESTAMP;
  entity.frame_id = "world";
  entity.id = "control_trajectory";
  entity.lifetime = lifetime;
  entity.metadata = {
      {.key = "tracker", .value = modules::TrackerStateName(value.tracker_state)},
      {.key = "slot", .value = std::to_string(value.selected_slot)},
      {.key = "reject", .value = std::string(modules::FireRejectReasonName(value.reject_reason))},
      {.key = "mpc", .value = value.plan.valid ? "valid" : "invalid"},
      {.key = "command_source",
       .value = std::string(modules::GimbalCommandSourceName(value.command_source))},
      {.key = "command_lookahead_s", .value = Number(value.plan.command_lookahead_s)},
      {.key = "fallback_trajectory_index",
       .value = std::to_string(value.fallback_trajectory_index)},
      {.key = "fire", .value = value.command.fire ? "pulse" : "off"},
      {.key = "actuator_mode", .value = ActuatorModeName(value.actuator_telemetry.mode)},
      {.key = "actuator_saturation",
       .value = std::to_string(value.actuator_telemetry.saturation_flags)},
      {.key = "feedback_source",
       .value = std::string(modules::GimbalFeedbackSourceName(value.feedback_source))},
      {.key = "runtime_actuator_age_s", .value = Number(value.runtime_actuator_age_s)},
      {.key = "pitch_feedback_minus_runtime",
       .value = value.feedback_runtime_comparison_valid
                    ? Number(value.pitch_feedback_minus_runtime_actuator)
                    : "null"},
      {.key = "selection_decision",
       .value = std::string(modules::ArmorSelectionDecisionName(value.armor_selection.decision))}};

  const double ARMOR_WIDTH =
      value.tracked_type == hal::CameraFrame::ArmorType::LARGE ? 0.225 : 0.135;
  constexpr double ARMOR_HEIGHT = 0.055;
  for (const auto& candidate : value.armor_selection.candidates) {
    const auto& pose = candidate.predicted_pose.world_t_armor;
    if (!pose.translation.allFinite() || !pose.rotation.coeffs().allFinite())
      continue;
    const auto X_AXIS = geometry::TransformVector(pose, geometry::Vector3::UnitX());
    const auto Y_AXIS = geometry::TransformVector(pose, geometry::Vector3::UnitY());
    const bool SELECTED = candidate.slot == value.armor_selection.locked_slot;
    const bool PENDING = candidate.slot == value.armor_selection.pending_slot;
    ::foxglove::schemas::LinePrimitive outline;
    outline.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
    outline.thickness = SELECTED ? 0.025 : PENDING ? 0.020 : 0.010;
    outline.color = SELECTED  ? ::foxglove::schemas::Color{.r = 0.1, .g = 1.0, .b = 0.2, .a = 1.0}
                    : PENDING ? ::foxglove::schemas::Color{.r = 1.0, .g = 0.8, .b = 0.0, .a = 1.0}
                    : candidate.enter_eligible
                        ? ::foxglove::schemas::Color{.r = 0.0, .g = 0.8, .b = 1.0, .a = 0.85}
                        : ::foxglove::schemas::Color{.r = 0.55, .g = 0.55, .b = 0.55, .a = 0.35};
    outline.points = {
        Point(pose.translation - X_AXIS * ARMOR_WIDTH * 0.5 + Y_AXIS * ARMOR_HEIGHT * 0.5),
        Point(pose.translation + X_AXIS * ARMOR_WIDTH * 0.5 + Y_AXIS * ARMOR_HEIGHT * 0.5),
        Point(pose.translation + X_AXIS * ARMOR_WIDTH * 0.5 - Y_AXIS * ARMOR_HEIGHT * 0.5),
        Point(pose.translation - X_AXIS * ARMOR_WIDTH * 0.5 - Y_AXIS * ARMOR_HEIGHT * 0.5)};
    entity.lines.push_back(std::move(outline));

    ::foxglove::schemas::TextPrimitive label;
    label.pose = ::foxglove::schemas::Pose{
        .position = ::foxglove::schemas::Vector3{.x = pose.translation.x(),
                                                 .y = pose.translation.y(),
                                                 .z = pose.translation.z() + 0.09},
        .orientation = ::foxglove::schemas::Quaternion{.w = 1.0}};
    label.billboard = true;
    label.font_size = 11.0;
    label.scale_invariant = true;
    label.color = outline.color;
    label.text = fmt::format("slot {} | view {:.1f} deg{}", candidate.slot,
                             candidate.view_angle_rad * 180.0 / std::numbers::pi,
                             SELECTED  ? " | SELECTED"
                             : PENDING ? " | PENDING"
                                       : "");
    entity.texts.push_back(std::move(label));
  }

  ::foxglove::schemas::SpherePrimitive muzzle_marker;
  muzzle_marker.pose = ::foxglove::schemas::Pose{
      .position = ::foxglove::schemas::Vector3{.x = muzzle.x(), .y = muzzle.y(), .z = muzzle.z()},
      .orientation = ::foxglove::schemas::Quaternion{.w = 1.0}};
  muzzle_marker.size = {.x = 0.06, .y = 0.06, .z = 0.06};
  muzzle_marker.color = {.r = 1.0, .g = 1.0, .b = 1.0, .a = 1.0};
  entity.spheres.push_back(std::move(muzzle_marker));

  if (value.ballistic.valid) {
    ::foxglove::schemas::SpherePrimitive target;
    target.pose = ::foxglove::schemas::Pose{
        .position = ::foxglove::schemas::Vector3{.x = value.ballistic.target_world.x(),
                                                 .y = value.ballistic.target_world.y(),
                                                 .z = value.ballistic.target_world.z()},
        .orientation = ::foxglove::schemas::Quaternion{.w = 1.0}};
    target.size = {.x = 0.10, .y = 0.10, .z = 0.10};
    target.color =
        value.command.fire    ? ::foxglove::schemas::Color{.r = 1.0, .g = 0.1, .b = 0.1, .a = 1.0}
        : value.fire_eligible ? ::foxglove::schemas::Color{.r = 1.0, .g = 0.8, .b = 0.0, .a = 1.0}
                              : ::foxglove::schemas::Color{.r = 0.8, .g = 0.2, .b = 0.8, .a = 1.0};
    entity.spheres.push_back(std::move(target));
  }

  auto add_ray = [&](double yaw, double pitch, ::foxglove::schemas::Color color, double thickness) {
    if (!std::isfinite(yaw) || !std::isfinite(pitch))
      return;
    ::foxglove::schemas::LinePrimitive ray;
    ray.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    ray.thickness = thickness;
    ray.color = color;
    ray.points = {Point(muzzle), Point(AimPoint(muzzle, yaw, pitch, distance))};
    entity.lines.push_back(std::move(ray));
  };
  add_ray(value.feedback.yaw, value.feedback.pitch, {.r = 1.0, .g = 1.0, .b = 1.0, .a = 0.8},
          0.008);
  if (value.command_publish_succeeded && value.command.valid) {
    add_ray(value.command.yaw, value.command.pitch, {.r = 0.1, .g = 1.0, .b = 0.2, .a = 1.0},
            0.015);
  }

  auto add_history = [&](const std::deque<FeedbackHistorySample>& history,
                         ::foxglove::schemas::Color color, double thickness) {
    ::foxglove::schemas::LinePrimitive line;
    line.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    line.thickness = thickness;
    line.color = color;
    for (const auto& sample : history) {
      if (!sample.feedback.valid || !std::isfinite(sample.feedback.yaw) ||
          !std::isfinite(sample.feedback.pitch))
        continue;
      line.points.push_back(
          Point(AimPoint(muzzle, sample.feedback.yaw, sample.feedback.pitch, distance)));
    }
    if (line.points.size() >= 2)
      entity.lines.push_back(std::move(line));
  };
  add_history(estimated_history, {.r = 1.0, .g = 1.0, .b = 1.0, .a = 0.85}, 0.010);
  add_history(measured_history, {.r = 1.0, .g = 0.1, .b = 0.8, .a = 1.0}, 0.016);

  if (!value.plan.reference.empty()) {
    ::foxglove::schemas::LinePrimitive reference;
    reference.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    reference.thickness = 0.008;
    reference.color = {.r = 0.0, .g = 0.8, .b = 1.0, .a = 0.9};
    for (const auto& point : value.plan.reference) {
      if (!std::isfinite(point.yaw) || !std::isfinite(point.pitch))
        continue;
      reference.points.push_back(Point(AimPoint(muzzle, point.yaw, point.pitch, distance)));
    }
    if (reference.points.size() >= 2)
      entity.lines.push_back(std::move(reference));
  }
  if (!value.plan.trajectory.empty()) {
    ::foxglove::schemas::LinePrimitive planned;
    planned.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    planned.thickness = 0.012;
    planned.color = value.plan.valid
                        ? ::foxglove::schemas::Color{.r = 1.0, .g = 0.75, .b = 0.0, .a = 1.0}
                        : ::foxglove::schemas::Color{.r = 1.0, .g = 0.1, .b = 0.1, .a = 1.0};
    for (const auto& point : value.plan.trajectory) {
      if (!std::isfinite(point.yaw) || !std::isfinite(point.pitch))
        continue;
      planned.points.push_back(Point(AimPoint(muzzle, point.yaw, point.pitch, distance)));
    }
    if (planned.points.size() >= 2)
      entity.lines.push_back(std::move(planned));
  }

  ::foxglove::schemas::TextPrimitive status;
  status.pose = ::foxglove::schemas::Pose{
      .position =
          ::foxglove::schemas::Vector3{.x = muzzle.x(), .y = muzzle.y(), .z = muzzle.z() + 0.18},
      .orientation = ::foxglove::schemas::Quaternion{.w = 1.0}};
  status.billboard = true;
  status.font_size = 14.0;
  status.scale_invariant = true;
  status.color = value.command.fire
                     ? ::foxglove::schemas::Color{.r = 1.0, .g = 0.1, .b = 0.1, .a = 1.0}
                     : ::foxglove::schemas::Color{.r = 1.0, .g = 1.0, .b = 1.0, .a = 1.0};
  status.text = fmt::format(
      "cyan=reference yellow/red=MPC white=estimate magenta=measured green=published\n"
      "slot {} | pending {} ({:.0f} ms) | {} | {} | {} | source={}\n"
      "MPC {} axis={} index={} residual(y/p)={:.3g}/{:.3g} measure_age={:.1f}ms send={:.1f}us",
      value.selected_slot, value.armor_selection.pending_slot,
      value.armor_selection.pending_duration_s * 1.0e3,
      modules::ArmorSelectionDecisionName(value.armor_selection.decision),
      value.plan.valid ? "MPC OK" : "MPC FAIL", modules::FireRejectReasonName(value.reject_reason),
      modules::GimbalCommandSourceName(value.command_source),
      modules::GimbalTrajectoryFailureReasonName(value.plan.failure_reason),
      modules::GimbalTrajectoryAxisName(value.plan.failure_axis), value.plan.failure_index,
      MaxResidual(value.plan.yaw_solver), MaxResidual(value.plan.pitch_solver),
      value.measurement_age_s * 1.0e3, value.sink_send_time_us);
  status.text += fmt::format(
      "\nactuator={} valid={} saturation=0x{:02x} feedback={} runtime_age={}ms "
      "pitch_feedback_error={}",
      ActuatorModeName(value.actuator_telemetry.mode), value.actuator_telemetry.valid,
      value.actuator_telemetry.saturation_flags,
      modules::GimbalFeedbackSourceName(value.feedback_source),
      std::isfinite(value.runtime_actuator_age_s)
          ? fmt::format("{:.1f}", value.runtime_actuator_age_s * 1.0e3)
          : "invalid",
      value.feedback_runtime_comparison_valid
          ? fmt::format("{:.4f}", value.pitch_feedback_minus_runtime_actuator)
          : "invalid");
  entity.texts.push_back(std::move(status));
  update.entities.push_back(std::move(entity));
  return update;
}

/** @brief 以消息时间戳写入已经编码完成的 JSON 字节。 */
::foxglove::FoxgloveError Log(::foxglove::RawChannel& channel, const std::string& message,
                              std::uint64_t timestamp_ns) noexcept {
  const auto* data = reinterpret_cast<const std::byte*>(message.data());
  return channel.log(data, message.size(), timestamp_ns);
}

}  // namespace

struct ControlDebugPublisher::ChannelSet {
  /** @brief 在指定 Context 中原子式创建三条 JSON 频道和一条 SceneUpdate 频道。 */
  explicit ChannelSet(const ::foxglove::Context& context) {
    state = CreateChannel(K_STATE_TOPIC, "mv.vision.ControlState", K_STATE_SCHEMA,
                          sizeof(K_STATE_SCHEMA) - 1, context);
    tracking = CreateChannel(K_TRACKING_TOPIC, "mv.vision.GimbalTracking", K_TRACKING_SCHEMA,
                             sizeof(K_TRACKING_SCHEMA) - 1, context);
    trajectory = CreateChannel(K_TRAJECTORY_TOPIC, "mv.vision.ControlTrajectory",
                               K_TRAJECTORY_SCHEMA, sizeof(K_TRAJECTORY_SCHEMA) - 1, context);
    auto scene_result = ::foxglove::schemas::SceneUpdateChannel::create(K_SCENE_TOPIC, context);
    if (!scene_result.has_value()) {
      throw std::runtime_error(std::string("create ") + K_SCENE_TOPIC + ": " +
                               ::foxglove::strerror(scene_result.error()));
    }
    scene =
        std::make_unique<::foxglove::schemas::SceneUpdateChannel>(std::move(scene_result).value());
  }
  ~ChannelSet() { Close(); }
  /** @brief 幂等关闭当前集合内所有已创建频道。 */
  void Close() noexcept {
    if (closed)
      return;
    closed = true;
    if (state)
      state->close();
    if (tracking)
      tracking->close();
    if (trajectory)
      trajectory->close();
    if (scene)
      scene->close();
  }
  std::unique_ptr<::foxglove::RawChannel> state;       ///< 分组完整控制状态 JSON。
  std::unique_ptr<::foxglove::RawChannel> tracking;    ///< 扁平时序与误差诊断 JSON。
  std::unique_ptr<::foxglove::RawChannel> trajectory;  ///< 完整参考和计划轨迹 JSON。
  std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> scene;  ///< world 系控制场景。
  bool closed{false};  ///< 是否已执行频道关闭流程。
};

ControlDebugPublisher::ControlDebugPublisher(const Config& config,
                                             runtime::FoxgloveSession& session)
    : session_(session),
      trajectory_period_ns_(static_cast<std::uint64_t>(1.0e9 / config.image.max_fps)) {
  if (session_.LiveConfigured()) {
    try {
      live_ = std::make_unique<ChannelSet>(session_.LiveContext());
      live_state_id_ = live_->state->id();
      live_tracking_id_ = live_->tracking->id();
      live_trajectory_id_ = live_->trajectory->id();
      live_scene_id_ = live_->scene->id();
      session_.RegisterLiveChannel(live_state_id_);
      session_.RegisterLiveChannel(live_tracking_id_);
      session_.RegisterLiveChannel(live_trajectory_id_);
      session_.RegisterLiveChannel(live_scene_id_);
    } catch (const std::exception& error) {
      live_.reset();
      session_.FailLiveSetup(error.what());
    }
  }
  if (session_.RecordingConfigured()) {
    try {
      recording_ = std::make_unique<ChannelSet>(session_.RecordingContext());
    } catch (const std::exception& error) {
      recording_.reset();
      session_.FailRecordingSetup(error.what());
    }
  }
}

ControlDebugPublisher::~ControlDebugPublisher() {
  Stop();
}

void ControlDebugPublisher::Start() noexcept {
  if (!session_.AnyActive())
    return;
  accepting_.store(true, std::memory_order_release);
  try {
    worker_ = std::thread([this] { WorkerLoop(); });
  } catch (const std::exception& error) {
    accepting_.store(false, std::memory_order_release);
    MV_LOG_ERROR("Foxglove.Control", "failed to start worker: {}", error.what());
  }
}

void ControlDebugPublisher::Publish(const modules::FireControlResult& result) noexcept {
  const bool live_demand = live_ && session_.LiveActive() &&
                           (session_.Subscription(live_state_id_).subscribers > 0 ||
                            session_.Subscription(live_tracking_id_).subscribers > 0 ||
                            session_.Subscription(live_trajectory_id_).subscribers > 0 ||
                            session_.Subscription(live_scene_id_).subscribers > 0);
  if (!accepting_.load(std::memory_order_acquire) || (!live_demand && !session_.RecordingActive()))
    return;
  try {
    std::lock_guard lock(mutex_);
    // 控制线程不能因可视化积压无限增长内存；满队列优先淘汰最旧状态。
    if (queue_.size() == K_CAPACITY) {
      queue_.pop_front();
      dropped_.fetch_add(1, std::memory_order_relaxed);
    }
    queue_.push_back(result);
    condition_.notify_one();
  } catch (const std::exception& error) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
    MV_LOG_ERROR("Foxglove.Control", "failed to enqueue control state: {}", error.what());
  }
}

void ControlDebugPublisher::WorkerLoop() noexcept {
  while (true) {
    modules::FireControlResult result;
    {
      std::unique_lock lock(mutex_);
      condition_.wait(lock, [this] { return !queue_.empty() || !accepting_.load(); });
      if (queue_.empty())
        return;
      result = std::move(queue_.front());
      queue_.pop_front();
    }
    Process(result);
  }
}

void ControlDebugPublisher::UpdateHistory(const modules::FireControlResult& result) noexcept {
  constexpr std::uint64_t history_ns = 1'000'000'000ULL;
  if (result.feedback.valid) {
    estimated_history_.push_back(
        {.timestamp_ns = result.command_timestamp_ns, .feedback = result.feedback});
  }
  if (result.measurement_fresh && result.measured_feedback.valid &&
      result.measured_feedback.source_sequence != last_measured_sequence_) {
    // 优先使用相机采集 Unix 时间；缺失时由控制命令时间减量测年龄近似恢复。
    std::uint64_t timestamp_ns = result.command_timestamp_ns;
    if (result.source_capture_timestamp_ns) {
      timestamp_ns = *result.source_capture_timestamp_ns;
    } else if (std::isfinite(result.measurement_age_s) && result.measurement_age_s >= 0.0) {
      const auto age_ns = static_cast<std::uint64_t>(result.measurement_age_s * 1.0e9);
      timestamp_ns = age_ns < timestamp_ns ? timestamp_ns - age_ns : 0;
    }
    measured_history_.push_back(
        {.timestamp_ns = timestamp_ns, .feedback = result.measured_feedback});
    last_measured_sequence_ = result.measured_feedback.source_sequence;
  }
  const auto oldest =
      result.command_timestamp_ns > history_ns ? result.command_timestamp_ns - history_ns : 0;
  while (!estimated_history_.empty() && estimated_history_.front().timestamp_ns < oldest)
    estimated_history_.pop_front();
  while (!measured_history_.empty() && measured_history_.front().timestamp_ns < oldest)
    measured_history_.pop_front();
}

void ControlDebugPublisher::Process(const modules::FireControlResult& result) noexcept {
  try {
    UpdateHistory(result);
    const auto state_json = EncodeState(result, dropped_.load(std::memory_order_relaxed));
    const auto tracking_json = EncodeTracking(result);
    const bool trajectory_sample =
        result.source_sequence != last_trajectory_sequence_ &&
        (last_trajectory_timestamp_ns_ == 0 ||
         result.command_timestamp_ns >= last_trajectory_timestamp_ns_ + trajectory_period_ns_);
    // 数组轨迹和三维场景编码较重，按图像帧率采样；标量频道仍保留每个控制周期。
    std::string trajectory_json;
    ::foxglove::schemas::SceneUpdate scene;
    if (trajectory_sample) {
      last_trajectory_sequence_ = result.source_sequence;
      last_trajectory_timestamp_ns_ = result.command_timestamp_ns;
      trajectory_json = EncodeTrajectory(result, estimated_history_, measured_history_);
      scene = EncodeScene(result, estimated_history_, measured_history_);
    }
    // Foxglove Context 的写入由会话级互斥量串行化，避免其他发布器并发写 live/MCAP。
    std::lock_guard publish_lock(session_.PublishMutex());
    if (live_ && session_.LiveActive()) {
      if (session_.Subscription(live_state_id_).subscribers > 0) {
        const auto error = Log(*live_->state, state_json, result.command_timestamp_ns);
        if (error != ::foxglove::FoxgloveError::Ok) {
          session_.ReportLiveError("publish control state", error);
        }
      }
      if (session_.Subscription(live_tracking_id_).subscribers > 0) {
        const auto error = Log(*live_->tracking, tracking_json, result.command_timestamp_ns);
        if (error != ::foxglove::FoxgloveError::Ok) {
          session_.ReportLiveError("publish gimbal tracking", error);
        }
      }
      if (trajectory_sample && session_.Subscription(live_trajectory_id_).subscribers > 0) {
        const auto error = Log(*live_->trajectory, trajectory_json, result.command_timestamp_ns);
        if (error != ::foxglove::FoxgloveError::Ok) {
          session_.ReportLiveError("publish control trajectory", error);
        }
      }
      if (trajectory_sample && session_.Subscription(live_scene_id_).subscribers > 0) {
        const auto error = live_->scene->log(scene, result.command_timestamp_ns);
        if (error != ::foxglove::FoxgloveError::Ok) {
          session_.ReportLiveError("publish control scene", error);
        }
      }
    }
    if (recording_ && session_.RecordingActive()) {
      if (const auto error = Log(*recording_->state, state_json, result.command_timestamp_ns);
          error != ::foxglove::FoxgloveError::Ok) {
        session_.ReportRecordingError("record control state", error);
      }
      if (const auto error = Log(*recording_->tracking, tracking_json, result.command_timestamp_ns);
          error != ::foxglove::FoxgloveError::Ok) {
        session_.ReportRecordingError("record gimbal tracking", error);
      }
      if (trajectory_sample) {
        const auto error =
            Log(*recording_->trajectory, trajectory_json, result.command_timestamp_ns);
        if (error != ::foxglove::FoxgloveError::Ok) {
          session_.ReportRecordingError("record control trajectory", error);
        }
        const auto scene_error = recording_->scene->log(scene, result.command_timestamp_ns);
        if (scene_error != ::foxglove::FoxgloveError::Ok) {
          session_.ReportRecordingError("record control scene", scene_error);
        }
      }
    }
  } catch (const std::exception& error) {
    MV_LOG_ERROR("Foxglove.Control", "control serialization failed: {}", error.what());
  }
}

void ControlDebugPublisher::Stop() noexcept {
  if (stop_called_.exchange(true, std::memory_order_acq_rel))
    return;
  accepting_.store(false, std::memory_order_release);
  condition_.notify_all();
  if (worker_.joinable())
    worker_.join();
  if (live_)
    live_->Close();
  if (recording_)
    recording_->Close();
}

std::uint64_t ControlDebugPublisher::DroppedSamples() const noexcept {
  return dropped_.load(std::memory_order_relaxed);
}

}  // namespace mv::tool::foxglove::control
