#include "core/logger.hpp"
#include "runtime/control_runtime_impl.hpp"
#include "runtime/runtime_diagnostics_sink.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <thread>

#include <numbers>
#include <optional>

namespace mv::runtime {
namespace {

std::uint64_t SystemNowNs() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}

}  // namespace

void ControlRuntimeImpl::ProcessSnapshot(
    const std::shared_ptr<const modules::ControlInputSnapshot>& snapshot, LoopState& state,
    std::chrono::steady_clock::time_point now, const CycleTiming& timing) {
  const auto SYSTEM_NOW_NS = SystemNowNs();
  const auto ACTUATOR = sink_->ActuatorTelemetry();
  const std::optional<hal::GimbalActuatorMode> ACTUATOR_MODE =
      ACTUATOR.valid ? std::optional(ACTUATOR.mode) : std::nullopt;
  if (ACTUATOR_MODE != state.last_actuator_mode) {
    feedback_estimator_.ClearRuntimeActuator();
    ClearPublishedProjection("actuator_mode_changed");
    state.last_actuator_mode = ACTUATOR_MODE;
  }

  auto input = *snapshot;
  input.external_control_enabled = sink_->ExternalControlEnabled();
  if (!state.last_external_control ||
      *state.last_external_control != input.external_control_enabled) {
    if (input.external_control_enabled) {
      MV_LOG_INFO("Control", "Talos external auto-aim subscription enabled");
    } else {
      MV_LOG_WARN("Control",
                  "Talos external auto-aim subscription is disabled; press F5 in simulation");
    }
    feedback_estimator_.ClearRuntimeActuator();
    ClearPublishedProjection(input.external_control_enabled ? "external_control_enabled"
                                                            : "external_control_disabled");
    state.last_external_control = input.external_control_enabled;
  }

  const bool RUNTIME_WAS_ACTIVE = feedback_estimator_.RuntimeActuatorActive();
  feedback_estimator_.ObserveActuatorTelemetry(ACTUATOR, now, SYSTEM_NOW_NS);
  if (RUNTIME_WAS_ACTIVE != feedback_estimator_.RuntimeActuatorActive()) {
    ClearPublishedProjection(feedback_estimator_.RuntimeActuatorActive()
                                 ? "runtime_actuator_enabled"
                                 : "runtime_actuator_invalid");
  }

  bool measurement_fresh = false;
  if (snapshot->prediction.sequence != state.observed_sequence) {
    feedback_estimator_.ObserveMeasurement(snapshot->prediction.sequence,
                                           snapshot->prediction.source_receive_steady_time,
                                           snapshot->world_t_gimbal, snapshot->frame_actuator);
    state.observed_sequence = snapshot->prediction.sequence;
    measurement_fresh = true;
    state.matched_command =
        MatchCommand(snapshot->prediction.source_capture_timestamp_ns, snapshot->frame_actuator);
  }
  const bool SINK_HEALTHY = sink_->IsHealthy();
  if (!state.last_sink_healthy || *state.last_sink_healthy != SINK_HEALTHY) {
    if (!SINK_HEALTHY)
      ClearPublishedProjection("talos_unhealthy");
    state.last_sink_healthy = SINK_HEALTHY;
  }
  const auto FEEDBACK = feedback_estimator_.Estimate(now);
  const auto FEEDBACK_SOURCE = feedback_estimator_.Source();
  auto result = fire_control_.Step(input, FEEDBACK, now);
  auto& output = result.output;
  auto& diagnostics = result.diagnostics;
  if (output.tracking_object_reset && control_projection_active_) {
    feedback_estimator_.ClearRuntimeActuator();
    ClearPublishedProjection("tracking_object_changed");
  }
  diagnostics.feedback_source = FEEDBACK_SOURCE;
  diagnostics.measured_feedback = feedback_estimator_.LastMeasurement();
  diagnostics.measurement_fresh = measurement_fresh;
  diagnostics.measurement_age_s =
      diagnostics.measured_feedback.valid
          ? std::max(0.0,
                     std::chrono::duration<double>(now - diagnostics.measured_feedback.timestamp)
                         .count())
          : std::numeric_limits<double>::infinity();
  diagnostics.matched_prior_command = state.matched_command;
  diagnostics.actuator_telemetry = ACTUATOR;
  diagnostics.frame_actuator_telemetry = snapshot->frame_actuator;
  diagnostics.runtime_actuator_age_s = feedback_estimator_.RuntimeActuatorAgeS();
  diagnostics.feedback_projection_dt_s = feedback_estimator_.ProjectionDtS();
  diagnostics.feedback_runtime_state_timestamp_ns = feedback_estimator_.RuntimeStateTimestampNs();
  if (snapshot->frame_actuator && snapshot->frame_actuator->valid &&
      snapshot->frame_actuator->state_timestamp_ns != 0 &&
      snapshot->frame_actuator->state_timestamp_ns <= SYSTEM_NOW_NS) {
    diagnostics.frame_actuator_age_s =
        static_cast<double>(SYSTEM_NOW_NS - snapshot->frame_actuator->state_timestamp_ns) * 1.0e-9;
  } else {
    diagnostics.frame_actuator_age_s = std::numeric_limits<double>::infinity();
  }
  diagnostics.feedback_runtime_comparison_valid =
      FEEDBACK.valid && ACTUATOR.valid && ACTUATOR.mode == hal::GimbalActuatorMode::PHYSICAL;
  if (diagnostics.feedback_runtime_comparison_valid) {
    diagnostics.yaw_feedback_minus_runtime_actuator =
        std::remainder(FEEDBACK.yaw - ACTUATOR.actual_yaw, 2.0 * std::numbers::pi);
    diagnostics.pitch_feedback_minus_runtime_actuator = FEEDBACK.pitch - ACTUATOR.actual_pitch;
  }
  diagnostics.frame_runtime_comparison_valid =
      snapshot->frame_actuator && snapshot->frame_actuator->valid && ACTUATOR.valid &&
      snapshot->frame_actuator->mode == hal::GimbalActuatorMode::PHYSICAL &&
      ACTUATOR.mode == hal::GimbalActuatorMode::PHYSICAL;
  if (diagnostics.frame_runtime_comparison_valid) {
    const auto& frame = *snapshot->frame_actuator;
    diagnostics.yaw_frame_minus_runtime_actuator =
        std::remainder(frame.actual_yaw - ACTUATOR.actual_yaw, 2.0 * std::numbers::pi);
    diagnostics.pitch_frame_minus_runtime_actuator = frame.actual_pitch - ACTUATOR.actual_pitch;
    diagnostics.yaw_frame_acceleration_minus_runtime =
        frame.yaw_acceleration - ACTUATOR.yaw_acceleration;
    diagnostics.pitch_frame_acceleration_minus_runtime =
        frame.pitch_acceleration - ACTUATOR.pitch_acceleration;
  }
  diagnostics.control_period_s = timing.period_s;
  diagnostics.deadline_lateness_us = timing.deadline_lateness_us;
  output.command_sink_healthy = SINK_HEALTHY;
  output.talos_heartbeat_ns = sink_->HeartbeatTimestampNs();

  if (output.reject_reason == modules::FireRejectReason::MPC_FAILED) {
    ++consecutive_mpc_failure_cycles_;
  } else {
    consecutive_mpc_failure_cycles_ = 0;
  }
  output.consecutive_mpc_failure_cycles = consecutive_mpc_failure_cycles_;

  if (output.reject_reason == modules::FireRejectReason::MPC_FAILED &&
      !last_successful_trajectory_.empty() && output.external_control_enabled &&
      output.command_sink_healthy && !output.tracking_object_reset &&
      output.selected_slot == last_successful_plan_slot_) {
    const double FALLBACK_AGE_S =
        std::max(0.0, std::chrono::duration<double>(now - last_successful_plan_time_).count());
    output.fallback_age_s = FALLBACK_AGE_S;
    output.fallback_source_slot = last_successful_plan_slot_;
    constexpr double MAX_FALLBACK_AGE_S = 0.100;
    const auto ELAPSED_STEPS =
        static_cast<std::size_t>(std::max(1LL, std::llround(FALLBACK_AGE_S / PLANNER_DT_S)));
    const auto FALLBACK_INDEX = last_successful_command_index_ + ELAPSED_STEPS;
    if (FALLBACK_AGE_S <= MAX_FALLBACK_AGE_S &&
        FALLBACK_INDEX < last_successful_trajectory_.size()) {
      const auto& point = last_successful_trajectory_[FALLBACK_INDEX];
      output.command = {.valid = true,
                        .fire = false,
                        .timestamp_ns = output.command_timestamp_ns,
                        .yaw = std::remainder(point.yaw, 2.0 * std::numbers::pi),
                        .yaw_velocity = point.yaw_velocity,
                        .yaw_acceleration = point.yaw_acceleration,
                        .pitch = point.pitch,
                        .pitch_velocity = point.pitch_velocity,
                        .pitch_acceleration = point.pitch_acceleration,
                        .target_distance_m = last_successful_target_distance_m_};
      output.command.valid = true;
      output.command.fire = false;
      output.command_source = modules::GimbalCommandSource::TRAJECTORY_FALLBACK;
      output.fallback_active = true;
      output.fallback_trajectory_index = static_cast<int>(FALLBACK_INDEX);
      output.fallback_remaining_points =
          static_cast<int>(last_successful_trajectory_.size() - FALLBACK_INDEX - 1);
    }
  }

  if (!output.command_sink_healthy || !output.external_control_enabled) {
    output.command.valid = false;
    output.command.fire = false;
    output.command_source = modules::GimbalCommandSource::STOP;
    output.reject_reason = modules::FireRejectReason::TALOS_UNHEALTHY;
    if (!output.external_control_enabled)
      output.reject_reason = modules::FireRejectReason::EXTERNAL_CONTROL_DISABLED;
  }

  if (!output.command.valid) {
    output.command_source = modules::GimbalCommandSource::STOP;
    output.command.fire = false;
    if (control_projection_active_) {
      if (output.reject_reason == modules::FireRejectReason::MPC_FAILED) {
        diagnostics.fallback_expired_this_cycle = true;
        ClearPublishedProjection("mpc_fallback_expired");
      } else {
        ClearPublishedProjection(modules::FireRejectReasonName(output.reject_reason));
      }
    }
  }
  const auto SEND_START = std::chrono::steady_clock::now();
  const bool SEND_SUCCEEDED = sink_->Send(output.command);
  diagnostics.sink_send_time_us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - SEND_START)
          .count();
  output.command_publish_succeeded = SEND_SUCCEEDED && output.command.valid;
  output.published_valid = output.command_publish_succeeded;
  if (!SEND_SUCCEEDED) {
    output.command_sink_healthy = false;
    output.command.valid = false;
    output.command.fire = false;
    output.reject_reason = modules::FireRejectReason::TALOS_UNHEALTHY;
    output.command_source = modules::GimbalCommandSource::STOP;
    output.published_valid = false;
    ClearPublishedProjection("command_send_failed");
  }
  if (SEND_SUCCEEDED)
    RememberCommand(output.command);
  if (output.command_publish_succeeded) {
    feedback_estimator_.ObservePublishedCommand(output.command, now, false);
    control_projection_active_ = true;
    if (output.command_source == modules::GimbalCommandSource::MPC) {
      last_successful_trajectory_ = output.plan.trajectory;
      last_successful_command_index_ = static_cast<std::size_t>(output.plan.command_index);
      last_successful_target_distance_m_ = output.command.target_distance_m;
      last_successful_plan_time_ = now;
      last_successful_plan_slot_ = output.selected_slot;
    }
  }
  AttachProjectionDiagnostics(result);
  if (++state.control_cycles % 100 == 0) {
    MV_LOG_INFO("Control",
                "seq={} tracker={} slot={} command={} fire={} reject={} "
                "age(pred/fb)={:.1f}/{:.1f}ms mpc={} iter={}/{} solve={:.1f}us",
                output.source_sequence, modules::TrackerStateName(output.tracker_state),
                output.selected_slot, output.command.valid, output.command.fire,
                modules::FireRejectReasonName(output.reject_reason),
                output.prediction_age_s * 1.0e3, output.feedback_age_s * 1.0e3, output.plan.valid,
                diagnostics.plan.yaw_iterations, diagnostics.plan.pitch_iterations,
                diagnostics.plan.solve_time_us);
  }
  if (diagnostics_)
    diagnostics_->PublishControl({.fire_control = output}, {.fire_control = diagnostics});
}

void ControlRuntimeImpl::Loop() noexcept {
  LoopState state;
  auto next = std::chrono::steady_clock::now();
  auto previous_cycle = next;
  try {
    while (running_.load(std::memory_order_acquire)) {
      const auto NOW = std::chrono::steady_clock::now();
      const double CONTROL_PERIOD_S = std::chrono::duration<double>(NOW - previous_cycle).count();
      previous_cycle = NOW;
      const double DEADLINE_LATENESS_US =
          NOW > next ? std::chrono::duration<double, std::micro>(NOW - next).count() : 0.0;
      auto snapshot = std::atomic_load_explicit(&latest_snapshot_, std::memory_order_acquire);
      if (snapshot) {
        ProcessSnapshot(
            snapshot, state, NOW,
            {.period_s = CONTROL_PERIOD_S, .deadline_lateness_us = DEADLINE_LATENESS_US});
      } else {
        SendStop();
      }

      next += PERIOD;
      const auto FINISHED = std::chrono::steady_clock::now();
      if (FINISHED >= next + PERIOD)
        next = FINISHED + PERIOD;
      std::this_thread::sleep_until(next);
    }
  } catch (const std::exception& error) {
    failed_.store(true, std::memory_order_release);
    running_.store(false, std::memory_order_release);
    SendStop();
    MV_LOG_ERROR("Control", "100 Hz control thread stopped after exception: {}", error.what());
  } catch (...) {
    failed_.store(true, std::memory_order_release);
    running_.store(false, std::memory_order_release);
    SendStop();
    MV_LOG_ERROR("Control", "100 Hz control thread stopped after unknown exception");
  }
}

}  // namespace mv::runtime
