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

RuntimeDecision ControlRuntimeImpl::ObserveCommandChannel(
    bool sink_healthy, bool send_succeeded, std::chrono::steady_clock::time_point now) noexcept {
  if (!sink_healthy) {
    return supervisor_.ReportAt(RuntimeFaultCode::COMMAND_SINK_UNHEALTHY,
                                "Talos command heartbeat is unhealthy", now);
  }
  if (!send_succeeded) {
    return supervisor_.ReportAt(RuntimeFaultCode::COMMAND_SEND_FAILED,
                                "Talos command publication failed", now);
  }
  supervisor_.RecoverAt(RuntimeComponent::COMMAND_SINK, now);
  return {};
}

void ControlRuntimeImpl::ProcessSnapshot(
    const std::shared_ptr<const modules::ControlInputSnapshot>& snapshot, LoopState& state,
    std::chrono::steady_clock::time_point now, const CycleTiming& timing) {
  const auto SYSTEM_NOW_NS = SystemNowNs();
  const bool SINK_HEALTHY = sink_->IsHealthy();
  const auto COMPUTE_START = std::chrono::steady_clock::now();
  auto input = *snapshot;
  input.external_control_enabled = sink_->ExternalControlEnabled();
  auto result = session_.Step(input, sink_->ActuatorTelemetry(), SINK_HEALTHY, now, SYSTEM_NOW_NS);
  const auto SEND_START = std::chrono::steady_clock::now();
  const bool SEND_SUCCEEDED = sink_->Send(result.output.command);
  const auto SEND_END = std::chrono::steady_clock::now();
  session_.AcknowledgePublication(result, SEND_SUCCEEDED, now);
  const auto COMPUTE_END = std::chrono::steady_clock::now();
  auto& output = result.output;
  auto& diagnostics = result.diagnostics;
  output.talos_heartbeat_ns = sink_->HeartbeatTimestampNs();
  diagnostics.control_period_s = timing.period_s;
  diagnostics.deadline_lateness_us = timing.deadline_lateness_us;
  diagnostics.sink_send_time_us =
      std::chrono::duration<double, std::micro>(SEND_END - SEND_START).count();
  diagnostics.control_compute_time_us =
      std::chrono::duration<double, std::micro>(COMPUTE_END - COMPUTE_START).count() -
      diagnostics.sink_send_time_us;
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
  const auto CHANNEL_DECISION = ObserveCommandChannel(SINK_HEALTHY, SEND_SUCCEEDED, now);
  if (CHANNEL_DECISION.request_safe_stop)
    SendStop();
  if (CHANNEL_DECISION.termination) {
    running_.store(false, std::memory_order_release);
  }
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
        const bool SINK_HEALTHY = sink_->IsHealthy();
        const bool STOP_SENT = SendStop();
        const auto DECISION = ObserveCommandChannel(SINK_HEALTHY, STOP_SENT, NOW);
        if (DECISION.termination)
          running_.store(false, std::memory_order_release);
      }

      next += PERIOD;
      const auto FINISHED = std::chrono::steady_clock::now();
      if (FINISHED >= next + PERIOD)
        next = FINISHED + PERIOD;
      std::this_thread::sleep_until(next);
    }
  } catch (const std::exception& error) {
    running_.store(false, std::memory_order_release);
    SendStop();
    static_cast<void>(supervisor_.Report(RuntimeFaultCode::CONTROL_THREAD_EXCEPTION, error.what()));
    MV_LOG_ERROR("Control", "100 Hz control thread stopped after exception: {}", error.what());
  } catch (...) {
    running_.store(false, std::memory_order_release);
    SendStop();
    static_cast<void>(supervisor_.Report(RuntimeFaultCode::CONTROL_THREAD_EXCEPTION,
                                         "unknown control thread exception"));
    MV_LOG_ERROR("Control", "100 Hz control thread stopped after unknown exception");
  }
}

}  // namespace mv::runtime
