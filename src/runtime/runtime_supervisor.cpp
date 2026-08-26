#include "runtime/runtime_supervisor.hpp"

#include "core/logger.hpp"

#include <mutex>

namespace mv::runtime {
namespace {

using Clock = std::chrono::steady_clock;

struct FaultRule {
  RuntimeComponent component;
  RuntimeFaultAction initial_action;
  std::optional<RuntimeTerminationReason> immediate_termination;
};

FaultRule RuleFor(RuntimeFaultCode code) noexcept {
  switch (code) {
    case RuntimeFaultCode::CAMERA_TIMEOUT:
    case RuntimeFaultCode::CAMERA_INVALID_FRAME:
      return {RuntimeComponent::CAMERA, RuntimeFaultAction::SKIP_CURRENT_WORK, std::nullopt};
    case RuntimeFaultCode::CAMERA_DISCONNECTED:
    case RuntimeFaultCode::CAMERA_FATAL:
      return {RuntimeComponent::CAMERA, RuntimeFaultAction::SAFE_STOP_AND_TERMINATE,
              RuntimeTerminationReason::CAMERA_FAILURE};
    case RuntimeFaultCode::VISION_PIPELINE_EXCEPTION:
      return {RuntimeComponent::VISION_PIPELINE, RuntimeFaultAction::SAFE_STOP_AND_TERMINATE,
              RuntimeTerminationReason::PIPELINE_FAILURE};
    case RuntimeFaultCode::CONTROL_UPDATE_EXCEPTION:
    case RuntimeFaultCode::CONTROL_THREAD_EXCEPTION:
      return {RuntimeComponent::CONTROL_RUNTIME, RuntimeFaultAction::SAFE_STOP_AND_TERMINATE,
              RuntimeTerminationReason::CONTROL_FAILURE};
    case RuntimeFaultCode::COMMAND_SINK_UNHEALTHY:
    case RuntimeFaultCode::COMMAND_SEND_FAILED:
      return {RuntimeComponent::COMMAND_SINK, RuntimeFaultAction::SAFE_STOP, std::nullopt};
    case RuntimeFaultCode::EVALUATION_INIT_FAILURE:
    case RuntimeFaultCode::EVALUATION_EXCEPTION:
      return {RuntimeComponent::SIMULATION_EVALUATION, RuntimeFaultAction::SKIP_CURRENT_WORK,
              std::nullopt};
    case RuntimeFaultCode::DEBUG_WINDOW_INIT_FAILURE:
    case RuntimeFaultCode::DEBUG_WINDOW_EXCEPTION:
      return {RuntimeComponent::DEBUG_WINDOW, RuntimeFaultAction::SKIP_CURRENT_WORK, std::nullopt};
    case RuntimeFaultCode::DIAGNOSTICS_INIT_FAILURE:
    case RuntimeFaultCode::DIAGNOSTICS_UNAVAILABLE:
    case RuntimeFaultCode::DIAGNOSTICS_EXCEPTION:
      return {RuntimeComponent::DIAGNOSTICS, RuntimeFaultAction::CONTINUE, std::nullopt};
  }
  return {RuntimeComponent::DIAGNOSTICS, RuntimeFaultAction::CONTINUE, std::nullopt};
}

std::size_t Index(RuntimeComponent component) noexcept {
  return static_cast<std::size_t>(component);
}

bool RequestsSafeStop(RuntimeFaultAction action) noexcept {
  return action == RuntimeFaultAction::SAFE_STOP ||
         action == RuntimeFaultAction::SAFE_STOP_AND_TERMINATE;
}

}  // namespace

struct RuntimeSupervisor::Impl {
  struct ComponentState {
    RuntimeHealthState health{RuntimeHealthState::HEALTHY};
    std::uint64_t total_events{0};
    bool active{false};
    bool has_healthy_observation{false};
    Clock::time_point last_healthy{};
    RuntimeFault fault;
  };

  explicit Impl(RuntimeErrorPolicy input_policy) : policy(input_policy) {
    for (std::size_t index = 0; index < components.size(); ++index) {
      components[index].fault.component = static_cast<RuntimeComponent>(index);
    }
  }

  RuntimeDecision ReportAt(RuntimeFaultCode code, std::string_view detail,
                           Clock::time_point now) noexcept {
    try {
      const auto RULE = RuleFor(code);
      std::lock_guard lock(mutex);
      auto& component = components[Index(RULE.component)];
      const bool ENTERING = !component.active;
      ++component.total_events;
      if (ENTERING) {
        component.active = true;
        component.health = RuntimeHealthState::DEGRADED;
        component.fault = {.component = RULE.component,
                           .code = code,
                           .state = RuntimeHealthState::DEGRADED,
                           .action = RULE.initial_action,
                           .total_occurrences = component.total_events,
                           .consecutive_occurrences = 1,
                           .first_seen = now,
                           .last_seen = now,
                           .detail = std::string(detail)};
      } else {
        component.fault.code = code;
        component.fault.action = RULE.initial_action;
        component.fault.total_occurrences = component.total_events;
        ++component.fault.consecutive_occurrences;
        component.fault.last_seen = now;
        if (!detail.empty())
          component.fault.detail.assign(detail);
      }

      RuntimeDecision decision{
          .state = component.health,
          .skip_current_work = RULE.initial_action == RuntimeFaultAction::SKIP_CURRENT_WORK,
          .disable_component = false,
          .request_safe_stop = RequestsSafeStop(RULE.initial_action),
          .termination = RULE.immediate_termination};
      bool terminal_latched_now = false;
      if (RULE.component == RuntimeComponent::CAMERA && !decision.termination) {
        const auto START =
            component.has_healthy_observation ? component.last_healthy : component.fault.first_seen;
        if (now - START >= policy.camera_no_valid_frame_timeout) {
          decision.request_safe_stop = true;
          decision.termination = RuntimeTerminationReason::CAMERA_FAILURE;
        }
      } else if (RULE.component == RuntimeComponent::COMMAND_SINK) {
        const auto START =
            component.has_healthy_observation ? component.last_healthy : component.fault.first_seen;
        if (now - START >= policy.command_sink_unhealthy_timeout) {
          decision.request_safe_stop = true;
          decision.termination = RuntimeTerminationReason::CONTROL_FAILURE;
        }
      } else if (RULE.component == RuntimeComponent::SIMULATION_EVALUATION) {
        decision.skip_current_work = true;
        decision.disable_component =
            code == RuntimeFaultCode::EVALUATION_INIT_FAILURE ||
            component.fault.consecutive_occurrences ==
                static_cast<std::uint64_t>(policy.evaluation_disable_after_consecutive_errors);
      } else if (RULE.component == RuntimeComponent::DEBUG_WINDOW) {
        decision.skip_current_work = true;
        decision.disable_component =
            code == RuntimeFaultCode::DEBUG_WINDOW_INIT_FAILURE ||
            component.fault.consecutive_occurrences ==
                static_cast<std::uint64_t>(policy.debug_window_disable_after_consecutive_errors);
      }

      if (decision.disable_component)
        component.fault.action = RuntimeFaultAction::DISABLE_COMPONENT;
      if (decision.termination) {
        component.health = RuntimeHealthState::FAILED;
        component.fault.state = RuntimeHealthState::FAILED;
        component.fault.action = decision.request_safe_stop
                                     ? RuntimeFaultAction::SAFE_STOP_AND_TERMINATE
                                     : RuntimeFaultAction::TERMINATE;
        decision.state = RuntimeHealthState::FAILED;
        if (!terminal_fault) {
          terminal_fault = component.fault;
          terminal_latched_now = true;
        }
      }

      if (ENTERING && !decision.termination) {
        MV_LOG_WARN("Runtime", "component={} degraded code={} action={} detail={}",
                    RuntimeComponentName(RULE.component), RuntimeFaultCodeName(code),
                    RuntimeFaultActionName(component.fault.action), component.fault.detail);
      }
      if (decision.disable_component &&
          (code == RuntimeFaultCode::EVALUATION_INIT_FAILURE ||
           code == RuntimeFaultCode::DEBUG_WINDOW_INIT_FAILURE ||
           component.fault.consecutive_occurrences ==
               (RULE.component == RuntimeComponent::SIMULATION_EVALUATION
                    ? static_cast<std::uint64_t>(policy.evaluation_disable_after_consecutive_errors)
                    : static_cast<std::uint64_t>(
                          policy.debug_window_disable_after_consecutive_errors)))) {
        MV_LOG_WARN("Runtime", "component={} disabled code={} after {} consecutive errors",
                    RuntimeComponentName(RULE.component), RuntimeFaultCodeName(code),
                    component.fault.consecutive_occurrences);
      }
      if (terminal_latched_now) {
        MV_LOG_ERROR("Runtime", "component={} failed code={} action={} consecutive={} detail={}",
                     RuntimeComponentName(RULE.component), RuntimeFaultCodeName(code),
                     RuntimeFaultActionName(component.fault.action),
                     component.fault.consecutive_occurrences, component.fault.detail);
      }
      return decision;
    } catch (...) {
      // 健康监督本身不能让正式控制线程因诊断分配失败而终止。
      return {.state = RuntimeHealthState::DEGRADED,
              .skip_current_work = false,
              .disable_component = false,
              .request_safe_stop = false,
              .termination = std::nullopt};
    }
  }

  void RecoverAt(RuntimeComponent runtime_component, Clock::time_point now) noexcept {
    if (runtime_component == RuntimeComponent::COUNT)
      return;
    try {
      std::lock_guard lock(mutex);
      auto& component = components[Index(runtime_component)];
      component.has_healthy_observation = true;
      component.last_healthy = now;
      if (!component.active || component.health == RuntimeHealthState::FAILED)
        return;
      const auto DURATION_MS =
          std::chrono::duration<double, std::milli>(now - component.fault.first_seen).count();
      MV_LOG_INFO("Runtime", "component={} recovered code={} after {:.1f}ms and {} events",
                  RuntimeComponentName(runtime_component),
                  RuntimeFaultCodeName(component.fault.code), DURATION_MS,
                  component.fault.consecutive_occurrences);
      component.active = false;
      component.health = RuntimeHealthState::HEALTHY;
    } catch (...) {
    }
  }

  RuntimeHealthSnapshot Snapshot() const noexcept {
    RuntimeHealthSnapshot result;
    try {
      std::lock_guard lock(mutex);
      result.terminal_fault = terminal_fault;
      result.overall = terminal_fault ? RuntimeHealthState::FAILED : RuntimeHealthState::HEALTHY;
      for (std::size_t index = 0; index < components.size(); ++index) {
        const auto& component = components[index];
        auto& output = result.components[index];
        output.component = static_cast<RuntimeComponent>(index);
        output.state = component.health;
        output.total_events = component.total_events;
        if (component.active)
          output.active_fault = component.fault;
        if (!terminal_fault && component.active)
          result.overall = RuntimeHealthState::DEGRADED;
      }
    } catch (...) {
      result.overall = RuntimeHealthState::DEGRADED;
    }
    return result;
  }

  std::optional<RuntimeRunResult> TerminalResult() const noexcept {
    try {
      std::lock_guard lock(mutex);
      if (!terminal_fault)
        return std::nullopt;
      RuntimeTerminationReason reason = RuntimeTerminationReason::CONTROL_FAILURE;
      switch (terminal_fault->component) {
        case RuntimeComponent::CAMERA:
          reason = RuntimeTerminationReason::CAMERA_FAILURE;
          break;
        case RuntimeComponent::VISION_PIPELINE:
          reason = RuntimeTerminationReason::PIPELINE_FAILURE;
          break;
        case RuntimeComponent::CONTROL_RUNTIME:
        case RuntimeComponent::COMMAND_SINK:
          reason = RuntimeTerminationReason::CONTROL_FAILURE;
          break;
        case RuntimeComponent::SIMULATION_EVALUATION:
        case RuntimeComponent::DEBUG_WINDOW:
        case RuntimeComponent::DIAGNOSTICS:
        case RuntimeComponent::COUNT:
          return std::nullopt;
      }
      return RuntimeRunResult{.reason = reason, .fault = terminal_fault};
    } catch (...) {
      return RuntimeRunResult{.reason = RuntimeTerminationReason::CONTROL_FAILURE,
                              .fault = std::nullopt};
    }
  }

  RuntimeErrorPolicy policy;
  mutable std::mutex mutex;
  std::array<ComponentState, K_RUNTIME_COMPONENT_COUNT> components;
  std::optional<RuntimeFault> terminal_fault;
};

const char* RuntimeComponentName(RuntimeComponent component) noexcept {
  switch (component) {
    case RuntimeComponent::CAMERA:
      return "camera";
    case RuntimeComponent::VISION_PIPELINE:
      return "vision_pipeline";
    case RuntimeComponent::CONTROL_RUNTIME:
      return "control_runtime";
    case RuntimeComponent::COMMAND_SINK:
      return "command_sink";
    case RuntimeComponent::SIMULATION_EVALUATION:
      return "simulation_evaluation";
    case RuntimeComponent::DEBUG_WINDOW:
      return "debug_window";
    case RuntimeComponent::DIAGNOSTICS:
      return "diagnostics";
    case RuntimeComponent::COUNT:
      return "count";
  }
  return "unknown";
}

const char* RuntimeFaultCodeName(RuntimeFaultCode code) noexcept {
  switch (code) {
    case RuntimeFaultCode::CAMERA_TIMEOUT:
      return "camera_timeout";
    case RuntimeFaultCode::CAMERA_INVALID_FRAME:
      return "camera_invalid_frame";
    case RuntimeFaultCode::CAMERA_DISCONNECTED:
      return "camera_disconnected";
    case RuntimeFaultCode::CAMERA_FATAL:
      return "camera_fatal";
    case RuntimeFaultCode::VISION_PIPELINE_EXCEPTION:
      return "vision_pipeline_exception";
    case RuntimeFaultCode::CONTROL_UPDATE_EXCEPTION:
      return "control_update_exception";
    case RuntimeFaultCode::CONTROL_THREAD_EXCEPTION:
      return "control_thread_exception";
    case RuntimeFaultCode::COMMAND_SINK_UNHEALTHY:
      return "command_sink_unhealthy";
    case RuntimeFaultCode::COMMAND_SEND_FAILED:
      return "command_send_failed";
    case RuntimeFaultCode::EVALUATION_INIT_FAILURE:
      return "evaluation_init_failure";
    case RuntimeFaultCode::EVALUATION_EXCEPTION:
      return "evaluation_exception";
    case RuntimeFaultCode::DEBUG_WINDOW_INIT_FAILURE:
      return "debug_window_init_failure";
    case RuntimeFaultCode::DEBUG_WINDOW_EXCEPTION:
      return "debug_window_exception";
    case RuntimeFaultCode::DIAGNOSTICS_INIT_FAILURE:
      return "diagnostics_init_failure";
    case RuntimeFaultCode::DIAGNOSTICS_UNAVAILABLE:
      return "diagnostics_unavailable";
    case RuntimeFaultCode::DIAGNOSTICS_EXCEPTION:
      return "diagnostics_exception";
  }
  return "unknown";
}

const char* RuntimeFaultActionName(RuntimeFaultAction action) noexcept {
  switch (action) {
    case RuntimeFaultAction::CONTINUE:
      return "continue";
    case RuntimeFaultAction::SKIP_CURRENT_WORK:
      return "skip_current_work";
    case RuntimeFaultAction::DISABLE_COMPONENT:
      return "disable_component";
    case RuntimeFaultAction::SAFE_STOP:
      return "safe_stop";
    case RuntimeFaultAction::TERMINATE:
      return "terminate";
    case RuntimeFaultAction::SAFE_STOP_AND_TERMINATE:
      return "safe_stop_and_terminate";
  }
  return "unknown";
}

int RuntimeExitCode(RuntimeTerminationReason reason) noexcept {
  switch (reason) {
    case RuntimeTerminationReason::NORMAL:
      return 0;
    case RuntimeTerminationReason::CAMERA_FAILURE:
      return 4;
    case RuntimeTerminationReason::PIPELINE_FAILURE:
      return 5;
    case RuntimeTerminationReason::CONTROL_FAILURE:
      return 7;
  }
  return 1;
}

RuntimeSupervisor::RuntimeSupervisor(RuntimeErrorPolicy policy)
    : impl_(std::make_unique<Impl>(policy)) {}

RuntimeSupervisor::~RuntimeSupervisor() = default;

RuntimeDecision RuntimeSupervisor::Report(RuntimeFaultCode code, std::string_view detail) noexcept {
  return impl_->ReportAt(code, detail, Clock::now());
}

RuntimeDecision RuntimeSupervisor::ReportAt(RuntimeFaultCode code, std::string_view detail,
                                            Clock::time_point now) noexcept {
  return impl_->ReportAt(code, detail, now);
}

void RuntimeSupervisor::Recover(RuntimeComponent component) noexcept {
  impl_->RecoverAt(component, Clock::now());
}

void RuntimeSupervisor::RecoverAt(RuntimeComponent component, Clock::time_point now) noexcept {
  impl_->RecoverAt(component, now);
}

RuntimeHealthSnapshot RuntimeSupervisor::Snapshot() const noexcept {
  return impl_->Snapshot();
}

std::optional<RuntimeRunResult> RuntimeSupervisor::TerminalResult() const noexcept {
  return impl_->TerminalResult();
}

}  // namespace mv::runtime
