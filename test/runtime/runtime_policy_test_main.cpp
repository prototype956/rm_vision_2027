#include "hal/gimbal/i_gimbal_command_sink.hpp"
#include "runtime/control_runtime.hpp"
#include "runtime/runtime_supervisor.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string_view>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class FakeCommandSink final : public mv::hal::IGimbalCommandSink {
 public:
  bool Send(const mv::hal::GimbalCommand& command) noexcept override {
    ++send_count;
    if (!command.valid)
      ++invalid_stop_count;
    return send_succeeds.load();
  }

  [[nodiscard]] bool ExternalControlEnabled() const noexcept override { return true; }
  [[nodiscard]] bool IsHealthy() const noexcept override { return healthy.load(); }
  [[nodiscard]] std::uint64_t HeartbeatTimestampNs() const noexcept override { return 0; }
  [[nodiscard]] mv::hal::GimbalActuatorTelemetry ActuatorTelemetry() const noexcept override {
    return {};
  }

  std::atomic<bool> healthy{true};
  std::atomic<bool> send_succeeds{true};
  std::atomic<int> send_count{0};
  std::atomic<int> invalid_stop_count{0};
};

bool Check(bool condition, std::string_view message) {
  if (!condition)
    std::cerr << "FAILED: " << message << '\n';
  return condition;
}

bool TestPolicyParsing() {
  const auto POLICY = mv::runtime::ParseRuntimeErrorPolicy(YAML::Load(R"(
schema_version: 1
camera:
  no_valid_frame_timeout_ms: 2000
control:
  command_sink_unhealthy_timeout_ms: 1000
optional_components:
  evaluation_disable_after_consecutive_errors: 1
  debug_window_disable_after_consecutive_errors: 1
)"));
  bool valid = Check(
      POLICY.camera_no_valid_frame_timeout == 2s && POLICY.command_sink_unhealthy_timeout == 1s,
      "runtime policy parser must preserve configured timeouts");
  valid &= Check(!POLICY.command_sink_wait_for_recovery,
                 "old configs retain terminal command timeout behavior");
  auto reconnect_config = YAML::Load(R"(
schema_version: 1
camera:
  no_valid_frame_timeout_ms: 2000
control:
  command_sink_unhealthy_timeout_ms: 1000
  command_sink_wait_for_recovery: true
optional_components:
  evaluation_disable_after_consecutive_errors: 1
  debug_window_disable_after_consecutive_errors: 1
)");
  valid &= Check(mv::runtime::ParseRuntimeErrorPolicy(reconnect_config)
                     .command_sink_wait_for_recovery,
                 "reconnect policy must parse explicitly");
  reconnect_config["control"]["command_sink_wait_for_recovery"] = "invalid";
  try {
    static_cast<void>(mv::runtime::ParseRuntimeErrorPolicy(reconnect_config));
    valid &= Check(false, "reconnect policy must reject invalid boolean values");
  } catch (const std::exception&) {
  }
  try {
    static_cast<void>(mv::runtime::ParseRuntimeErrorPolicy(YAML::Load(R"(
schema_version: 1
camera:
  no_valid_frame_timeout_ms: 0
control:
  command_sink_unhealthy_timeout_ms: 1000
optional_components:
  evaluation_disable_after_consecutive_errors: 1
  debug_window_disable_after_consecutive_errors: 1
)")));
    valid &= Check(false, "runtime policy parser must reject non-positive thresholds");
  } catch (const std::exception&) {
  }
  return valid;
}

bool TestCameraTimeoutAndRecovery() {
  mv::runtime::RuntimeErrorPolicy policy;
  mv::runtime::RuntimeSupervisor supervisor(policy);
  const auto START = Clock::time_point(10s);
  supervisor.RecoverAt(mv::runtime::RuntimeComponent::CAMERA, START);

  const auto TIMEOUT =
      supervisor.ReportAt(mv::runtime::RuntimeFaultCode::CAMERA_TIMEOUT, "timeout", START + 200ms);
  const auto MIXED = supervisor.ReportAt(mv::runtime::RuntimeFaultCode::CAMERA_INVALID_FRAME,
                                         "invalid", START + 1999ms);
  bool valid = Check(TIMEOUT.skip_current_work && !TIMEOUT.termination,
                     "camera timeout must skip without early termination") &&
               Check(MIXED.skip_current_work && !MIXED.termination,
                     "mixed invalid frames must share the pre-threshold timer");

  supervisor.RecoverAt(mv::runtime::RuntimeComponent::CAMERA, START + 2s);
  valid &= Check(supervisor.Snapshot().overall == mv::runtime::RuntimeHealthState::HEALTHY,
                 "a valid frame must clear the transient camera fault");
  const auto AGAIN =
      supervisor.ReportAt(mv::runtime::RuntimeFaultCode::CAMERA_TIMEOUT, "timeout", START + 2100ms);
  const auto TERMINAL = supervisor.ReportAt(mv::runtime::RuntimeFaultCode::CAMERA_INVALID_FRAME,
                                            "invalid", START + 4s);
  valid &= Check(!AGAIN.termination, "camera timer must restart after recovery") &&
           Check(TERMINAL.termination == mv::runtime::RuntimeTerminationReason::CAMERA_FAILURE,
                 "camera must terminate at the two-second threshold");
  return valid;
}

bool TestImmediateAndLatchedFailures() {
  mv::runtime::RuntimeSupervisor disconnected({});
  const auto CAMERA = disconnected.Report(mv::runtime::RuntimeFaultCode::CAMERA_DISCONNECTED);
  bool valid = Check(CAMERA.request_safe_stop && CAMERA.termination,
                     "camera disconnect must stop and terminate immediately");
  static_cast<void>(disconnected.Report(mv::runtime::RuntimeFaultCode::CONTROL_THREAD_EXCEPTION));
  const auto LATCHED = disconnected.TerminalResult();
  valid &= Check(LATCHED && LATCHED->fault &&
                     LATCHED->fault->code == mv::runtime::RuntimeFaultCode::CAMERA_DISCONNECTED,
                 "the first terminal fault must remain latched");
  disconnected.Recover(mv::runtime::RuntimeComponent::CAMERA);
  valid &=
      Check(disconnected.TerminalResult().has_value(), "recovery must not clear a terminal fault");

  mv::runtime::RuntimeSupervisor fatal({});
  valid &= Check(fatal.Report(mv::runtime::RuntimeFaultCode::CAMERA_FATAL).termination.has_value(),
                 "camera fatal must terminate immediately");

  mv::runtime::RuntimeSupervisor pipeline({});
  const auto PIPELINE = pipeline.Report(mv::runtime::RuntimeFaultCode::VISION_PIPELINE_EXCEPTION);
  valid &=
      Check(PIPELINE.request_safe_stop &&
                PIPELINE.termination == mv::runtime::RuntimeTerminationReason::PIPELINE_FAILURE,
            "formal pipeline exceptions must stop and terminate immediately");

  mv::runtime::RuntimeSupervisor control({});
  const auto CONTROL = control.Report(mv::runtime::RuntimeFaultCode::CONTROL_UPDATE_EXCEPTION);
  valid &= Check(CONTROL.request_safe_stop &&
                     CONTROL.termination == mv::runtime::RuntimeTerminationReason::CONTROL_FAILURE,
                 "control snapshot exceptions must stop and terminate immediately");
  return valid;
}

bool TestOptionalComponentsAndExitCodes() {
  mv::runtime::RuntimeSupervisor supervisor({});
  const auto EVALUATION = supervisor.Report(mv::runtime::RuntimeFaultCode::EVALUATION_EXCEPTION);
  const auto EVALUATION_AGAIN =
      supervisor.Report(mv::runtime::RuntimeFaultCode::EVALUATION_EXCEPTION);
  const auto WINDOW = supervisor.Report(mv::runtime::RuntimeFaultCode::DEBUG_WINDOW_EXCEPTION);
  const auto WINDOW_AGAIN =
      supervisor.Report(mv::runtime::RuntimeFaultCode::DEBUG_WINDOW_EXCEPTION);
  const auto DIAGNOSTICS =
      supervisor.Report(mv::runtime::RuntimeFaultCode::DIAGNOSTICS_UNAVAILABLE);
  bool valid =
      Check(EVALUATION.disable_component && !EVALUATION.termination,
            "evaluation failure must only disable evaluation") &&
      Check(!EVALUATION_AGAIN.disable_component,
            "evaluation disable decision must only be emitted once") &&
      Check(WINDOW.disable_component && !WINDOW.termination,
            "window failure must only disable the window") &&
      Check(!WINDOW_AGAIN.disable_component, "window disable decision must only be emitted once") &&
      Check(!DIAGNOSTICS.termination,
            "diagnostics unavailability must never terminate the formal chain");
  valid &= Check(
      mv::runtime::RuntimeExitCode(mv::runtime::RuntimeTerminationReason::NORMAL) == 0 &&
          mv::runtime::RuntimeExitCode(mv::runtime::RuntimeTerminationReason::CAMERA_FAILURE) ==
              4 &&
          mv::runtime::RuntimeExitCode(mv::runtime::RuntimeTerminationReason::PIPELINE_FAILURE) ==
              5 &&
          mv::runtime::RuntimeExitCode(mv::runtime::RuntimeTerminationReason::CONTROL_FAILURE) == 7,
      "runtime termination reasons must preserve existing exit codes");
  return valid;
}

bool TestCommandRecovery() {
  mv::runtime::RuntimeErrorPolicy policy;
  policy.command_sink_unhealthy_timeout = 1s;
  mv::runtime::RuntimeSupervisor supervisor(policy);
  const auto START = Clock::time_point(20s);
  supervisor.RecoverAt(mv::runtime::RuntimeComponent::COMMAND_SINK, START);
  const auto FIRST =
      supervisor.ReportAt(mv::runtime::RuntimeFaultCode::COMMAND_SEND_FAILED, "send", START + 10ms);
  const auto BEFORE = supervisor.ReportAt(mv::runtime::RuntimeFaultCode::COMMAND_SINK_UNHEALTHY,
                                          "health", START + 999ms);
  bool valid = Check(FIRST.request_safe_stop && !FIRST.termination,
                     "first command failure must request an immediate safe stop") &&
               Check(!BEFORE.termination, "command channel may recover before one second");
  supervisor.RecoverAt(mv::runtime::RuntimeComponent::COMMAND_SINK, START + 999ms);
  valid &= Check(supervisor.Snapshot().overall == mv::runtime::RuntimeHealthState::HEALTHY,
                 "healthy check and send recovery must clear command degradation");

  static_cast<void>(supervisor.ReportAt(mv::runtime::RuntimeFaultCode::COMMAND_SEND_FAILED, "send",
                                        START + 1100ms));
  const auto TERMINAL = supervisor.ReportAt(mv::runtime::RuntimeFaultCode::COMMAND_SINK_UNHEALTHY,
                                            "health", START + 2099ms);
  valid &= Check(TERMINAL.termination == mv::runtime::RuntimeTerminationReason::CONTROL_FAILURE,
                 "persistent command failure must terminate after one second");
  return valid;
}

bool TestControlRuntimeSendsStop() {
  mv::runtime::RuntimeErrorPolicy policy;
  policy.command_sink_unhealthy_timeout = 80ms;
  mv::runtime::RuntimeSupervisor supervisor(policy);
  auto fake = std::make_unique<FakeCommandSink>();
  auto* observed = fake.get();
  mv::runtime::ControlRuntime control({}, {}, std::move(fake), nullptr, supervisor);
  control.Start();

  std::this_thread::sleep_for(20ms);
  observed->send_succeeds.store(false);
  std::this_thread::sleep_for(30ms);
  observed->send_succeeds.store(true);
  std::this_thread::sleep_for(100ms);
  bool valid = Check(!supervisor.TerminalResult(),
                     "a fake command send failure must recover before the timeout");

  observed->healthy.store(false);
  const auto DEADLINE = Clock::now() + 500ms;
  while (!supervisor.TerminalResult() && Clock::now() < DEADLINE)
    std::this_thread::sleep_for(2ms);
  control.Stop();
  valid &= Check(supervisor.TerminalResult().has_value(),
                 "unhealthy fake command sink must latch a control failure") &&
           Check(observed->invalid_stop_count.load() > 0,
                 "the fake command sink must receive an invalid stop command") &&
           Check(observed->send_count.load() > 0,
                 "the fake command sink must receive control-loop publications");
  return valid;
}

bool TestPersistentCommandRecovery() {
  mv::runtime::RuntimeErrorPolicy policy;
  policy.command_sink_wait_for_recovery = true;
  mv::runtime::RuntimeSupervisor supervisor(policy);
  const auto START = Clock::time_point(20s);
  supervisor.RecoverAt(mv::runtime::RuntimeComponent::COMMAND_SINK, START);
  bool valid = true;
  for (const auto DELAY : {10ms, 1000ms, 3000ms, 3600000ms}) {
    const auto DECISION = supervisor.ReportAt(
        mv::runtime::RuntimeFaultCode::COMMAND_SINK_UNHEALTHY, "stale feedback", START + DELAY);
    valid &= Check(DECISION.request_safe_stop && !DECISION.termination,
                   "persistent outage must always stop without terminating in recovery mode");
  }
  const auto SEND_FAILURE = supervisor.ReportAt(
      mv::runtime::RuntimeFaultCode::COMMAND_SEND_FAILED, "disconnected", START + 3600001ms);
  valid &= Check(SEND_FAILURE.request_safe_stop && !SEND_FAILURE.termination,
                 "send failure must share recovery policy");
  supervisor.RecoverAt(mv::runtime::RuntimeComponent::COMMAND_SINK, START + 3600002ms);
  valid &= Check(!supervisor.TerminalResult() &&
                     supervisor.Snapshot().overall == mv::runtime::RuntimeHealthState::HEALTHY,
                 "late reconnection must clear degradation without restarting the process");
  valid &= Check(supervisor.Report(mv::runtime::RuntimeFaultCode::CONTROL_THREAD_EXCEPTION)
                     .termination.has_value(),
                 "reconnect policy must not suppress control thread exceptions");
  return valid;
}

bool TestControlRuntimeWaitsForRecovery() {
  mv::runtime::RuntimeErrorPolicy policy;
  policy.command_sink_unhealthy_timeout = 30ms;
  policy.command_sink_wait_for_recovery = true;
  mv::runtime::RuntimeSupervisor supervisor(policy);
  auto fake = std::make_unique<FakeCommandSink>();
  auto* observed = fake.get();
  observed->healthy.store(false);
  observed->send_succeeds.store(false);
  mv::runtime::ControlRuntime control({}, {}, std::move(fake), nullptr, supervisor);
  control.Start();
  std::this_thread::sleep_for(150ms);
  const int STOPS = observed->invalid_stop_count.load();
  bool valid = Check(!supervisor.TerminalResult() && STOPS > 0,
                     "control loop must keep sending stop beyond the terminal timeout");
  observed->healthy.store(true);
  observed->send_succeeds.store(true);
  const auto DEADLINE = Clock::now() + 500ms;
  while (supervisor.Snapshot().overall != mv::runtime::RuntimeHealthState::HEALTHY &&
         Clock::now() < DEADLINE)
    std::this_thread::sleep_for(2ms);
  valid &= Check(supervisor.Snapshot().overall == mv::runtime::RuntimeHealthState::HEALTHY &&
                     observed->send_count.load() > STOPS,
                 "same control loop must recover and continue publishing after outage");
  control.Stop();
  return valid;
}

}  // namespace

int main() {
  const bool PASSED = TestPolicyParsing() && TestCameraTimeoutAndRecovery() &&
                      TestImmediateAndLatchedFailures() && TestOptionalComponentsAndExitCodes() &&
                      TestCommandRecovery() && TestControlRuntimeSendsStop() &&
                      TestPersistentCommandRecovery() && TestControlRuntimeWaitsForRecovery();
  if (!PASSED)
    return EXIT_FAILURE;
  std::cout << "runtime policy acceptance passed\n";
  return EXIT_SUCCESS;
}
