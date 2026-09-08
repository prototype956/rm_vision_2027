#pragma once

#include "hal/gimbal/i_gimbal_command_sink.hpp"
#include "modules/fire_control/control_session.hpp"
#include "modules/fire_control/gimbal_feedback_estimator.hpp"
#include "runtime/control_runtime.hpp"
#include "runtime/runtime_supervisor.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <optional>

namespace mv::runtime {

/** @brief ControlRuntime 的私有固定周期状态与实现。 */
class ControlRuntimeImpl final {
 public:
  ControlRuntimeImpl(modules::FireControlConfig fire_config,
                     modules::GimbalTrajectoryPlannerConfig planner_config,
                     std::unique_ptr<hal::IGimbalCommandSink> sink,
                     IRuntimeDiagnosticsSink* diagnostics, RuntimeSupervisor& supervisor);
  ~ControlRuntimeImpl();

  void Start();
  void Update(const modules::ArmorPredictionOutput& prediction,
              const frame::FrameKinematics& kinematics,
              const std::optional<frame::ChassisMotionObservation>& chassis_motion,
              const std::optional<hal::GimbalActuatorTelemetry>& gimbal_actuator,
              const modules::RefereeObservation& referee = {});
  void Stop() noexcept;

 private:
  struct LoopState {
    std::uint64_t control_cycles{0};
  };

  struct CycleTiming {
    double period_s{0.0};
    double deadline_lateness_us{0.0};
  };

  void ProcessSnapshot(const std::shared_ptr<const modules::ControlInputSnapshot>& snapshot,
                       LoopState& state, std::chrono::steady_clock::time_point now,
                       const CycleTiming& timing);
  [[nodiscard]] RuntimeDecision ObserveCommandChannel(
      bool sink_healthy, bool send_succeeded, std::chrono::steady_clock::time_point now) noexcept;
  void Loop() noexcept;
  bool SendStop() noexcept;

  const std::chrono::steady_clock::duration PERIOD;
  modules::ControlSession session_;
  std::unique_ptr<hal::IGimbalCommandSink> sink_;
  IRuntimeDiagnosticsSink* diagnostics_{nullptr};
  RuntimeSupervisor& supervisor_;
  std::shared_ptr<const modules::ControlInputSnapshot> latest_snapshot_;
  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace mv::runtime
