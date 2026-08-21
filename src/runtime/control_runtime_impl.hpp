#pragma once

#include "hal/gimbal/i_gimbal_command_sink.hpp"
#include "modules/fire_control/fire_control.hpp"
#include "modules/fire_control/gimbal_feedback_estimator.hpp"
#include "runtime/control_runtime.hpp"

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
                     tool::foxglove::VisionDebugPublisher* diagnostics);
  ~ControlRuntimeImpl();

  void Start();
  void Update(const modules::ArmorPredictionResult& prediction,
              const hal::CameraFrame::FrameGeometry& geometry);
  void Stop() noexcept;
  [[nodiscard]] bool Failed() const noexcept;

 private:
  struct LoopState {
    std::uint64_t observed_sequence = ~std::uint64_t{0};
    std::optional<bool> last_external_control;
    std::optional<bool> last_sink_healthy;
    std::optional<hal::GimbalActuatorMode> last_actuator_mode;
    std::uint64_t control_cycles{0};
    modules::MatchedGimbalCommand matched_command;
  };

  struct CycleTiming {
    double period_s{0.0};
    double deadline_lateness_us{0.0};
  };

  [[nodiscard]] modules::MatchedGimbalCommand MatchCommand(
      const std::optional<std::uint64_t>& capture_timestamp_ns,
      const std::optional<hal::GimbalActuatorTelemetry>& actuator) const noexcept;
  void RememberCommand(const hal::GimbalCommand& command);
  void ClearPublishedProjection(std::string_view reason) noexcept;
  void AttachProjectionDiagnostics(modules::FireControlResult& result);
  void ProcessSnapshot(const std::shared_ptr<const modules::ControlInputSnapshot>& snapshot,
                       LoopState& state, std::chrono::steady_clock::time_point now,
                       const CycleTiming& timing);
  void Loop() noexcept;
  void SendStop() noexcept;

  const std::chrono::steady_clock::duration PERIOD;
  const double PLANNER_DT_S;
  modules::FireControl fire_control_;
  modules::GimbalFeedbackEstimator feedback_estimator_;
  std::unique_ptr<hal::IGimbalCommandSink> sink_;
  tool::foxglove::VisionDebugPublisher* diagnostics_{nullptr};
  std::shared_ptr<const modules::ControlInputSnapshot> latest_snapshot_;
  std::deque<hal::GimbalCommand> sent_commands_;
  std::vector<modules::PlannedGimbalPoint> last_successful_trajectory_;
  std::size_t last_successful_command_index_{1};
  double last_successful_target_distance_m_{-1.0};
  std::chrono::steady_clock::time_point last_successful_plan_time_{};
  int last_successful_plan_slot_{-1};
  int consecutive_mpc_failure_cycles_{0};
  bool control_projection_active_{false};
  bool output_projection_cleared_pending_{false};
  std::string output_projection_clear_reason_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> failed_{false};
};

}  // namespace mv::runtime
