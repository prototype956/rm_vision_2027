#include "runtime/control_runtime.hpp"

#include "runtime/control_runtime_impl.hpp"
#include "runtime/runtime_diagnostics_sink.hpp"
#include "runtime/runtime_supervisor.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>

namespace mv::runtime {
namespace {

std::uint64_t SystemNowNs() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}

}  // namespace

ControlRuntimeImpl::ControlRuntimeImpl(modules::FireControlConfig fire_config,
                                       modules::GimbalTrajectoryPlannerConfig planner_config,
                                       std::unique_ptr<hal::IGimbalCommandSink> sink,
                                       IRuntimeDiagnosticsSink* diagnostics,
                                       RuntimeSupervisor& supervisor)
    : PERIOD(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(planner_config.dt_s))),
      session_(fire_config, planner_config),
      sink_(std::move(sink)),
      diagnostics_(diagnostics),
      supervisor_(supervisor) {
  if (!sink_)
    throw std::invalid_argument("control runtime requires a command sink");
}

ControlRuntimeImpl::~ControlRuntimeImpl() {
  Stop();
}

void ControlRuntimeImpl::Start() {
  if (running_.exchange(true, std::memory_order_acq_rel))
    return;
  try {
    thread_ = std::thread([this] { Loop(); });
  } catch (...) {
    running_.store(false, std::memory_order_release);
    SendStop();
    static_cast<void>(supervisor_.Report(RuntimeFaultCode::CONTROL_THREAD_EXCEPTION,
                                         "failed to start 100 Hz control thread"));
    throw;
  }
}

void ControlRuntimeImpl::Update(
    const modules::ArmorPredictionOutput& prediction, const frame::FrameKinematics& kinematics,
    const std::optional<frame::ChassisMotionObservation>& chassis_motion,
    const std::optional<hal::GimbalActuatorTelemetry>& gimbal_actuator,
    const modules::RefereeObservation& referee) {
  auto snapshot = std::make_shared<modules::ControlInputSnapshot>();
  snapshot->prediction = prediction;
  snapshot->referee = referee;
  // 仿真真值不进入控制快照，只保留同帧云台与枪口外参。
  snapshot->world_t_gimbal = kinematics.world_t_gimbal;
  snapshot->gimbal_t_camera_optical = kinematics.gimbal_t_camera_optical;
  snapshot->gimbal_t_muzzle = kinematics.gimbal_t_muzzle;
  snapshot->chassis_motion = chassis_motion;
  snapshot->frame_actuator = gimbal_actuator;
  std::atomic_store_explicit(&latest_snapshot_,
                             std::shared_ptr<const modules::ControlInputSnapshot>(snapshot),
                             std::memory_order_release);
}

void ControlRuntimeImpl::Stop() noexcept {
  running_.store(false, std::memory_order_release);
  if (thread_.joinable())
    thread_.join();
  SendStop();
}

bool ControlRuntimeImpl::SendStop() noexcept {
  if (!sink_)
    return false;
  hal::GimbalCommand stop;
  stop.timestamp_ns = SystemNowNs();
  const bool SENT = sink_->Send(stop);
  session_.ClearPublishedProjection("runtime_stop");
  return SENT;
}

ControlRuntime::ControlRuntime(modules::FireControlConfig fire_config,
                               modules::GimbalTrajectoryPlannerConfig planner_config,
                               std::unique_ptr<hal::IGimbalCommandSink> sink,
                               IRuntimeDiagnosticsSink* diagnostics, RuntimeSupervisor& supervisor)
    : impl_(std::make_unique<ControlRuntimeImpl>(fire_config, planner_config, std::move(sink),
                                                 diagnostics, supervisor)) {}

ControlRuntime::~ControlRuntime() = default;

void ControlRuntime::Start() {
  impl_->Start();
}

void ControlRuntime::Update(const modules::ArmorPredictionOutput& prediction,
                            const frame::FrameKinematics& kinematics,
                            const std::optional<frame::ChassisMotionObservation>& chassis_motion,
                            const std::optional<hal::GimbalActuatorTelemetry>& gimbal_actuator,
                            const modules::RefereeObservation& referee) {
  impl_->Update(prediction, kinematics, chassis_motion, gimbal_actuator, referee);
}

void ControlRuntime::Stop() noexcept {
  impl_->Stop();
}

}  // namespace mv::runtime
