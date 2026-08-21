#include "runtime/control_runtime.hpp"

#include "runtime/control_runtime_impl.hpp"

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
                                       tool::foxglove::VisionDebugPublisher* diagnostics)
    : PERIOD(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(planner_config.dt_s))),
      PLANNER_DT_S(planner_config.dt_s),
      fire_control_(fire_config, planner_config),
      feedback_estimator_(planner_config.max_yaw_velocity_rad_s,
                          planner_config.max_pitch_velocity_rad_s),
      sink_(std::move(sink)),
      diagnostics_(diagnostics) {
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
    throw;
  }
}

void ControlRuntimeImpl::Update(
    const modules::ArmorPredictionResult& prediction, const frame::FrameKinematics& kinematics,
    const std::optional<hal::GimbalActuatorTelemetry>& gimbal_actuator) {
  auto snapshot = std::make_shared<modules::ControlInputSnapshot>();
  snapshot->prediction = prediction;
  // 仿真真值不进入控制快照，只保留同帧云台与枪口外参。
  snapshot->world_t_gimbal = kinematics.world_t_gimbal;
  snapshot->gimbal_t_muzzle = kinematics.gimbal_t_muzzle;
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

bool ControlRuntimeImpl::Failed() const noexcept {
  return failed_.load(std::memory_order_acquire);
}

modules::MatchedGimbalCommand ControlRuntimeImpl::MatchCommand(
    const std::optional<std::uint64_t>& capture_timestamp_ns,
    const std::optional<hal::GimbalActuatorTelemetry>& actuator) const noexcept {
  modules::MatchedGimbalCommand match;
  if (!capture_timestamp_ns)
    return match;
  if (actuator && actuator->valid && actuator->consumed_command_timestamp_ns != 0) {
    for (auto iterator = sent_commands_.rbegin(); iterator != sent_commands_.rend(); ++iterator) {
      if (iterator->timestamp_ns == actuator->consumed_command_timestamp_ns) {
        match.valid = iterator->valid;
        match.approximate = false;
        match.command = *iterator;
        match.age_at_capture_s =
            *capture_timestamp_ns >= iterator->timestamp_ns
                ? static_cast<double>(*capture_timestamp_ns - iterator->timestamp_ns) * 1.0e-9
                : 0.0;
        return match;
      }
    }
  }
  for (auto iterator = sent_commands_.rbegin(); iterator != sent_commands_.rend(); ++iterator) {
    if (iterator->timestamp_ns <= *capture_timestamp_ns) {
      match.valid = iterator->valid;
      match.approximate = true;
      match.command = *iterator;
      match.age_at_capture_s =
          static_cast<double>(*capture_timestamp_ns - iterator->timestamp_ns) * 1.0e-9;
      return match;
    }
  }
  return match;
}

void ControlRuntimeImpl::RememberCommand(const hal::GimbalCommand& command) {
  sent_commands_.push_back(command);
  while (!sent_commands_.empty() &&
         command.timestamp_ns > sent_commands_.front().timestamp_ns + 1'000'000'000ULL) {
    sent_commands_.pop_front();
  }
}

void ControlRuntimeImpl::ClearPublishedProjection(std::string_view reason) noexcept {
  feedback_estimator_.ClearCommandProjection();
  fire_control_.ResetFireReadiness();
  last_successful_trajectory_.clear();
  control_projection_active_ = false;
  output_projection_cleared_pending_ = true;
  if (!output_projection_clear_reason_.empty())
    output_projection_clear_reason_.push_back(',');
  output_projection_clear_reason_.append(reason);
}

void ControlRuntimeImpl::AttachProjectionDiagnostics(modules::FireControlResult& result) {
  result.output_projection_cleared = output_projection_cleared_pending_;
  result.output_projection_clear_reason = std::move(output_projection_clear_reason_);
  output_projection_cleared_pending_ = false;
  output_projection_clear_reason_.clear();
}

void ControlRuntimeImpl::SendStop() noexcept {
  if (!sink_)
    return;
  hal::GimbalCommand stop;
  stop.timestamp_ns = SystemNowNs();
  sink_->Send(stop);
  feedback_estimator_.ClearCommandProjection();
  last_successful_trajectory_.clear();
  control_projection_active_ = false;
}

ControlRuntime::ControlRuntime(modules::FireControlConfig fire_config,
                               modules::GimbalTrajectoryPlannerConfig planner_config,
                               std::unique_ptr<hal::IGimbalCommandSink> sink,
                               tool::foxglove::VisionDebugPublisher* diagnostics)
    : impl_(std::make_unique<ControlRuntimeImpl>(fire_config, planner_config, std::move(sink),
                                                 diagnostics)) {}

ControlRuntime::~ControlRuntime() = default;

void ControlRuntime::Start() {
  impl_->Start();
}

void ControlRuntime::Update(const modules::ArmorPredictionResult& prediction,
                            const frame::FrameKinematics& kinematics,
                            const std::optional<hal::GimbalActuatorTelemetry>& gimbal_actuator) {
  impl_->Update(prediction, kinematics, gimbal_actuator);
}

void ControlRuntime::Stop() noexcept {
  impl_->Stop();
}

bool ControlRuntime::Failed() const noexcept {
  return impl_->Failed();
}

}  // namespace mv::runtime
