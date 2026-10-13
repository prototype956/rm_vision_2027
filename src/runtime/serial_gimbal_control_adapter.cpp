#include "runtime/serial_gimbal_control_adapter.hpp"

#include "core/logger.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace mv::runtime {
SerialGimbalControlAdapter::SerialGimbalControlAdapter(
    std::unique_ptr<hal::IGimbalRateCommandSink> channel, GimbalTuningSnapshot config,
    std::filesystem::path config_root)
    : channel_(std::move(channel)), config_(config.control) {
  if (!channel_)
    throw std::invalid_argument("serial control requires a rate channel");
  if (config_.mode == modules::GimbalControlMode::PID) {
    if (!config_.pid)
      throw std::invalid_argument("PID mode requires parameters");
    pid_.emplace(*config_.pid, config_.rate_limits);
  }
  if (!config_root.empty())
    reload_ = std::make_unique<GimbalTuningReload>(std::move(config_root), config);
}

void SerialGimbalControlAdapter::ResetPid() noexcept {
  if (pid_)
    pid_->Reset();
  last_pid_time_.reset();
  std::lock_guard lock(mutex_);
  pid_telemetry_valid_ = false;
}

bool SerialGimbalControlAdapter::Send(const hal::GimbalCommand& command) noexcept {
  if (reload_) {
    const auto NEXT = reload_->TakePending();
    if (NEXT) {
      if (pid_ && NEXT->control.pid)
        pid_->ApplyTuning(*NEXT->control.pid, NEXT->control.rate_limits);
      config_.rate_limits = NEXT->control.rate_limits;
      config_.pid = NEXT->control.pid;
      config_.follower = NEXT->control.follower;
      MV_LOG_INFO("GimbalTuning", "applied {} tuning",
                  modules::GimbalControlModeName(config_.mode));
    }
  }
  if (!IsHealthy() || !command.valid) {
    ResetPid();
    return channel_->SendRate(command, {});
  }
  // Sample again at send time, as the former serial backend did. Do not substitute
  // the fire-control estimator's projected/clamped feedback for this direct state.
  const auto ACTUAL = channel_->ActuatorTelemetry();
  if (!ACTUAL.valid || !std::isfinite(command.yaw) || !std::isfinite(command.pitch) ||
      !std::isfinite(command.yaw_velocity) || !std::isfinite(command.pitch_velocity) ||
      !std::isfinite(command.target_distance_m) || command.target_distance_m < 0.0) {
    ResetPid();
    return false;
  }
  hal::GimbalRateCommand rate;
  modules::PidRateResult terms;
  if (pid_) {
    const auto NOW = std::chrono::steady_clock::now();
    const double DT =
        last_pid_time_ ? std::chrono::duration<double>(NOW - *last_pid_time_).count() : 0.0;
    last_pid_time_ = NOW;
    terms = pid_->Step(command, ACTUAL, DT);
    rate = terms.command;
  } else {
    rate = modules::FollowMpcRate(command, ACTUAL, config_.follower, config_.rate_limits);
  }
  const bool SENT = channel_->SendRate(command, rate);
  if (!SENT) {
    ResetPid();
  } else {
    std::lock_guard lock(mutex_);
    pid_telemetry_valid_ = pid_.has_value();
    yaw_terms_ = terms.yaw;
    pitch_terms_ = terms.pitch;
  }
  return SENT;
}

bool SerialGimbalControlAdapter::IsHealthy() const noexcept {
  return channel_->IsHealthy();
}
bool SerialGimbalControlAdapter::ExternalControlEnabled() const noexcept {
  return channel_->ExternalControlEnabled();
}
std::uint64_t SerialGimbalControlAdapter::HeartbeatTimestampNs() const noexcept {
  return channel_->HeartbeatTimestampNs();
}
hal::GimbalActuatorTelemetry SerialGimbalControlAdapter::ActuatorTelemetry() const noexcept {
  auto result = channel_->ActuatorTelemetry();
  std::lock_guard lock(mutex_);
  result.serial_pid_active = result.valid && result.command_valid && pid_telemetry_valid_;
  result.yaw_pid = yaw_terms_;
  result.pitch_pid = pitch_terms_;
  return result;
}
}  // namespace mv::runtime
