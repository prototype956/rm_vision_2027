#pragma once

#include "hal/gimbal/i_gimbal_command_sink.hpp"
#include "hal/gimbal/i_gimbal_rate_command_sink.hpp"
#include "runtime/gimbal_tuning_reload.hpp"

#include <chrono>
#include <memory>
#include <mutex>

#include <optional>

namespace mv::runtime {

/** @brief 保持角度命令接口，在实机发送阶段以直接反馈计算速率。 */
class SerialGimbalControlAdapter final : public hal::IGimbalCommandSink {
 public:
  SerialGimbalControlAdapter(std::unique_ptr<hal::IGimbalRateCommandSink> channel,
                             GimbalTuningSnapshot config, std::filesystem::path config_root = {});
  bool Send(const hal::GimbalCommand& command) noexcept override;
  [[nodiscard]] bool ExternalControlEnabled() const noexcept override;
  [[nodiscard]] bool IsHealthy() const noexcept override;
  [[nodiscard]] std::uint64_t HeartbeatTimestampNs() const noexcept override;
  [[nodiscard]] hal::GimbalActuatorTelemetry ActuatorTelemetry() const noexcept override;

 private:
  void ResetPid() noexcept;
  std::unique_ptr<hal::IGimbalRateCommandSink> channel_;
  modules::GimbalControlConfig config_;
  std::optional<modules::PidRateController> pid_;
  std::optional<std::chrono::steady_clock::time_point> last_pid_time_;
  mutable std::mutex mutex_;
  bool pid_telemetry_valid_{false};
  hal::GimbalRatePidTerms yaw_terms_;
  hal::GimbalRatePidTerms pitch_terms_;
  std::unique_ptr<GimbalTuningReload> reload_;  ///< 先停止后台读取再释放控制状态。
};

}  // namespace mv::runtime
