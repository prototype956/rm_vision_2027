#pragma once

#include "hal/gimbal/gimbal_types.hpp"

namespace mv::hal {
/** @brief 实机双轴速率通道；target 保留目标、时戳及发射元数据。 */
class IGimbalRateCommandSink {
 public:
  virtual ~IGimbalRateCommandSink() = default;
  virtual bool SendRate(const GimbalCommand& target, const GimbalRateCommand& rate) noexcept = 0;
  [[nodiscard]] virtual bool ExternalControlEnabled() const noexcept = 0;
  [[nodiscard]] virtual bool IsHealthy() const noexcept = 0;
  [[nodiscard]] virtual std::uint64_t HeartbeatTimestampNs() const noexcept = 0;
  [[nodiscard]] virtual GimbalActuatorTelemetry ActuatorTelemetry() const noexcept = 0;
};
}  // namespace mv::hal
