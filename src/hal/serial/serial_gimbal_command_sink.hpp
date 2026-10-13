#pragma once

#include "geometry/gimbal_geometry.hpp"
#include "hal/gimbal/i_gimbal_rate_command_sink.hpp"
#include "hal/imu/serial_imu.hpp"

#include <mutex>

#include <yaml-cpp/yaml.h>

namespace mv::hal {
/** @brief 仅描述实机传输使能和反馈时效；算法参数归 modules。 */
struct SerialGimbalCommandSinkConfig {
  bool operator==(const SerialGimbalCommandSinkConfig&) const = default;
  bool enabled{true};
  bool fire_output_enabled{false};
  double max_feedback_age_ms{100.0};
};
[[nodiscard]] SerialGimbalCommandSinkConfig ParseSerialGimbalCommandSinkConfig(
    const YAML::Node& root);

/** @brief 同一 SerialImu 会话的速率传输与反馈适配，不拥有算法或调参线程。 */
class SerialGimbalCommandSink final : public IGimbalRateCommandSink {
 public:
  SerialGimbalCommandSink(SerialImu& imu, geometry::GimbalExtrinsics extrinsics,
                          SerialGimbalCommandSinkConfig config);
  bool SendRate(const GimbalCommand& target, const GimbalRateCommand& rate) noexcept override;
  [[nodiscard]] bool ExternalControlEnabled() const noexcept override;
  [[nodiscard]] bool IsHealthy() const noexcept override;
  [[nodiscard]] std::uint64_t HeartbeatTimestampNs() const noexcept override;
  [[nodiscard]] GimbalActuatorTelemetry ActuatorTelemetry() const noexcept override;

 private:
  SerialImu& imu_;
  geometry::GimbalExtrinsics extrinsics_;
  SerialGimbalCommandSinkConfig config_;
  mutable std::mutex mutex_;
  GimbalCommand last_command_;
  std::uint64_t consumed_at_timestamp_ns_{0};
  bool command_valid_{false};
};
}  // namespace mv::hal
