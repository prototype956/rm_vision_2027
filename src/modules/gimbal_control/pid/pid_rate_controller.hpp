#pragma once

#include "hal/gimbal/gimbal_types.hpp"
#include "modules/gimbal_control/gimbal_rate_limits.hpp"
#include "modules/gimbal_control/pid/variable_integral_rate_pid.hpp"

#include <yaml-cpp/yaml.h>

namespace mv::modules {

/** @brief 两轴独立 PID 参数，角度使用 rad，时间使用秒。 */
struct PidRateControllerConfig {
  bool operator==(const PidRateControllerConfig&) const = default;
  VariableIntegralRatePidConfig yaw;
  VariableIntegralRatePidConfig pitch;
};

struct PidRateResult {
  hal::GimbalRateCommand command;
  hal::GimbalRatePidTerms yaw;
  hal::GimbalRatePidTerms pitch;
};

[[nodiscard]] PidRateControllerConfig ParsePidRateControllerConfig(const YAML::Node& root);

/** @brief 单控制线程拥有双轴状态，不接触串口或火控使能。 */
class PidRateController final {
 public:
  PidRateController(PidRateControllerConfig config, GimbalRateLimits limits)
      : config_(config), limits_(limits) {}
  void Reset() noexcept;
  /** @brief 参数或限速变化只重置受影响轴的积分及 D 滤波。 */
  void ApplyTuning(PidRateControllerConfig config, GimbalRateLimits limits) noexcept;
  /** @brief 使用发送阶段的直接反馈计算速率；dt 为实际周期，不外推反馈。 */
  [[nodiscard]] PidRateResult Step(const hal::GimbalCommand& target,
                                   const hal::GimbalActuatorTelemetry& actual, double dt) noexcept;

 private:
  PidRateControllerConfig config_;
  GimbalRateLimits limits_;
  VariableIntegralRatePid yaw_;
  VariableIntegralRatePid pitch_;
};

}  // namespace mv::modules
