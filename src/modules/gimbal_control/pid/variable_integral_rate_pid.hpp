#pragma once

#include "hal/gimbal/gimbal_types.hpp"

#include <algorithm>
#include <cmath>

namespace mv::modules {

/** @brief Angle-error PID producing a rate command. Gains use radian/second units. */
struct VariableIntegralRatePidConfig {
  bool operator==(const VariableIntegralRatePidConfig&) const = default;
  double kp{1.0};
  double ki{0.0};
  double kd{0.0};
  double integral_full_error_rad{0.02};
  double integral_stop_error_rad{0.10};
  double integral_limit_rad_s{0.2};
  double derivative_filter_tau_s{0.03};
  double deadband_rad{0.002};
};

/** @brief Single-threaded PID with measured-rate D, variable integration and anti-windup. */
class VariableIntegralRatePid {
 public:
  void Reset() noexcept {
    integral_ = 0.0;
    filtered_rate_ = 0.0;
    filter_ready_ = false;
  }

  /**
   * @param error Target minus actual angle; caller wraps yaw to [-pi, pi].
   * @param measured_rate Measured angular rate; D does not differentiate target steps.
   * @param dt Actual elapsed seconds; zero starts a fresh controller without integration.
   * @return Terms and bounded rate command. Deadband clears I and outputs zero.
   */
  [[nodiscard]] hal::GimbalRatePidTerms Step(double error, double measured_rate, double dt,
                                             double max_rate,
                                             const VariableIntegralRatePidConfig& cfg) noexcept {
    hal::GimbalRatePidTerms out;
    if (!std::isfinite(error) || !std::isfinite(measured_rate) || !std::isfinite(dt) ||
        !std::isfinite(max_rate) || max_rate <= 0.0) {
      Reset();
      return out;
    }
    if (dt <= 0.0 || dt > 0.1) {
      Reset();
      dt = 0.0;
    }
    if (!filter_ready_) {
      filtered_rate_ = measured_rate;
      filter_ready_ = true;
    } else {
      const double ALPHA =
          cfg.derivative_filter_tau_s == 0.0 ? 1.0 : dt / (cfg.derivative_filter_tau_s + dt);
      filtered_rate_ += ALPHA * (measured_rate - filtered_rate_);
    }
    out.error_rad = error;
    const double ABS_ERROR = std::abs(error);
    if (ABS_ERROR <= cfg.deadband_rad) {
      integral_ = 0.0;
      return out;
    }
    out.p = cfg.kp * error;
    out.d = -cfg.kd * filtered_rate_;
    out.integral_weight =
        std::clamp((cfg.integral_stop_error_rad - ABS_ERROR) /
                       (cfg.integral_stop_error_rad - cfg.integral_full_error_rad),
                   0.0, 1.0);
    // Separate large-error/target-change motion from fine integral correction.
    if (out.integral_weight == 0.0 || cfg.ki == 0.0)
      integral_ = 0.0;
    const double CANDIDATE = std::clamp(integral_ + cfg.ki * out.integral_weight * error * dt,
                                        -cfg.integral_limit_rad_s, cfg.integral_limit_rad_s);
    const double REQUEST = out.p + CANDIDATE + out.d;
    // Freeze integration that pushes farther into output saturation; allow unwinding.
    const bool PUSHES_INTO_SATURATION =
        (REQUEST > max_rate && error > 0.0) || (REQUEST < -max_rate && error < 0.0);
    if (!PUSHES_INTO_SATURATION)
      integral_ = CANDIDATE;
    out.i = integral_;
    const double RAW = out.p + out.i + out.d;
    if (!std::isfinite(RAW)) {
      Reset();
      return {};
    }
    out.rate_command = std::clamp(RAW, -max_rate, max_rate);
    return out;
  }

 private:
  double integral_{0.0};
  double filtered_rate_{0.0};
  bool filter_ready_{false};
};

}  // namespace mv::modules
