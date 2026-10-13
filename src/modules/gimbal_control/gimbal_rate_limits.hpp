#pragma once

namespace mv::modules {

/** @brief 实机输出限速，单位 rad/s；与规划约束及反馈限速分别配置。 */
struct GimbalRateLimits {
  bool operator==(const GimbalRateLimits&) const = default;
  double max_yaw_rate_rad_s{3.0};
  double max_pitch_rate_rad_s{4.0};
};

}  // namespace mv::modules
