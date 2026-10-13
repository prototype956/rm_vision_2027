#pragma once

#include "modules/gimbal_control/mpc/mpc_rate_follower.hpp"
#include "modules/gimbal_control/mpc/mpc_trajectory_planner_config.hpp"
#include "modules/gimbal_control/pid/pid_rate_controller.hpp"

#include <string_view>

#include <filesystem>
#include <optional>

namespace mv::modules {

/** @brief 启动时选择的双轴算法；运行期间不自动切换或回退。 */
enum class GimbalControlMode { PID, MPC };
[[nodiscard]] std::string_view GimbalControlModeName(GimbalControlMode mode) noexcept;

/** @brief 公共参考时域和独立的反馈/输出限速；仅加载选中算法的配置。 */
struct GimbalControlConfig {
  bool operator==(const GimbalControlConfig&) const = default;
  GimbalControlMode mode{GimbalControlMode::MPC};
  GimbalControlMode mindvision_mode{GimbalControlMode::PID};
  GimbalControlMode talos_mode{GimbalControlMode::MPC};
  double dt_s{0.01};
  int horizon_steps{50};
  double feedback_max_yaw_velocity_rad_s{3.0};
  double feedback_max_pitch_velocity_rad_s{3.0};
  GimbalRateLimits rate_limits;
  std::optional<PidRateControllerConfig> pid;
  std::optional<MpcTrajectoryPlannerConfig> mpc{MpcTrajectoryPlannerConfig{}};
  MpcRateFollowerConfig follower;
};

/** @brief 解析公共配置及所选算法快照；拒绝未支持的后端和历史实验模式。 */
[[nodiscard]] GimbalControlConfig ParseGimbalControlConfig(const YAML::Node& root,
                                                           std::string_view backend,
                                                           const YAML::Node& algorithm);
/** @brief 只打开当前后端所选的 PID 或 MPC 文件，不要求另一算法配置存在。 */
[[nodiscard]] GimbalControlConfig LoadGimbalControlConfig(const std::filesystem::path& config_root,
                                                          std::string_view backend);

}  // namespace mv::modules
