#pragma once

#include "modules/gimbal_control/gimbal_control_config.hpp"
#include "modules/gimbal_control/gimbal_trajectory_types.hpp"
#include "modules/gimbal_control/mpc/mpc_trajectory_planner.hpp"

#include <memory>

namespace mv::modules {
/** @brief 按启动模式组织参考；PID 不创建 MPC，MPC 状态仅由其求解器拥有。 */
class GimbalReferencePlanner final {
 public:
  explicit GimbalReferencePlanner(GimbalControlConfig config);
  [[nodiscard]] GimbalTrajectoryResult Plan(const hal::GimbalFeedback& feedback,
                                            std::span<const AimReferencePoint> reference);
  void RequestWarmStartRebase() noexcept;
  [[nodiscard]] const GimbalControlConfig& Config() const noexcept { return config_; }

 private:
  GimbalControlConfig config_;
  std::unique_ptr<MpcTrajectoryPlanner> mpc_;
  std::vector<AimReferencePoint> previous_reference_;
};
}  // namespace mv::modules
