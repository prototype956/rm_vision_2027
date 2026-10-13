#pragma once

#include "modules/gimbal_control/gimbal_trajectory_types.hpp"
#include "modules/gimbal_control/mpc/mpc_trajectory_planner_config.hpp"

namespace mv::modules {
/**
 * @brief 使用两个独立双积分 TinyMPC 求解器生成受速度和加速度约束的云台轨迹。
 *
 * 每个实例保存两轴求解器、角度原点和跨周期 warm start，只能按控制周期串行调用。
 */
class MpcTrajectoryPlanner final {
 public:
  /** @brief 按已校验配置创建偏航和俯仰求解器，并请求首次 warm-start 重建。 */
  explicit MpcTrajectoryPlanner(MpcTrajectoryPlannerConfig config);
  ~MpcTrajectoryPlanner();

  MpcTrajectoryPlanner(const MpcTrajectoryPlanner&) = delete;
  MpcTrajectoryPlanner& operator=(const MpcTrajectoryPlanner&) = delete;
  MpcTrajectoryPlanner(MpcTrajectoryPlanner&&) noexcept;
  MpcTrajectoryPlanner& operator=(MpcTrajectoryPlanner&&) noexcept;

  /**
   * @brief 从当前反馈跟踪整段角度/角速度参考，求解下一条前视命令。
   * @param feedback 当前云台角度和角速度；valid=false 时直接返回失败。
   * @param reference 点数必须等于 horizon_steps，时间间隔固定为 config.dt_s。
   * @return 完整轨迹、正式命令和求解诊断；失败时 valid=false。
   */
  [[nodiscard]] GimbalTrajectoryResult Plan(const hal::GimbalFeedback& feedback,
                                            std::span<const AimReferencePoint> reference);
  /** @brief 请求下一周期以当前反馈为原点重建两轴 warm start。 */
  void RequestWarmStartRebase() noexcept;
  /** @brief 返回构造时保存的只读规划器配置。 */
  [[nodiscard]] const MpcTrajectoryPlannerConfig& Config() const noexcept;

 private:
  struct Impl;                  ///< 隔离 TinyMPC、归一化参数和跨周期状态。
  std::unique_ptr<Impl> impl_;  ///< 当前规划器唯一拥有的双轴求解器。
};

}  // namespace mv::modules
