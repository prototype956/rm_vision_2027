#pragma once
#include "modules/fire_control/control_types.hpp"
#include "modules/fire_control/rule_policy.hpp"
namespace mv::modules {
/**
 * @brief 根据目标预测和云台反馈完成装甲选择、弹道解算、MPC 规划与开火门控。
 *
 * 实例保存槽位滞回、开火脉冲和求解器 warm-start 状态，应由同一控制序列串行调用。
 */
class FireControl final {
 public:
  /** @brief 保存已校验的火控和轨迹规划配置，并创建空控制状态。 */
  FireControl(FireControlConfig config, GimbalTrajectoryPlannerConfig planner_config);

  /**
   * @brief 运行一个控制周期并生成云台目标、开火建议及完整诊断。
   * @param input 同一预测源帧的目标状态、坐标变换和外部控制使能状态。
   * @param feedback 当前控制时刻采用的融合云台反馈。
   * @param now 本控制周期的本机单调时刻。
   * @return 控制命令和各阶段诊断；控制链失败时命令无效，开火门控失败时仅禁止开火。
   */
  [[nodiscard]] FireControlResult Step(const ControlInputSnapshot& input,
                                       const hal::GimbalFeedback& feedback,
                                       std::chrono::steady_clock::time_point now,
                                       std::uint64_t command_timestamp_ns,
                                       std::optional<PolicyDecision> decision = std::nullopt,
                                       const ControlPolicy& policy = {});
  /** @brief 新回合开始时重置全部状态，包括机械发射间隔。 */
  void Reset();
  /** @brief 仅估算候选观测，不推进控制状态。 */
  [[nodiscard]] PolicyObservation Observe(const ControlInputSnapshot& input,
                                          const hal::GimbalFeedback& feedback,
                                          std::chrono::steady_clock::time_point now) const;
  /** @brief 清除瞄准稳定计数和正在输出的开火脉冲，不改变槽位锁定。 */
  void ResetFireReadiness() noexcept;
  /** @brief 返回构造时保存的只读火控配置。 */
  [[nodiscard]] const FireControlConfig& Config() const noexcept { return config_; }

 private:
  /** @brief 迭代目标运动与弹丸飞行时间，求解指定槽位的低弹道命中角。 */
  [[nodiscard]] BallisticSolution SolveBallistic(const ControlInputSnapshot& input, int slot,
                                                 const geometry::RigidTransform& world_t_muzzle,
                                                 double base_horizon_s) const;
  /** @brief 在 MPC 各离散时刻求解弹道角，并用中心差分生成速度参考。 */
  [[nodiscard]] std::vector<AimReferencePoint> BuildReference(
      const ControlInputSnapshot& input, int slot, const geometry::RigidTransform& world_t_muzzle,
      double prediction_age_s, BallisticSolution& current, double yaw_anchor) const;
  /** @brief 清除锁定、待切换槽位及开火稳定状态。 */
  void ResetSelection() noexcept;
  /** @brief 请求规划器重建 warm start，并合并本周期的稳定原因字符串。 */
  void RequestPlannerRebase(std::string_view reason) noexcept;

  RulePolicy rule_;
  int external_slot_{-1};
  bool external_mode_{false};
  std::optional<std::uint64_t> round_;
  std::uint64_t generation_{0};
  FireControlConfig config_;                 ///< 不可变火控参数。
  GimbalTrajectoryPlanner planner_;          ///< 双轴 MPC 轨迹规划器及 warm-start 状态。
  std::optional<ArmorLabel> tracked_label_;  ///< 上周期目标标签，用于检测目标变化。
  std::optional<geometry::ArmorType> tracked_type_;  ///< 上周期装甲尺寸。
  std::optional<std::chrono::steady_clock::time_point> last_fire_start_;  ///< 最近脉冲起始时刻。
  std::optional<std::chrono::steady_clock::time_point>
      pulse_until_;  ///< 当前 fire=true 脉冲结束时刻。
  std::optional<std::chrono::steady_clock::time_point> temp_lost_since_;  ///< TEMP_LOST 起始时刻。
  bool tracking_input_valid_{false};  ///< 上周期输入是否通过控制前置校验。
  bool previous_mpc_failed_{false};   ///< 上一周期 MPC 是否失败。
  int solver_cycles_since_reset_{0};  ///< 最近 warm-start 重建后的周期数。
  bool previous_solver_residual_valid_{false};  ///< 是否保存了可比较的上一周期残差。
  double previous_yaw_solver_residual_{0.0};    ///< 上一周期偏航轴最大残差。
  double previous_pitch_solver_residual_{0.0};  ///< 上一周期俯仰轴最大残差。
  bool previous_reference_valid_{false};        ///< 是否保存了上一周期下一参考点。
  double previous_reference_yaw_{0.0};          ///< 上一周期下一参考偏航。
  double previous_reference_pitch_{0.0};        ///< 上一周期下一参考俯仰。
  std::string pending_solver_rebase_reason_{"planner_initialization"};  ///< 待上报重建原因。
};

}  // namespace mv::modules
