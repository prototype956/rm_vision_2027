/**
 * @file mpc_voter.hpp
 * @brief 基于 TinyMPC 的开火决策投票器（Pimpl 公开接口）
 *
 * 【设计动机：为什么用 MPC 做开火决策而非直接用角度误差】
 *   CooldownVoter 基于瞬时角度误差判断开火，无法预见"即将对准"的情况。
 *   MPC（模型预测控制）将从当前帧起 1s（Horizon=100, DT=10ms）内的
 *   云台运动轨迹一并纳入考量：
 *   - 若 HALF_HORIZON（0.5s 后）处，规划轨迹能跟上目标轨迹（误差 < fire_thresh），
 *     则认为"届时可以击中"，提前允许开火；
 *   - 避免了目标高速旋转时"瞬时对准却会立刻偏离"的误判。
 *
 * 【架构层次】
 *   MpcVoter（IVoter）
 *     └── MpcPlanner（detail/mpc_planner.hpp）
 *           ├── TinySolver (yaw)   ←── 3rdparty/tinympc
 *           └── TinySolver (pitch) ←── 3rdparty/tinympc
 *
 *   平行于 CooldownVoter，不需要 EkfPredictor 的内部状态，
 *   只使用 TrackTarget（由 IPredictor::GetTrackTarget() 提供）。
 *
 * 【Vote() 流程】
 * @code
 *   1. 若 !target.is_tracking → return false
 *   2. 从 target 生成 yaw/pitch 参考轨迹
 *      （以 TrackTarget::armor_xyza_list 推演 Horizon 步后的角度序列）
 *   3. 调用 MpcPlanner::Plan(current_yaw, current_yaw_vel, ..., ref_yaw, ref_pitch)
 *   4. return auto_fire_ && plan.fire
 * @endcode
 *
 * 【YAML 配置字段】（vision.yaml 的 auto_aim.mpc_voter 节点）
 * @code
 *   auto_aim:
 *     mpc_voter:
 *       auto_fire:        true
 *       fire_thresh:      0.05    # HALF_HORIZON 处角度误差阈值（rad）
 *       dt:               0.01    # MPC 步长（s）
 *       horizon:          100     # 预测时域
 *       max_iter:         10
 *       rho:              1.0
 *       Q_yaw:            1.0
 *       Q_yaw_vel:        1.0
 *       Q_pitch:          1.0
 *       Q_pitch_vel:      1.0
 *       R_yaw:            1.0
 *       R_pitch:          1.0
 *       max_yaw_acc:      200.0   # rad/s²
 *       max_pitch_acc:    200.0
 * @endcode
 *
 * 工厂键：`"mpc"`
 */
#pragma once

#include "interfaces/i_voter.hpp"

#include <memory>

namespace mv::modules {

/**
 * @brief TinyMPC 开火投票器（Pimpl，工厂键 "mpc"）
 */
class MpcVoter final : public IVoter {
 public:
  MpcVoter();
  ~MpcVoter() override;

  MpcVoter(const MpcVoter&) = delete;
  MpcVoter& operator=(const MpcVoter&) = delete;
  MpcVoter(MpcVoter&&) = delete;
  MpcVoter& operator=(MpcVoter&&) = delete;

  // ── IVoter 接口实现 ────────────────────────────────────────────────────

  bool Init(const YAML::Node& config) override;

  /**
   * @brief 调用 MPC 规划器，判断 0.5s 后云台能否跟上目标轨迹
   *
   * @param target  跟踪状态快照（位置/速度/yaw_predicted）
   * @param control 本帧云台指令（读取 yaw/pitch 作为 MPC 初始状态）
   * @return true = MPC 预测 HALF_HORIZON 处误差 < fire_thresh 且 auto_fire == true
   */
  [[nodiscard]] bool Vote(const TrackTarget& target, const GimbalControl& control) override;

  /**
   * @brief 清除内部状态（重置 MPC 求解器初始条件）
   */
  void Reset() override;

  [[nodiscard]] bool IsInitialized() const noexcept override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::modules
