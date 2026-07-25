/**
 * @file trajectory_solver.hpp
 * @brief 飞行时间迭代求解器（从 EKF 目标状态反算最优瞄准点）
 *
 * 【职责】
 *   TrajectorySolver 是 EkfPredictor 的内部弹道计算模块：
 *   1. 从目标 EKF 状态预测"delay_time + fly_time"后的目标位置；
 *   2. 按 jumped/角速度分支选择最优被击装甲板，并通过 lock_id 抑制候选抖动；
 *   3. 通过弹道迭代（最多 10 次）收敛飞行时间；
 *   4. 输出云台需要瞄准的 (yaw, pitch) 偏角。
 *
 * 【弹道模型】
 *   假设子弹在竖直平面内做抛物线运动（不考虑空气阻力，重力修正）：
 *     fly_time ≈ d / (v₀·cosθ)
 *     pitch    = atan(v₀·sinθ / v₀·cosθ) ≈ atan(g·d / v₀²) + atan(Δz / d)
 *   具体实现参见 tools/trajectory.hpp（sp 项目），移植时去掉 sp namespace。
 *
 * 【迭代飞行时间算法（来自 sp 的 Aimer）】
 * @code
 *   future = now + delay_time + fly_time₀
 *   for iter in range(10):
 *     target.Predict(future)            // 预测目标未来位置
 *     aim_point = choose_aim_point()    // 选最优装甲板
 *     new_traj  = Trajectory(speed, d, z)
 *     if |new_traj.fly_time - prev_fly_time| < 1ms: converged, break
 *     prev_fly_time = new_traj.fly_time
 *     future += Δfly_time               // 更新预测时刻
 * @endcode
 *
 * 【与 sp 的 Aimer 的差异】
 *   sp 的 Aimer 直接操作 auto_aim::Target 并输出 io::Command（含 fire 信号）。
 *   新架构中：
 *   - TrajectorySolver 只输出 (yaw, pitch)（无 fire 决策，这是 Voter 的职责）；
 *   - TrajectorySolver 操作 EkfTrackTarget 副本（不修改 EkfTracker 内部的目标）；
 *   - 弹速（bullet_speed）由 PredictNode 从 SharedState 传入（TODO: Stage 8-F）。
 *
 * 【线程安全】
 *   非线程安全。EkfPredictor 在单一线程调用。
 */
#pragma once

#include "ekf_track_target.hpp"
#include "interfaces/types.hpp"

#include <string>

#include <Eigen/Dense>
#include <optional>

namespace mv::modules::detail {

/**
 * @brief TrajectorySolver 配置参数（从 YAML 加载）
 */
struct TrajectorySolverParams {
  double yaw_offset_rad{0.0};    ///< yaw 固定偏置修正（rad），补偿瞄准中心偏差
  double pitch_offset_rad{0.0};  ///< pitch 固定偏置修正（rad）

  /**
   * 根据弹速选择延迟补偿时间（ms）：
   * - 弹速 < decision_speed → low_speed_delay_ms（枪管响应慢）
   * - 弹速 ≥ decision_speed → high_speed_delay_ms（弹速快，飞行时间已短）
   */
  double low_speed_delay_ms{100.0};
  double high_speed_delay_ms{70.0};
  double decision_speed{25.0};  ///< 弹速切换阈值（m/s）

  int max_iter{10};              ///< 飞行时间迭代最大次数
  double iter_converge_ms{1.0};  ///< 迭代收敛阈值（|Δfly_time| < 此值停止）

  /**
   * 瞄准点选择：装甲板正对角度阈值（rad）
   * 若候选装甲板的旋转角 |α - gimbal_yaw| > 此值，认为装甲板侧对云台，不选
   */
  double max_approaching_angle{0.5};
  double max_leaving_angle{0.5};
};

/**
 * @brief 瞄准点结构体（中间结果，供调试可视化）
 */
struct AimPoint {
  bool valid{false};       ///< 是否找到有效瞄准点
  Eigen::Vector4d xyza{};  ///< [x, y, z, yaw]（世界系）
};

/**
 * @brief 飞行时间迭代求解器
 */
class TrajectorySolver {
 public:
  explicit TrajectorySolver(const TrajectorySolverParams& params);

  /**
   * @brief 从当前目标 EKF 状态求解最优 (yaw, pitch)
   *
   * @param target        当前跟踪目标（会对副本进行时间外推，不修改原始状态）
   * @param timestamp     当前帧时间戳（从此点向未来外推）
   * @param bullet_speed  弹速（m/s）；< 14 时退化为固定值 23（保护）
   * @return GimbalControl  tracking=true 时含有效 yaw/pitch；
   *                        tracking=false 表示弹道不可解或无有效装甲板
   *
   * @note 不可解路径返回 tracking=false；fire 字段由 Voter 负责，此处始终为 false。
   * @note current_yaw 参数当前为预留口径，内部调用 ChooseAimPoint 时传入 0.0。
   */
  [[nodiscard]] GimbalControl Solve(const EkfTrackTarget& target,
                                    std::chrono::steady_clock::time_point timestamp,
                                    double bullet_speed);

  /** @return 最近一帧的调试瞄准点（供 Foxglove 可视化）*/
  [[nodiscard]] const AimPoint& GetDebugAimPoint() const noexcept { return debug_aim_point_; }

 private:
  TrajectorySolverParams params_;
  AimPoint debug_aim_point_;
  mutable int lock_id_{-1};  ///< 双候选防抖锁定 id（-1 表示未锁定）

  /**
   * @brief 从目标装甲板列表中选择当前最优瞄准装甲板
   *
   * 选择策略：
   *   1. target.jumped=false 时直接选择 id=0；
   *   2. 非小陀螺分支：用 approaching_angle 筛候选，双候选保持 lock_id，单候选解锁；
   *   3. 小陀螺/前哨站分支：按 approaching/leaving 角度策略选择装甲板。
   *
   * @param target         当前状态（用于 ArmorXyzaList()）
   * @param current_yaw    预留参数，当前未接入（实现中忽略）
   */
  [[nodiscard]] AimPoint ChooseAimPoint(const EkfTrackTarget& target, double current_yaw) const;
};

}  // namespace mv::modules::detail
