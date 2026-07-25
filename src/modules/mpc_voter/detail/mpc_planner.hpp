/**
 * @file mpc_planner.hpp
 * @brief TinyMPC 轨迹规划器封装（内部实现，不对外暴露）
 *
 * 【设计动机】
 *   MpcVoter 的核心计算是"给定目标轨迹预测，求云台 yaw/pitch 的最优控制序列"。
 *   该计算依赖 TinyMPC (ADMM 在线 MPC 求解器)，数学模型和初始化流程较为复杂。
 *   将其独立封装为 MpcPlanner，使 MpcVoter 的 Vote() 逻辑保持可读性，
 *   同时方便对求解器进行单独测试和参数调优。
 *
 * 【参考实现】
 *   sp_vision_25/tasks/auto_aim/planner/planner.hpp/.cpp
 *   从中保留：
 *   - DT=0.01s，HORIZON=100 的参数设定；
 *   - 分离 yaw/pitch 两个独立 2×1 线性系统；
 *   - Trajectory 矩阵 4×HORIZON（yaw, ẏaw, pitch, ṗitch）；
 *   - fire 判断：HALF_HORIZON 处轨迹误差 < fire_thresh。
 *   删除/替换：
 *   - sp 的 Target 类型 → 本项目 TrackTarget；
 *   - sp 的 yaml tools → yaml-cpp；
 *   - sp 的 math_tools → 内联实现。
 *
 * 【状态空间模型（yaw / pitch 独立）】
 *   状态：[θ, θ̇]（角度、角速度）
 *   输入：[θ̈]（角加速度，约束在 [-max_acc, max_acc]）
 *
 *   离散化（欧拉前向，DT=0.01s）：
 *     θ[k+1] = θ[k] + DT·θ̇[k]
 *     θ̇[k+1] = θ̇[k] + DT·θ̈[k]
 *
 *   代价函数：
 *     J = Σ Q·||θ-θ_ref||² + R·||θ̈||²
 */
#pragma once

#include <Eigen/Dense>
#include <optional>

// mpc_planner.hpp 是内部 detail 头，不对 MpcVoter 外部暴露，
// 直接包含 TinyMPC 头文件（通过 mv-3rdparty-tinympc 的 PUBLIC include 路径访问）。
#include "tinympc/tiny_api.hpp"

namespace mv::modules::detail {

/// MPC 规划输出（仅火控决策，具体 yaw/pitch 已由 TrajectorySolver 给出）
struct MpcResult {
  bool fire{false};         ///< 是否允许开火（HALF_HORIZON 处轨迹误差 < fire_thresh）
  double planned_yaw{0.0};  ///< HALF_HORIZON 处规划 yaw（调试用）
  double planned_pitch{0.0};  ///< HALF_HORIZON 处规划 pitch（调试用）
};

/// 用于构造 MpcPlanner 的参数
struct MpcPlannerParams {
  // ── 求解器超参 ───────────────────────────────────────────────────────
  double dt{0.01};   ///< 离散步长（s），默认 10ms
  int horizon{100};  ///< 预测时域（步数），默认 100 步 = 1s
  int max_iter{10};  ///< ADMM 最大迭代次数
  double rho{1.0};   ///< ADMM 增广拉格朗日参数

  // ── 代价权重 ─────────────────────────────────────────────────────────
  double Q_yaw{1.0};        ///< yaw 追踪代价
  double Q_yaw_vel{1.0};    ///< yaw 速度代价
  double Q_pitch{1.0};      ///< pitch 追踪代价
  double Q_pitch_vel{1.0};  ///< pitch 速度代价
  double R_yaw{1.0};        ///< yaw 加速度代价（输入约束系数）
  double R_pitch{1.0};      ///< pitch 加速度代价

  // ── 执行器约束 ───────────────────────────────────────────────────────
  double max_yaw_acc{200.0};    ///< 最大 yaw 角加速度（rad/s²）
  double max_pitch_acc{200.0};  ///< 最大 pitch 角加速度

  // ── 开火阈值 ─────────────────────────────────────────────────────────
  double fire_thresh{0.05};  ///< HALF_HORIZON 处欧氏角度误差阈值（rad）
};

/**
 * @brief TinyMPC yaw/pitch 双轴规划器封装
 *
 * 管理两个独立 TinySolver 的生命周期，以及每帧的 solve 流程。
 * 只提供一个 Plan() 接口，集中所有 TinyMPC 细节。
 */
class MpcPlanner {
 public:
  /**
   * @brief 构造并初始化两个 TinySolver（yaw / pitch）
   *
   * 在构造时调用 tiny_setup + tiny_set_bound_constraints，
   * 若求解器初始化失败，后续 Plan() 将始终返回 {fire=false}。
   *
   * @param params  MPC 参数（从 yaml 中加载后传入）
   */
  explicit MpcPlanner(const MpcPlannerParams& params);
  ~MpcPlanner();

  MpcPlanner(const MpcPlanner&) = delete;
  MpcPlanner& operator=(const MpcPlanner&) = delete;

  /**
   * @brief 执行单步 MPC 求解
   *
   * @param current_yaw    当前云台 yaw（rad）
   * @param current_yaw_vel  yaw 角速度（rad/s），通常为 0 或来自 IMU
   * @param current_pitch  当前云台 pitch（rad）
   * @param current_pitch_vel  pitch 角速度
   * @param ref_yaw        目标预测 yaw 序列（长度 >= horizon）
   * @param ref_pitch      目标预测 pitch 序列（长度 >= horizon）
   * @return MpcResult（fire + 调试轨迹点）
   */
  MpcResult Plan(double current_yaw, double current_yaw_vel, double current_pitch,
                 double current_pitch_vel, const Eigen::VectorXd& ref_yaw,
                 const Eigen::VectorXd& ref_pitch);

  /// 返回规划器参数（供 MpcVoter 生成参考轨迹时读取 horizon / dt）
  const MpcPlannerParams& params() const noexcept { return params_; }

 private:
  MpcPlannerParams params_;
  TinySolver* yaw_solver_{nullptr};
  TinySolver* pitch_solver_{nullptr};
  bool ready_{false};

  void SetupYawSolver();
  void SetupPitchSolver();
};

}  // namespace mv::modules::detail
