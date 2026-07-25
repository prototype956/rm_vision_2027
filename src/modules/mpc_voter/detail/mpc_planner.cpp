/**
 * @file mpc_planner.cpp
 * @brief MpcPlanner TinyMPC 封装实现（Stage 8-E）
 *
 * 【参考】sp_vision_25/tasks/auto_aim/planner/planner.cpp
 *
 * 【与 sp 的差异】
 *   - sp 的 get_trajectory() 基于 Aimer 重新解算飞行时间（需要弹速）；
 *     本实现中 TrajectorySolver 已经将目标的 yaw/pitch 序列生成为参考轨迹；
 *     MpcVoter 直接传入 ref_yaw / ref_pitch 向量，MpcPlanner 只做 MPC 求解。
 *   - sp 全局泄漏 TinySolver*；本实现在析构时手动 delete（TinyMPC 无 free API）。
 */

#include "mpc_planner.hpp"

#include "core/logger.hpp"

#include <cmath>

namespace mv::modules::detail {

// ── 构造 / 析构 ──────────────────────────────────────────────────────────────

MpcPlanner::MpcPlanner(const MpcPlannerParams& params) : params_(params) {
  SetupYawSolver();
  SetupPitchSolver();
  ready_ = (yaw_solver_ != nullptr && pitch_solver_ != nullptr);
  if (!ready_) {
    MV_LOG_WARN("MpcPlanner", "Solver setup failed, Plan() will be a no-op");
  }
}

MpcPlanner::~MpcPlanner() {
  // TinyMPC 通过 new 分配，无 tiny_free；手动释放各子结构后释放外层结构体
  auto free_solver = [](TinySolver* s) {
    if (!s)
      return;
    delete s->solution;
    delete s->cache;
    delete s->settings;
    delete s->work;
    delete s;
  };
  free_solver(yaw_solver_);
  free_solver(pitch_solver_);
}

// ── Plan ─────────────────────────────────────────────────────────────────────

MpcResult MpcPlanner::Plan(double current_yaw, double current_yaw_vel, double current_pitch,
                           double current_pitch_vel, const Eigen::VectorXd& ref_yaw,
                           const Eigen::VectorXd& ref_pitch) {
  if (!ready_)
    return {};

  const int N = params_.horizon;
  // 参考轨迹长度不足时直接放弃（避免越界）
  if (ref_yaw.size() < N || ref_pitch.size() < N) {
    MV_LOG_DEBUG("MpcPlanner", "ref trajectory too short ({} < {})", ref_yaw.size(), N);
    return {};
  }

  const int half = N / 2;
  constexpr int kShootOffset = 2;

  // ── 以当前 yaw 为原点，转换为相对坐标（避免角度累积误差）────────────────
  const double yaw0 = current_yaw;

  // ── yaw 求解 ──────────────────────────────────────────────────────────────
  // 1. 初始状态
  Eigen::Vector2d x0_yaw(current_yaw - yaw0, current_yaw_vel);
  tiny_set_x0(yaw_solver_, x0_yaw);

  // 2. 构建参考轨迹（相对 yaw，数值差分求速度）
  Eigen::MatrixXd yaw_ref = Eigen::MatrixXd::Zero(2, N);
  for (int i = 0; i < N; ++i) {
    yaw_ref(0, i) = ref_yaw[i] - yaw0;  // 相对角度
    // 角速度：中心差分（两端用单边差分）
    if (i == 0) {
      yaw_ref(1, i) = (ref_yaw.size() > 1) ? (ref_yaw[1] - ref_yaw[0]) / params_.dt : 0.0;
    } else if (i == N - 1) {
      yaw_ref(1, i) = (ref_yaw[N - 1] - ref_yaw[N - 2]) / params_.dt;
    } else {
      yaw_ref(1, i) = (ref_yaw[i + 1] - ref_yaw[i - 1]) / (2.0 * params_.dt);
    }
  }
  yaw_solver_->work->Xref = yaw_ref;

  // 3. 求解
  tiny_solve(yaw_solver_);

  // ── pitch 求解 ────────────────────────────────────────────────────────────
  Eigen::Vector2d x0_pitch(current_pitch, current_pitch_vel);
  tiny_set_x0(pitch_solver_, x0_pitch);

  Eigen::MatrixXd pitch_ref = Eigen::MatrixXd::Zero(2, N);
  for (int i = 0; i < N; ++i) {
    pitch_ref(0, i) = ref_pitch[i];
    if (i == 0) {
      pitch_ref(1, i) = (ref_pitch.size() > 1) ? (ref_pitch[1] - ref_pitch[0]) / params_.dt : 0.0;
    } else if (i == N - 1) {
      pitch_ref(1, i) = (ref_pitch[N - 1] - ref_pitch[N - 2]) / params_.dt;
    } else {
      pitch_ref(1, i) = (ref_pitch[i + 1] - ref_pitch[i - 1]) / (2.0 * params_.dt);
    }
  }
  pitch_solver_->work->Xref = pitch_ref;

  tiny_solve(pitch_solver_);

  // ── 生成结果 ──────────────────────────────────────────────────────────────
  MpcResult result;

  // HALF_HORIZON 处规划轨迹（调试用）
  result.planned_yaw = yaw_solver_->work->x(0, half) + yaw0;
  result.planned_pitch = pitch_solver_->work->x(0, half);

  // 开火判断：HALF_HORIZON + shoot_offset 处轨迹跟踪误差 < fire_thresh
  const double yaw_err =
      (ref_yaw[half + kShootOffset] - yaw0) - yaw_solver_->work->x(0, half + kShootOffset);
  const double pitch_err =
      ref_pitch[half + kShootOffset] - pitch_solver_->work->x(0, half + kShootOffset);

  result.fire = std::hypot(yaw_err, pitch_err) < params_.fire_thresh;

  return result;
}

// ── Solver 初始化 ─────────────────────────────────────────────────────────────

void MpcPlanner::SetupYawSolver() {
  // 离散化双积分器：[θ; θ̇]，输入 θ̈
  Eigen::Matrix2d A{{1.0, params_.dt}, {0.0, 1.0}};
  Eigen::Vector2d B{0.0, params_.dt};
  Eigen::Vector2d f{0.0, 0.0};
  Eigen::DiagonalMatrix<double, 2> Q(params_.Q_yaw, params_.Q_yaw_vel);
  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = params_.R_yaw;

  const int ret = tiny_setup(&yaw_solver_, A, B, f, Q, R, params_.rho, /*nx=*/2, /*nu=*/1,
                             params_.horizon, /*verbose=*/0);
  if (ret != 0 || yaw_solver_ == nullptr) {
    MV_LOG_WARN("MpcPlanner", "tiny_setup (yaw) failed, ret={}", ret);
    yaw_solver_ = nullptr;
    return;
  }

  // 加速度约束
  const Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, params_.horizon, -1e17);
  const Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, params_.horizon, 1e17);
  const Eigen::MatrixXd u_min =
      Eigen::MatrixXd::Constant(1, params_.horizon - 1, -params_.max_yaw_acc);
  const Eigen::MatrixXd u_max =
      Eigen::MatrixXd::Constant(1, params_.horizon - 1, params_.max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = params_.max_iter;
}

void MpcPlanner::SetupPitchSolver() {
  Eigen::Matrix2d A{{1.0, params_.dt}, {0.0, 1.0}};
  Eigen::Vector2d B{0.0, params_.dt};
  Eigen::Vector2d f{0.0, 0.0};
  Eigen::DiagonalMatrix<double, 2> Q(params_.Q_pitch, params_.Q_pitch_vel);
  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = params_.R_pitch;

  const int ret = tiny_setup(&pitch_solver_, A, B, f, Q, R, params_.rho, /*nx=*/2, /*nu=*/1,
                             params_.horizon, /*verbose=*/0);
  if (ret != 0 || pitch_solver_ == nullptr) {
    MV_LOG_WARN("MpcPlanner", "tiny_setup (pitch) failed, ret={}", ret);
    pitch_solver_ = nullptr;
    return;
  }

  const Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, params_.horizon, -1e17);
  const Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, params_.horizon, 1e17);
  const Eigen::MatrixXd u_min =
      Eigen::MatrixXd::Constant(1, params_.horizon - 1, -params_.max_pitch_acc);
  const Eigen::MatrixXd u_max =
      Eigen::MatrixXd::Constant(1, params_.horizon - 1, params_.max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = params_.max_iter;
}

}  // namespace mv::modules::detail
