#include "modules/gimbal_trajectory_planner/gimbal_trajectory_planner.hpp"

#include "modules/armor_predictor/detail/four_armor_model.hpp"
#include "modules/gimbal_trajectory_planner/detail/tinympc_workspace.hpp"
#include "tinympc/tiny_api.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <numbers>

namespace mv::modules {
namespace {

// TinyMPC C 接口分别分配顶层对象及其四个子对象，需要按所有权逐项释放。
void DestroySolver(TinySolver* solver) noexcept {
  if (solver == nullptr)
    return;
  delete solver->solution;
  delete solver->settings;
  delete solver->cache;
  delete solver->work;
  delete solver;
}

using SolverPtr = std::unique_ptr<TinySolver, decltype(&DestroySolver)>;

/** @brief 单轴角度、角速度和角加速度的物理量归一化尺度。 */
struct AxisNormalization {
  double angle{0.1};
  double velocity{1.0};
  double acceleration{1.0};
};

AxisNormalization MakeNormalization(const GimbalTrajectoryPlannerConfig& config,
                                    double max_velocity, double max_acceleration) noexcept {
  return {.angle = config.normalization_angle_scale_rad,
          .velocity = max_velocity,
          .acceleration = max_acceleration};
}

SolverPtr MakeSolver(const GimbalTrajectoryPlannerConfig& config,
                     const AxisNormalization& normalization, const std::array<double, 2>& q,
                     double r) {
  // 物理空间双积分模型经尺度变换后，在归一化坐标中保持相同离散动力学。
  Eigen::MatrixXd a{{1.0, config.dt_s * normalization.velocity / normalization.angle}, {0.0, 1.0}};
  Eigen::MatrixXd b{
      {0.5 * config.dt_s * config.dt_s * normalization.acceleration / normalization.angle},
      {config.dt_s * normalization.acceleration / normalization.velocity}};
  Eigen::VectorXd f{{0.0, 0.0}};
  Eigen::Vector2d normalized_q(q[0] * normalization.angle * normalization.angle,
                               q[1] * normalization.velocity * normalization.velocity);
  double normalized_r = r * normalization.acceleration * normalization.acceleration;
  // 同比例缩放全部代价不改变最优解，同时避免大权重恶化数值条件。
  const double COST_SCALE = std::max({normalized_q[0], normalized_q[1], normalized_r, 1.0e-12});
  normalized_q /= COST_SCALE;
  normalized_r /= COST_SCALE;
  Eigen::Matrix2d q_matrix = normalized_q.asDiagonal();
  Eigen::Matrix<double, 1, 1> r_matrix;
  r_matrix << normalized_r;
  TinySolver* raw = nullptr;
  if (tiny_setup(&raw, a, b, f, q_matrix, r_matrix, config.rho, 2, 1, config.horizon_steps, 0) !=
          0 ||
      raw == nullptr) {
    DestroySolver(raw);
    throw std::runtime_error("failed to initialize TinyMPC solver");
  }
  SolverPtr result(raw, &DestroySolver);
  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, config.horizon_steps, -1.0e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, config.horizon_steps, 1.0e17);
  x_min.row(1).setConstant(-1.0);
  x_max.row(1).setConstant(1.0);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, config.horizon_steps - 1, -1.0);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, config.horizon_steps - 1, 1.0);
  if (tiny_set_bound_constraints(result.get(), x_min, x_max, u_min, u_max) != 0) {
    throw std::runtime_error("failed to configure TinyMPC bounds");
  }
  result->settings->max_iter = config.max_iterations;
  result->settings->abs_pri_tol = config.absolute_primal_tolerance;
  result->settings->abs_dua_tol = config.absolute_dual_tolerance;
  return result;
}

bool Finite(const PlannedGimbalPoint& point) noexcept {
  return std::isfinite(point.yaw) && std::isfinite(point.yaw_velocity) &&
         std::isfinite(point.yaw_acceleration) && std::isfinite(point.pitch) &&
         std::isfinite(point.pitch_velocity) && std::isfinite(point.pitch_acceleration);
}

MpcAxisDiagnostics Diagnostics(const TinySolver& solver) noexcept {
  return {.status = solver.work->status,
          .solved = solver.solution->solved != 0,
          .iterations = solver.solution->iter,
          .primal_residual_state = solver.work->primal_residual_state,
          .primal_residual_input = solver.work->primal_residual_input,
          .dual_residual_state = solver.work->dual_residual_state,
          .dual_residual_input = solver.work->dual_residual_input};
}

bool Solved(int status, const MpcAxisDiagnostics& diagnostics) noexcept {
  return status == 0 && diagnostics.solved;
}

GimbalTrajectoryAxis FailedAxes(bool yaw_solved, bool pitch_solved) noexcept {
  if (!yaw_solved && !pitch_solved)
    return GimbalTrajectoryAxis::BOTH;
  if (!yaw_solved)
    return GimbalTrajectoryAxis::YAW;
  if (!pitch_solved)
    return GimbalTrajectoryAxis::PITCH;
  return GimbalTrajectoryAxis::NONE;
}

}  // namespace

std::string_view GimbalTrajectoryFailureReasonName(GimbalTrajectoryFailureReason reason) noexcept {
  switch (reason) {
    case GimbalTrajectoryFailureReason::NONE:
      return "none";
    case GimbalTrajectoryFailureReason::INVALID_FEEDBACK:
      return "invalid_feedback";
    case GimbalTrajectoryFailureReason::INVALID_REFERENCE_SIZE:
      return "invalid_reference_size";
    case GimbalTrajectoryFailureReason::NONFINITE_REFERENCE:
      return "nonfinite_reference";
    case GimbalTrajectoryFailureReason::YAW_SOLVER_FAILED:
      return "yaw_solver_failed";
    case GimbalTrajectoryFailureReason::PITCH_SOLVER_FAILED:
      return "pitch_solver_failed";
    case GimbalTrajectoryFailureReason::BOTH_SOLVERS_FAILED:
      return "both_solvers_failed";
    case GimbalTrajectoryFailureReason::NONFINITE_SOLUTION:
      return "nonfinite_solution";
    case GimbalTrajectoryFailureReason::VELOCITY_LIMIT_VIOLATION:
      return "velocity_limit_violation";
    case GimbalTrajectoryFailureReason::ACCELERATION_LIMIT_VIOLATION:
      return "acceleration_limit_violation";
  }
  return "unknown";
}

std::string_view GimbalTrajectoryAxisName(GimbalTrajectoryAxis axis) noexcept {
  switch (axis) {
    case GimbalTrajectoryAxis::NONE:
      return "none";
    case GimbalTrajectoryAxis::YAW:
      return "yaw";
    case GimbalTrajectoryAxis::PITCH:
      return "pitch";
    case GimbalTrajectoryAxis::BOTH:
      return "both";
  }
  return "unknown";
}

std::string_view GimbalWarmStartActionName(GimbalWarmStartAction action) noexcept {
  switch (action) {
    case GimbalWarmStartAction::NONE:
      return "none";
    case GimbalWarmStartAction::SHIFT:
      return "shift";
    case GimbalWarmStartAction::REBASE:
      return "rebase";
    case GimbalWarmStartAction::SHIFT_AFTER_FAILURE:
      return "shift_after_failure";
  }
  return "unknown";
}

struct GimbalTrajectoryPlanner::Impl {
  explicit Impl(GimbalTrajectoryPlannerConfig value)
      : config(std::move(value)),
        yaw_normalization(MakeNormalization(config, config.max_yaw_velocity_rad_s,
                                            config.max_yaw_acceleration_rad_s2)),
        pitch_normalization(MakeNormalization(config, config.max_pitch_velocity_rad_s,
                                              config.max_pitch_acceleration_rad_s2)),
        yaw_solver(MakeSolver(config, yaw_normalization, config.q_yaw, config.r_yaw)),
        pitch_solver(MakeSolver(config, pitch_normalization, config.q_pitch, config.r_pitch)) {}

  GimbalTrajectoryPlannerConfig config;             ///< 不可变规划器参数。
  AxisNormalization yaw_normalization;              ///< 偏航轴物理量尺度。
  AxisNormalization pitch_normalization;            ///< 俯仰轴物理量尺度。
  SolverPtr yaw_solver{nullptr, &DestroySolver};    ///< 偏航双积分 TinyMPC 求解器。
  SolverPtr pitch_solver{nullptr, &DestroySolver};  ///< 俯仰双积分 TinyMPC 求解器。
  bool rebase_requested{true};  ///< 下一周期是否必须重建 warm start 和角度原点。
  bool previous_plan_failed{false};  ///< 上周期是否未产生有效轨迹。
  bool origins_initialized{false};   ///< 两轴局部角度原点是否已建立。
  double yaw_origin{0.0};            ///< 当前偏航归一化局部原点。
  double pitch_origin{0.0};          ///< 当前俯仰归一化局部原点。
  std::vector<AimReferencePoint> previous_reference;  ///< 上周期参考，用于跃变诊断。
};

GimbalTrajectoryPlanner::GimbalTrajectoryPlanner(GimbalTrajectoryPlannerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

GimbalTrajectoryPlanner::~GimbalTrajectoryPlanner() = default;
GimbalTrajectoryPlanner::GimbalTrajectoryPlanner(GimbalTrajectoryPlanner&&) noexcept = default;
GimbalTrajectoryPlanner& GimbalTrajectoryPlanner::operator=(GimbalTrajectoryPlanner&&) noexcept =
    default;

void GimbalTrajectoryPlanner::RequestWarmStartRebase() noexcept {
  impl_->rebase_requested = true;
}

GimbalTrajectoryPlan GimbalTrajectoryPlanner::Plan(const hal::GimbalFeedback& feedback,
                                                   std::span<const AimReferencePoint> reference) {
  GimbalTrajectoryPlan result;
  if (!feedback.valid) {
    result.failure_reason = GimbalTrajectoryFailureReason::INVALID_FEEDBACK;
    result.failure_axis = GimbalTrajectoryAxis::BOTH;
    return result;
  }
  if (reference.size() != static_cast<std::size_t>(impl_->config.horizon_steps)) {
    result.failure_reason = GimbalTrajectoryFailureReason::INVALID_REFERENCE_SIZE;
    result.failure_axis = GimbalTrajectoryAxis::BOTH;
    return result;
  }
  result.reference.assign(reference.begin(), reference.end());
  result.normalization_angle_scale_rad = impl_->yaw_normalization.angle;
  result.normalization_yaw_velocity_scale_rad_s = impl_->yaw_normalization.velocity;
  result.normalization_pitch_velocity_scale_rad_s = impl_->pitch_normalization.velocity;
  result.normalization_yaw_acceleration_scale_rad_s2 = impl_->yaw_normalization.acceleration;
  result.normalization_pitch_acceleration_scale_rad_s2 = impl_->pitch_normalization.acceleration;
  // 以当前反馈作为局部原点可保持归一化角度较小，并避免连续偏航跨越 ±pi 时跳变。
  if (impl_->rebase_requested || !impl_->origins_initialized) {
    impl_->yaw_origin = feedback.yaw;
    impl_->pitch_origin = feedback.pitch;
    impl_->origins_initialized = true;
  }
  const auto START = std::chrono::steady_clock::now();
  Eigen::VectorXd yaw_x0(2);
  yaw_x0 << (feedback.yaw - impl_->yaw_origin) / impl_->yaw_normalization.angle,
      feedback.yaw_velocity / impl_->yaw_normalization.velocity;
  Eigen::VectorXd pitch_x0(2);
  pitch_x0 << (feedback.pitch - impl_->pitch_origin) / impl_->pitch_normalization.angle,
      feedback.pitch_velocity / impl_->pitch_normalization.velocity;
  Eigen::MatrixXd yaw_ref(2, impl_->config.horizon_steps);
  Eigen::MatrixXd pitch_ref(2, impl_->config.horizon_steps);
  for (int index = 0; index < impl_->config.horizon_steps; ++index) {
    const auto& point = reference[static_cast<std::size_t>(index)];
    if (!std::isfinite(point.yaw) || !std::isfinite(point.yaw_velocity) ||
        !std::isfinite(point.pitch) || !std::isfinite(point.pitch_velocity)) {
      result.failure_reason = GimbalTrajectoryFailureReason::NONFINITE_REFERENCE;
      result.failure_axis = GimbalTrajectoryAxis::BOTH;
      result.failure_index = index;
      return result;
    }
    yaw_ref.col(index) << (point.yaw - impl_->yaw_origin) / impl_->yaw_normalization.angle,
        point.yaw_velocity / impl_->yaw_normalization.velocity;
    pitch_ref.col(index) << (point.pitch - impl_->pitch_origin) / impl_->pitch_normalization.angle,
        point.pitch_velocity / impl_->pitch_normalization.velocity;
  }
  if (impl_->previous_reference.size() == reference.size()) {
    // 对齐比较整段时域，而非只比较首点，以发现远端参考突然跳变。
    result.reference_horizon_delta_valid = true;
    for (std::size_t index = 0; index < reference.size(); ++index) {
      result.max_reference_horizon_delta_yaw = std::max(
          result.max_reference_horizon_delta_yaw,
          std::abs(std::remainder(reference[index].yaw - impl_->previous_reference[index].yaw,
                                  2.0 * std::numbers::pi)));
      result.max_reference_horizon_delta_pitch =
          std::max(result.max_reference_horizon_delta_pitch,
                   std::abs(reference[index].pitch - impl_->previous_reference[index].pitch));
    }
  }
  impl_->previous_reference.assign(reference.begin(), reference.end());

  Eigen::MatrixXd yaw_u_ref = Eigen::MatrixXd::Zero(1, impl_->config.horizon_steps - 1);
  Eigen::MatrixXd pitch_u_ref = Eigen::MatrixXd::Zero(1, impl_->config.horizon_steps - 1);
  if (impl_->rebase_requested) {
    // 外部状态断点、换目标或首次求解时，不继承旧轨迹的 ADMM 变量。
    tiny_set_x0(impl_->yaw_solver.get(), yaw_x0);
    tiny_set_x_ref(impl_->yaw_solver.get(), yaw_ref);
    tiny_set_u_ref(impl_->yaw_solver.get(), yaw_u_ref);
    tiny_set_x0(impl_->pitch_solver.get(), pitch_x0);
    tiny_set_x_ref(impl_->pitch_solver.get(), pitch_ref);
    tiny_set_u_ref(impl_->pitch_solver.get(), pitch_u_ref);
    detail::RebaseTinyMpcWarmStart(*impl_->yaw_solver, yaw_x0);
    detail::RebaseTinyMpcWarmStart(*impl_->pitch_solver, pitch_x0);
    result.warm_start_action = GimbalWarmStartAction::REBASE;
    result.warm_start_rebase_axes = GimbalTrajectoryAxis::BOTH;
    impl_->rebase_requested = false;
  } else {
    // 正常滚动时域将上一解左移一个采样点，为本周期提供相邻 warm start。
    detail::ShiftTinyMpcWarmStart(*impl_->yaw_solver);
    detail::ShiftTinyMpcWarmStart(*impl_->pitch_solver);
    tiny_set_x0(impl_->yaw_solver.get(), yaw_x0);
    tiny_set_x_ref(impl_->yaw_solver.get(), yaw_ref);
    tiny_set_u_ref(impl_->yaw_solver.get(), yaw_u_ref);
    tiny_set_x0(impl_->pitch_solver.get(), pitch_x0);
    tiny_set_x_ref(impl_->pitch_solver.get(), pitch_ref);
    tiny_set_u_ref(impl_->pitch_solver.get(), pitch_u_ref);
    result.warm_start_action = impl_->previous_plan_failed
                                   ? GimbalWarmStartAction::SHIFT_AFTER_FAILURE
                                   : GimbalWarmStartAction::SHIFT;
  }

  int yaw_status = detail::SolveTinyMpcWithFreshReference(*impl_->yaw_solver);
  int pitch_status = detail::SolveTinyMpcWithFreshReference(*impl_->pitch_solver);
  result.primary_yaw_solver = Diagnostics(*impl_->yaw_solver);
  result.primary_pitch_solver = Diagnostics(*impl_->pitch_solver);
  bool yaw_solved = Solved(yaw_status, result.primary_yaw_solver);
  bool pitch_solved = Solved(pitch_status, result.primary_pitch_solver);
  result.solver_retry_axes = FailedAxes(yaw_solved, pitch_solved);
  result.solver_retry_attempted = result.solver_retry_axes != GimbalTrajectoryAxis::NONE;
  // 首次未收敛只重建失败轴，保留已收敛轴的结果和诊断。
  if (!yaw_solved) {
    detail::RebaseTinyMpcWarmStart(*impl_->yaw_solver, yaw_x0);
    yaw_status = detail::SolveTinyMpcWithFreshReference(*impl_->yaw_solver);
    result.retry_yaw_solver = Diagnostics(*impl_->yaw_solver);
    yaw_solved = Solved(yaw_status, result.retry_yaw_solver);
  }
  if (!pitch_solved) {
    detail::RebaseTinyMpcWarmStart(*impl_->pitch_solver, pitch_x0);
    pitch_status = detail::SolveTinyMpcWithFreshReference(*impl_->pitch_solver);
    result.retry_pitch_solver = Diagnostics(*impl_->pitch_solver);
    pitch_solved = Solved(pitch_status, result.retry_pitch_solver);
  }
  result.solver_retry_succeeded = result.solver_retry_attempted && yaw_solved && pitch_solved;
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - START).count();
  result.yaw_solver = Diagnostics(*impl_->yaw_solver);
  result.pitch_solver = Diagnostics(*impl_->pitch_solver);
  result.yaw_iterations = result.primary_yaw_solver.iterations + result.retry_yaw_solver.iterations;
  result.pitch_iterations =
      result.primary_pitch_solver.iterations + result.retry_pitch_solver.iterations;
  if (!yaw_solved || !pitch_solved) {
    result.failure_reason = !yaw_solved && !pitch_solved
                                ? GimbalTrajectoryFailureReason::BOTH_SOLVERS_FAILED
                            : !yaw_solved ? GimbalTrajectoryFailureReason::YAW_SOLVER_FAILED
                                          : GimbalTrajectoryFailureReason::PITCH_SOLVER_FAILED;
    result.failure_axis = FailedAxes(yaw_solved, pitch_solved);
  }

  result.trajectory.resize(reference.size());
  bool solution_finite = true;
  // 将归一化解恢复到物理单位，并再次检查硬约束，防止容差内越界进入控制链。
  for (int index = 0; index < impl_->config.horizon_steps; ++index) {
    auto& point = result.trajectory[static_cast<std::size_t>(index)];
    point.yaw = impl_->yaw_origin +
                impl_->yaw_solver->solution->x(0, index) * impl_->yaw_normalization.angle;
    point.yaw_velocity =
        impl_->yaw_solver->solution->x(1, index) * impl_->yaw_normalization.velocity;
    point.pitch = impl_->pitch_origin +
                  impl_->pitch_solver->solution->x(0, index) * impl_->pitch_normalization.angle;
    point.pitch_velocity =
        impl_->pitch_solver->solution->x(1, index) * impl_->pitch_normalization.velocity;
    if (index < impl_->config.horizon_steps - 1) {
      point.yaw_acceleration =
          impl_->yaw_solver->solution->u(0, index) * impl_->yaw_normalization.acceleration;
      point.pitch_acceleration =
          impl_->pitch_solver->solution->u(0, index) * impl_->pitch_normalization.acceleration;
    } else if (index > 0) {
      point.yaw_acceleration =
          result.trajectory[static_cast<std::size_t>(index - 1)].yaw_acceleration;
      point.pitch_acceleration =
          result.trajectory[static_cast<std::size_t>(index - 1)].pitch_acceleration;
    }
    if (!Finite(point)) {
      solution_finite = false;
      if (result.failure_reason == GimbalTrajectoryFailureReason::NONE) {
        result.failure_reason = GimbalTrajectoryFailureReason::NONFINITE_SOLUTION;
        result.failure_axis = GimbalTrajectoryAxis::BOTH;
        result.failure_index = index;
      }
      continue;
    }
    const bool YAW_VELOCITY_VIOLATION =
        std::abs(point.yaw_velocity) > impl_->config.max_yaw_velocity_rad_s + 1.0e-6;
    const bool PITCH_VELOCITY_VIOLATION =
        std::abs(point.pitch_velocity) > impl_->config.max_pitch_velocity_rad_s + 1.0e-6;
    const bool YAW_ACCELERATION_VIOLATION =
        std::abs(point.yaw_acceleration) > impl_->config.max_yaw_acceleration_rad_s2 + 1.0e-6;
    const bool PITCH_ACCELERATION_VIOLATION =
        std::abs(point.pitch_acceleration) > impl_->config.max_pitch_acceleration_rad_s2 + 1.0e-6;
    if (result.failure_reason == GimbalTrajectoryFailureReason::NONE &&
        (YAW_VELOCITY_VIOLATION || PITCH_VELOCITY_VIOLATION || YAW_ACCELERATION_VIOLATION ||
         PITCH_ACCELERATION_VIOLATION)) {
      const bool YAW_VIOLATION = YAW_VELOCITY_VIOLATION || YAW_ACCELERATION_VIOLATION;
      const bool PITCH_VIOLATION = PITCH_VELOCITY_VIOLATION || PITCH_ACCELERATION_VIOLATION;
      result.failure_reason = YAW_VELOCITY_VIOLATION || PITCH_VELOCITY_VIOLATION
                                  ? GimbalTrajectoryFailureReason::VELOCITY_LIMIT_VIOLATION
                                  : GimbalTrajectoryFailureReason::ACCELERATION_LIMIT_VIOLATION;
      result.failure_axis = YAW_VIOLATION && PITCH_VIOLATION ? GimbalTrajectoryAxis::BOTH
                            : YAW_VIOLATION                  ? GimbalTrajectoryAxis::YAW
                                                             : GimbalTrajectoryAxis::PITCH;
      result.failure_index = index;
    }
  }
  if (!yaw_solved || !pitch_solved || !solution_finite ||
      result.failure_reason != GimbalTrajectoryFailureReason::NONE) {
    impl_->previous_plan_failed = true;
    return result;
  }
  result.command_index = std::clamp(
      static_cast<int>(std::llround(impl_->config.command_lookahead_s / impl_->config.dt_s)), 1,
      impl_->config.horizon_steps - 1);
  // 配置前视时间量化到最近离散点，且至少选择 index=1，避免重复发送当前状态。
  result.command_lookahead_s = static_cast<double>(result.command_index) * impl_->config.dt_s;
  result.command = result.trajectory[static_cast<std::size_t>(result.command_index)];
  result.command.yaw = detail::WrapAngle(result.command.yaw);
  result.valid = true;
  impl_->previous_plan_failed = false;
  return result;
}

const GimbalTrajectoryPlannerConfig& GimbalTrajectoryPlanner::Config() const noexcept {
  return impl_->config;
}

}  // namespace mv::modules
