#pragma once

#include "hal/gimbal/gimbal_types.hpp"
#include "modules/gimbal_control/gimbal_trajectory_output.hpp"

#include <memory>
#include <string_view>
#include <vector>

#include <span>

namespace mv::modules {

/** @brief 单个参考采样点的期望云台角度和角速度。 */
struct AimReferencePoint {
  double yaw{0.0};           ///< 期望偏航角，单位为弧度，可连续展开超过 ±pi。
  double yaw_velocity{0.0};  ///< 期望偏航角速度，单位为弧度每秒。
  double pitch{0.0};         ///< 期望俯仰角，单位为弧度。
  double pitch_velocity{0.0};  ///< 期望俯仰角速度，单位为弧度每秒。
};

/** @brief 轨迹规划未产生可发布命令的首要原因。 */
enum class GimbalTrajectoryFailureReason {
  NONE,                          ///< 规划成功或尚未记录失败。
  INVALID_FEEDBACK,              ///< 输入云台反馈 valid=false。
  INVALID_REFERENCE_SIZE,        ///< 参考点数量与配置时域长度不一致。
  NONFINITE_REFERENCE,           ///< 参考角度或角速度包含非有限值。
  YAW_SOLVER_FAILED,             ///< 偏航轴主求解和重试均未收敛。
  PITCH_SOLVER_FAILED,           ///< 俯仰轴主求解和重试均未收敛。
  BOTH_SOLVERS_FAILED,           ///< 两轴主求解和重试均未收敛。
  NONFINITE_SOLUTION,            ///< 反归一化后的轨迹包含非有限值。
  VELOCITY_LIMIT_VIOLATION,      ///< 输出轨迹超过配置角速度约束。
  ACCELERATION_LIMIT_VIOLATION,  ///< 输出轨迹超过配置角加速度约束。
};

/** @brief 失败、重试或 warm-start 操作涉及的云台轴。 */
enum class GimbalTrajectoryAxis {
  NONE,   ///< 不涉及任何轴。
  YAW,    ///< 仅偏航轴。
  PITCH,  ///< 仅俯仰轴。
  BOTH,   ///< 偏航和俯仰两轴。
};

/** @brief 本周期求解前对 TinyMPC warm start 的处理方式。 */
enum class GimbalWarmStartAction {
  NONE,                 ///< 尚未进入求解阶段。
  SHIFT,                ///< 将上一周期解向前平移一个离散步。
  REBASE,               ///< 以当前反馈重建两轴原点和 warm start。
  SHIFT_AFTER_FAILURE,  ///< 上周期失败后仍先平移已有 warm start，再按需单轴重试。
};

/** @brief 将规划失败原因转换为稳定的日志与 Foxglove 字段名称。 */
[[nodiscard]] std::string_view GimbalTrajectoryFailureReasonName(
    GimbalTrajectoryFailureReason reason) noexcept;
/** @brief 将云台轴集合转换为稳定的日志与 Foxglove 字段名称。 */
[[nodiscard]] std::string_view GimbalTrajectoryAxisName(GimbalTrajectoryAxis axis) noexcept;
/** @brief 将 warm-start 操作转换为稳定的日志与 Foxglove 字段名称。 */
[[nodiscard]] std::string_view GimbalWarmStartActionName(GimbalWarmStartAction action) noexcept;

/** @brief TinyMPC 单轴一次求解结束后的状态和四类归一化残差。 */
struct MpcAxisDiagnostics {
  int status{0};                      ///< tiny_solve() 返回状态；0 表示调用成功。
  bool solved{false};                 ///< 求解器是否在迭代上限内满足收敛条件。
  int iterations{0};                  ///< 本次求解实际执行的 ADMM 迭代次数。
  double primal_residual_state{0.0};  ///< 状态约束的原始残差。
  double primal_residual_input{0.0};  ///< 控制输入约束的原始残差。
  double dual_residual_state{0.0};    ///< 状态变量的对偶残差。
  double dual_residual_input{0.0};    ///< 控制输入的对偶残差。
};

/** @brief 双轴 MPC 主求解、重试、参考和性能诊断。 */
struct GimbalTrajectoryDiagnostics {
  bool residuals_normalized{true};  ///< 所有 MpcAxisDiagnostics 残差是否位于归一化空间。
  double normalization_angle_scale_rad{0.1};             ///< 两轴角度状态归一化尺度。
  double normalization_yaw_velocity_scale_rad_s{1.0};    ///< 偏航速度归一化尺度。
  double normalization_pitch_velocity_scale_rad_s{1.0};  ///< 俯仰速度归一化尺度。
  double normalization_yaw_acceleration_scale_rad_s2{1.0};    ///< 偏航输入归一化尺度。
  double normalization_pitch_acceleration_scale_rad_s2{1.0};  ///< 俯仰输入归一化尺度。
  int yaw_iterations{0};            ///< 偏航轴主求解与可选重试的总迭代次数。
  int pitch_iterations{0};          ///< 俯仰轴主求解与可选重试的总迭代次数。
  double solve_time_us{0.0};        ///< 两轴主求解和可选重试总耗时。
  MpcAxisDiagnostics yaw_solver;    ///< 偏航轴最终一次求解诊断。
  MpcAxisDiagnostics pitch_solver;  ///< 俯仰轴最终一次求解诊断。
  MpcAxisDiagnostics primary_yaw_solver;    ///< 偏航轴首次求解诊断。
  MpcAxisDiagnostics primary_pitch_solver;  ///< 俯仰轴首次求解诊断。
  MpcAxisDiagnostics retry_yaw_solver;  ///< 偏航轴 rebase 后重试诊断；未重试时为空。
  MpcAxisDiagnostics retry_pitch_solver;  ///< 俯仰轴 rebase 后重试诊断；未重试时为空。
  GimbalWarmStartAction warm_start_action{GimbalWarmStartAction::NONE};  ///< 初始处理方式。
  GimbalTrajectoryAxis warm_start_rebase_axes{GimbalTrajectoryAxis::NONE};  ///< 主动重建轴。
  bool solver_retry_attempted{false};  ///< 首次未收敛后是否至少重试一个轴。
  GimbalTrajectoryAxis solver_retry_axes{GimbalTrajectoryAxis::NONE};  ///< 重试涉及的轴。
  bool solver_retry_succeeded{false};           ///< 重试后两轴是否均收敛。
  double max_reference_horizon_delta_yaw{0.0};  ///< 同索引参考偏航跨周期最大包角差。
  double max_reference_horizon_delta_pitch{0.0};  ///< 同索引参考俯仰跨周期最大差。
  bool reference_horizon_delta_valid{false};  ///< 是否存在等长上一周期参考可供比较。
  GimbalTrajectoryFailureReason failure_reason{
      GimbalTrajectoryFailureReason::NONE};                       ///< 失败原因。
  GimbalTrajectoryAxis failure_axis{GimbalTrajectoryAxis::NONE};  ///< 失败涉及的轴。
  int failure_index{-1};  ///< 首个非法参考或轨迹点索引；不适用时为 -1。
  std::vector<AimReferencePoint> reference;  ///< 本周期复制保存的输入参考时域。
};

/** @brief 单周期轨迹规划的正式输出与诊断输出。 */
struct GimbalTrajectoryResult {
  GimbalTrajectoryOutput output;
  GimbalTrajectoryDiagnostics diagnostics;
};

}  // namespace mv::modules
