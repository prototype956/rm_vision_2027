#pragma once

#include "hal/camera/i_camera.hpp"
#include "hal/gimbal/gimbal_types.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"
#include "modules/fire_control/fire_control_config.hpp"
#include "modules/fire_control/gimbal_feedback_estimator.hpp"
#include "modules/gimbal_trajectory_planner/gimbal_trajectory_planner.hpp"

#include <array>
#include <chrono>
#include <string>
#include <string_view>

#include <optional>

namespace mv::modules {

/** @brief 本控制周期未给出开火建议的首要门控原因。 */
enum class FireRejectReason {
  NONE,                       ///< 当前正在输出开火脉冲。
  AUTO_FIRE_DISABLED,         ///< 配置未启用自动开火。
  EXTERNAL_CONTROL_DISABLED,  ///< Talos 未启用外部云台控制。
  TRACK_NOT_CONFIRMED,        ///< 跟踪状态尚未进入 TRACKING。
  TEMPORARY_LOSS,             ///< 目标处于 TEMP_LOST，仅允许维持控制。
  STALE_PREDICTION,           ///< 目标预测数据年龄超过配置上限。
  INVALID_FEEDBACK,           ///< 云台反馈无效或包含非有限值。
  STALE_FEEDBACK,             ///< 云台反馈数据年龄超过配置上限。
  TALOS_UNHEALTHY,            ///< Talos 命令通道断开、心跳超时或发送失败。
  NUMERICAL_INVALID,          ///< 预测状态、协方差或坐标变换包含非法数值。
  HIGH_UNCERTAINTY,           ///< 目标位置或航向标准差超过开火门限。
  NO_SHOOTABLE_ARMOR,         ///< 四个装甲槽位均不满足观察角条件。
  BALLISTIC_UNSOLVABLE,       ///< 弹道方程无解或飞行时间迭代未收敛。
  MPC_FAILED,                 ///< 云台轨迹规划器未生成有效 MPC 轨迹。
  AIM_ERROR_TOO_LARGE,        ///< 当前瞄准误差尚未进入距离自适应窗口。
  AIM_NOT_STABLE,             ///< 进入窗口的连续控制周期数不足。
  COOLDOWN,                   ///< 满足开火条件，但最小脉冲间隔尚未结束。
};

/** @brief 将开火拒绝原因转换为稳定的日志与 Foxglove 字段名称。 */
[[nodiscard]] std::string_view FireRejectReasonName(FireRejectReason reason) noexcept;

/** @brief 本周期装甲槽位选择状态机的决策。 */
enum class ArmorSelectionDecision {
  NONE,               ///< 尚未运行选择或没有决策。
  ACQUIRED,           ///< 从无锁定状态首次选中一个槽位。
  HELD,               ///< 当前锁定槽位仍满足滞回条件并继续保持。
  PENDING_SWITCH,     ///< 更优槽位出现，正在等待持续时间确认。
  SWITCHED,           ///< 已确认并切换到更优槽位。
  LOST_ANGLE,         ///< 原锁定槽位越过离开角门限。
  NO_CANDIDATE,       ///< 没有槽位满足进入角门限。
  TEMP_LOST_HELD,     ///< TEMP_LOST 期间按离开角门限保持原槽位。
  TEMP_LOST_CLEARED,  ///< TEMP_LOST 期间原槽位也不可见，清除锁定。
};

/** @brief 将槽位选择决策转换为稳定的日志与 Foxglove 字段名称。 */
[[nodiscard]] std::string_view ArmorSelectionDecisionName(ArmorSelectionDecision decision) noexcept;

/** @brief 最终交给云台命令后端的指令来源。 */
enum class GimbalCommandSource {
  MPC,                  ///< 当前周期新求解的 MPC 命令。
  TRAJECTORY_FALLBACK,  ///< MPC 短时失败后沿用上一条成功轨迹。
  STOP,                 ///< 无安全控制输出，发送 valid=false 停止命令。
};

/** @brief 将命令来源转换为稳定的日志与 Foxglove 字段名称。 */
[[nodiscard]] std::string_view GimbalCommandSourceName(GimbalCommandSource source) noexcept;

/** @brief 一个四装甲槽位在选择时域上的可见性和转动代价。 */
struct ArmorSelectionCandidate {
  int slot{-1};                       ///< 四装甲模型槽位编号。
  PredictedArmorPose predicted_pose;  ///< 选择时域上的 world 系预测位姿。
  double view_angle_rad{0.0};         ///< 装甲法向与装甲指向枪口方向的夹角。
  double slew_angle_rad{0.0};         ///< 从当前云台角转到目标方向的二维角距离。
  bool enter_eligible{false};         ///< 是否满足新锁定槽位的进入角门限。
  bool leave_eligible{false};         ///< 是否满足已锁槽位的较宽离开角门限。
};

/** @brief 单周期四装甲选择、滞回和延时切换诊断。 */
struct ArmorSelectionDiagnostics {
  double horizon_s{0.0};  ///< 候选位姿相对预测基准的选择时域，单位为秒。
  double switch_confirmation_s{0.0};  ///< 配置的槽位切换确认时间。
  int locked_slot{-1};                ///< 本周期结束后的锁定槽位；-1 表示无锁定。
  int pending_slot{-1};               ///< 等待切换确认的候选槽位；-1 表示无。
  double pending_duration_s{0.0};     ///< 当前候选已持续更优的时间。
  bool switched{false};               ///< 本周期是否实际改变了锁定槽位。
  ArmorSelectionDecision decision{ArmorSelectionDecision::NONE};  ///< 状态机决策。
  std::array<ArmorSelectionCandidate, 4> candidates{};            ///< 四个固定槽位诊断。
};

/** @brief 供相机同帧调试标注使用的轻量控制选择快照，不包含 MPC 轨迹。 */
struct ArmorSelectionSnapshot {
  bool valid{false};                 ///< 是否存在可与相机帧关联的选择结果。
  std::uint64_t source_sequence{0};  ///< 对应输入 CameraFrame::sequence。
  TrackerState tracker_state{TrackerState::LOST};  ///< 选择时的跟踪状态。
  std::optional<ArmorLabel> tracked_label;         ///< 当前目标标签；未跟踪时为空。
  std::optional<hal::CameraFrame::ArmorType> tracked_type;  ///< 当前装甲尺寸。
  int selected_slot{-1};                                    ///< 已锁定的四装甲槽位。
  int pending_slot{-1};                                     ///< 正等待切换确认的槽位。
  double pending_duration_s{0.0};     ///< pending_slot 已持续更优的时间。
  double switch_confirmation_s{0.0};  ///< 配置的切换确认时间。
};

/** @brief 单次火控计算所需的同一预测快照、坐标变换和控制使能状态。 */
struct ControlInputSnapshot {
  ArmorPredictionResult prediction;          ///< 目标跟踪器输出的不可变预测快照。
  geometry::RigidTransform world_t_gimbal;   ///< gimbal 到 world 的同帧变换。
  geometry::RigidTransform gimbal_t_muzzle;  ///< muzzle 到 gimbal 的同帧变换。
  std::optional<hal::GimbalActuatorTelemetry> frame_actuator;  ///< 相机帧内同步遥测。
  bool external_control_enabled{false};  ///< Talos 是否已启用外部云台控制。
};

/** @brief 指定装甲槽位的一次低抛物线弹道解。 */
struct BallisticSolution {
  bool valid{false};  ///< 飞行时间迭代是否收敛且目标位于枪口前方。
  int slot{-1};       ///< 本次求解对应的四装甲槽位。
  geometry::Vector3 target_world{geometry::Vector3::Zero()};  ///< 命中时刻目标世界坐标。
  double yaw{0.0};         ///< 枪口目标偏航角，单位为弧度。
  double pitch{0.0};       ///< 补偿重力后的低弹道俯仰角，单位为弧度。
  double distance_m{0.0};  ///< 枪口到预测命中点的直线距离。
  double fly_time_s{0.0};  ///< 弹丸到达预测命中点的飞行时间。
};

/** @brief 相机采集时刻与历史已发布云台命令的匹配结果。 */
struct MatchedGimbalCommand {
  bool valid{false};  ///< 是否找到一条有效历史命令。
  bool approximate{true};  ///< true 表示按时间近似匹配，false 表示执行器确认精确匹配。
  hal::GimbalCommand command;    ///< 匹配到的历史命令。
  double age_at_capture_s{0.0};  ///< 相机采集时命令已经生成的时间。
};

/** @brief 单周期控制输出以及用于运行监控、回放和 Foxglove 的完整诊断。 */
struct FireControlResult {
  std::uint64_t source_sequence{0};  ///< 对应输入预测和相机帧序号。
  std::optional<std::uint64_t> source_capture_timestamp_ns;  ///< 数据源采集 Unix 时间。
  std::uint64_t command_timestamp_ns{0};  ///< 本周期命令生成的 Unix epoch 纳秒时间。
  TrackerState tracker_state{TrackerState::LOST};           ///< 输入目标跟踪状态。
  std::optional<ArmorLabel> tracked_label;                  ///< 输入目标标签。
  std::optional<hal::CameraFrame::ArmorType> tracked_type;  ///< 输入目标装甲尺寸。
  double prediction_age_s{0.0};  ///< 控制时刻相对预测源帧接收时刻的数据年龄。
  double feedback_age_s{0.0};    ///< 控制时刻相对融合反馈时刻的数据年龄。
  int selected_slot{-1};         ///< 当前选中的四装甲槽位；-1 表示无。
  ArmorSelectionDiagnostics armor_selection;  ///< 完整槽位选择诊断。
  BallisticSolution ballistic;                ///< 当前槽位的弹道解。
  geometry::RigidTransform world_t_muzzle;    ///< muzzle 到 world 的同帧组合变换。
  bool muzzle_pose_valid{false};              ///< world_t_muzzle 是否通过输入校验。

  hal::GimbalFeedback feedback;                ///< MPC 使用的融合云台反馈。
  hal::GimbalFeedback measured_feedback;       ///< 最近相机源帧的直接位姿量测。
  bool measurement_fresh{false};               ///< 本周期是否接收到新相机量测。
  double measurement_age_s{0.0};               ///< 最近相机量测的数据年龄。
  MatchedGimbalCommand matched_prior_command;  ///< 与源帧采集时刻匹配的历史命令。
  hal::GimbalActuatorTelemetry actuator_telemetry;  ///< 控制时刻异步读取的执行器遥测。
  std::optional<hal::GimbalActuatorTelemetry> frame_actuator_telemetry;  ///< 源帧同步遥测。
  double runtime_actuator_age_s{0.0};    ///< 异步执行器遥测的数据年龄。
  double frame_actuator_age_s{0.0};      ///< 源帧同步遥测相对控制时刻的数据年龄。
  double feedback_projection_dt_s{0.0};  ///< 反馈估计器执行命令外推的时间跨度。
  std::uint64_t feedback_runtime_state_timestamp_ns{0};  ///< 反馈采用的运行时状态时间戳。
  bool feedback_runtime_comparison_valid{false};  ///< 融合反馈和异步遥测能否直接比较。
  double yaw_feedback_minus_runtime_actuator{0.0};    ///< 融合偏航减运行时实际偏航。
  double pitch_feedback_minus_runtime_actuator{0.0};  ///< 融合俯仰减运行时实际俯仰。
  bool frame_runtime_comparison_valid{false};  ///< 同帧遥测和异步遥测能否直接比较。
  double yaw_frame_minus_runtime_actuator{0.0};  ///< 同帧实际偏航减运行时实际偏航。
  double pitch_frame_minus_runtime_actuator{0.0};  ///< 同帧实际俯仰减运行时实际俯仰。
  double yaw_frame_acceleration_minus_runtime{0.0};    ///< 同帧减运行时偏航角加速度。
  double pitch_frame_acceleration_minus_runtime{0.0};  ///< 同帧减运行时俯仰角加速度。

  GimbalTrajectoryPlan plan;           ///< 本周期完整 MPC 求解与轨迹诊断。
  PlannedGimbalPoint raw_mpc_command;  ///< 当前轨迹中原始的下一控制点。
  hal::GimbalCommand command;          ///< 最终准备或已经提交给后端的命令。
  GimbalCommandSource command_source{GimbalCommandSource::STOP};  ///< 最终命令来源。
  bool raw_mpc_valid{false};          ///< 当前 MPC 轨迹是否有效。
  bool published_valid{false};        ///< 最终是否成功发布一条 valid=true 命令。
  bool fallback_active{false};        ///< 当前是否沿用上一条成功 MPC 轨迹。
  double fallback_age_s{0.0};         ///< 上一条成功轨迹距当前控制时刻的年龄。
  int fallback_source_slot{-1};       ///< 回退轨迹求解时对应的装甲槽位。
  int fallback_trajectory_index{-1};  ///< 本周期采用的回退轨迹点索引。
  int fallback_remaining_points{0};   ///< 回退轨迹中尚可使用的点数。
  int consecutive_mpc_failure_cycles{0};  ///< 连续 MPC 求解失败周期数。
  GimbalFeedbackSource feedback_source{GimbalFeedbackSource::NONE};  ///< 融合反馈来源。

  bool solver_warm_start_reset{false};         ///< 本周期是否重建求解器 warm start。
  std::string solver_warm_start_reset_reason;  ///< 触发重建的逗号分隔稳定原因。
  bool solver_continued_after_failure{false};  ///< 上周期失败后本周期是否继续求解。
  int solver_cycles_since_reset{0};            ///< 最近 warm start 重建后的求解周期数。
  bool previous_solver_residual_valid{false};  ///< 是否保存了上一周期残差基线。
  double previous_yaw_solver_residual{0.0};    ///< 上一周期偏航轴最大残差。
  double previous_pitch_solver_residual{0.0};  ///< 上一周期俯仰轴最大残差。
  double yaw_solver_residual_delta{0.0};       ///< 当前减上一周期偏航轴最大残差。
  double pitch_solver_residual_delta{0.0};     ///< 当前减上一周期俯仰轴最大残差。
  bool fallback_expired_this_cycle{false};     ///< 回退轨迹是否在本周期耗尽或超龄。
  bool output_projection_cleared{false};       ///< 命令反馈投影是否在本周期被清除。
  std::string output_projection_clear_reason;  ///< 清除投影的逗号分隔原因。
  bool reference_step_valid{false};            ///< 是否能与上一周期比较下一参考点。
  double reference_yaw_step{0.0};    ///< 下一参考偏航相对上一周期的变化量。
  double reference_pitch_step{0.0};  ///< 下一参考俯仰相对上一周期的变化量。

  double target_linear_speed_mps{0.0};  ///< EKF 车辆中心三维速度模长。
  double target_spin_rate_rad_s{0.0};   ///< EKF 目标航向角速度绝对值。
  bool tracking_object_reset{false};    ///< 标签或装甲尺寸变化是否触发状态重置。
  bool command_publish_succeeded{false};  ///< 后端是否接受本周期有效命令。
  double control_period_s{0.0};           ///< 相邻控制周期开始时刻间隔。
  double deadline_lateness_us{0.0};       ///< 控制线程超过计划唤醒时刻的延迟。
  double sink_send_time_us{0.0};          ///< 命令后端 Send() 调用耗时。

  double target_yaw{0.0};                   ///< 当前弹道参考偏航角，单位为弧度。
  double target_pitch{0.0};                 ///< 当前弹道参考俯仰角，单位为弧度。
  double yaw_error{0.0};                    ///< 目标偏航减融合反馈偏航的包角误差。
  double pitch_error{0.0};                  ///< 目标俯仰减融合反馈俯仰的误差。
  double fire_yaw_window{0.0};              ///< 距离自适应并钳位后的偏航开火窗口。
  double fire_pitch_window{0.0};            ///< 距离自适应并钳位后的俯仰开火窗口。
  double trajectory_dt_s{0.0};              ///< MPC 相邻轨迹点时间间隔。
  double bullet_speed_mps{0.0};             ///< 本周期弹道计算使用的弹丸速度。
  double max_yaw_velocity_rad_s{0.0};       ///< 规划器偏航速度约束。
  double max_pitch_velocity_rad_s{0.0};     ///< 规划器俯仰速度约束。
  double max_yaw_acceleration_rad_s2{0.0};  ///< 规划器偏航加速度约束。
  double max_pitch_acceleration_rad_s2{0.0};  ///< 规划器俯仰加速度约束。

  bool external_control_enabled{false};  ///< Talos 是否正在接受外部控制。
  bool command_sink_healthy{false};      ///< 命令后端连接和心跳是否健康。
  bool auto_fire_enabled{false};         ///< 配置是否允许自动开火。
  std::uint64_t talos_heartbeat_ns{0};   ///< Talos 最近心跳 Unix epoch 纳秒时间。
  bool fire_eligible{false};             ///< 除脉冲冷却外的开火门控是否全部通过。
  int stable_cycles{0};                  ///< 当前槽位连续落入开火窗口的周期数。
  FireRejectReason reject_reason{FireRejectReason::TRACK_NOT_CONFIRMED};  ///< 首要拒绝原因。
};

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
                                       std::chrono::steady_clock::time_point now);
  /** @brief 清除瞄准稳定计数和正在输出的开火脉冲，不改变槽位锁定。 */
  void ResetFireReadiness() noexcept;
  /** @brief 返回构造时保存的只读火控配置。 */
  [[nodiscard]] const FireControlConfig& Config() const noexcept { return config_; }

 private:
  /** @brief 在预测时域上按观察角滞回和持续改善条件选择四装甲槽位。 */
  [[nodiscard]] int SelectSlot(const ControlInputSnapshot& input,
                               const geometry::Vector3& muzzle_world, double horizon_s,
                               const hal::GimbalFeedback& feedback,
                               std::chrono::steady_clock::time_point now,
                               ArmorSelectionDiagnostics& diagnostics);
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

  FireControlConfig config_;         ///< 不可变火控参数。
  GimbalTrajectoryPlanner planner_;  ///< 双轴 MPC 轨迹规划器及 warm-start 状态。
  int locked_slot_{-1};              ///< 当前锁定的四装甲槽位。
  int pending_slot_{-1};             ///< 等待延时确认的切换候选槽位。
  std::chrono::steady_clock::time_point pending_since_{};  ///< 候选首次持续更优的时刻。
  std::optional<ArmorLabel> tracked_label_;  ///< 上周期目标标签，用于检测目标变化。
  std::optional<hal::CameraFrame::ArmorType> tracked_type_;  ///< 上周期装甲尺寸。
  int last_stable_slot_{-1};  ///< 开火稳定计数对应的槽位。
  int stable_cycles_{0};      ///< 当前槽位连续进入开火窗口的周期数。
  std::chrono::steady_clock::time_point last_fire_start_{};  ///< 最近脉冲起始时刻。
  std::chrono::steady_clock::time_point pulse_until_{};  ///< 当前 fire=true 脉冲结束时刻。
  std::chrono::steady_clock::time_point temp_lost_since_{};  ///< TEMP_LOST 起始时刻。
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
