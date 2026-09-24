#pragma once
#include "frame/frame_types.hpp"
#include "hal/gimbal/gimbal_types.hpp"
#include "modules/armor_predictor/armor_prediction_output.hpp"
#include "modules/fire_control/fire_control_config.hpp"
#include "modules/fire_control/fire_control_output.hpp"
#include "modules/fire_control/gimbal_feedback_estimator.hpp"
#include "modules/gimbal_trajectory_planner/gimbal_trajectory_planner.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <string>
#include <string_view>

#include <optional>

namespace mv::modules {

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
  std::uint64_t source_sequence{0};  ///< 对应输入采集帧序号。
  TrackerState tracker_state{TrackerState::LOST};   ///< 选择时的跟踪状态。
  std::optional<ArmorLabel> tracked_label;          ///< 当前目标标签；未跟踪时为空。
  std::optional<geometry::ArmorType> tracked_type;  ///< 当前装甲尺寸。
  int selected_slot{-1};                            ///< 已锁定的四装甲槽位。
  int pending_slot{-1};                             ///< 正等待切换确认的槽位。
  double pending_duration_s{0.0};                   ///< pending_slot 已持续更优的时间。
  double switch_confirmation_s{0.0};                ///< 配置的切换确认时间。
};

/** @brief 供相机同帧命中预测标注使用的轻量火控快照。 */
struct ArmorImpactSnapshot {
  std::uint64_t source_sequence{0};  ///< 对应输入预测和相机采集帧序号。
  int selected_slot{-1};             ///< 最终参与弹道求解的装甲槽位。
  BallisticSolution ballistic;       ///< 弹道目标、飞行时间及其准确预测时域。
};

/** @brief Deployment-visible sampled referee data; no evaluation counters. */
struct RefereeObservation {
  bool valid{false};
  bool alive{false};
  bool fire_permitted{false};
  bool unlimited{false};
  std::uint32_t allowance_remaining{0};
  std::uint32_t fire_blocks{0};
  double heat{0.0};
  double heat_limit{0.0};
  double cooling_per_second{0.0};
  std::uint64_t sample_sequence{0};
  std::uint64_t sample_time_ns{0};  ///< Source simulation clock, metadata only.
  double age_at_receive_s{0.0};
  std::chrono::steady_clock::time_point received_at{};
};

[[nodiscard]] FireRejectReason RefereeRejectReason(const RefereeObservation& value,
                                                   std::chrono::steady_clock::time_point now,
                                                   double max_age_s) noexcept;

/** @brief Version 1 actions: 0 wait; 1+2*i track; 2+2*i request one shot. */
struct PolicyDecision {
  int action{0};
  [[nodiscard]] bool Valid() const noexcept { return action >= 0 && action <= 8; }
  [[nodiscard]] int Slot() const noexcept { return action == 0 ? -1 : (action - 1) / 2; }
  [[nodiscard]] bool RequestsShot() const noexcept { return action > 0 && action % 2 == 0; }
};

/** @brief 单次火控计算所需的同一预测快照、坐标变换和控制使能状态。 */
struct ControlInputSnapshot {
  RefereeObservation referee;
  ArmorPredictionOutput prediction;  ///< 目标跟踪器输出的不可变正式预测快照。
  geometry::RigidTransform world_t_gimbal;           ///< gimbal 到 world 的同帧变换。
  geometry::RigidTransform gimbal_t_camera_optical;  ///< camera_optical 到 gimbal 外参。
  geometry::RigidTransform gimbal_t_muzzle;          ///< muzzle 到 gimbal 的同帧变换。
  std::optional<frame::ChassisMotionObservation> chassis_motion;  ///< 同帧底盘局部运动。
  std::optional<hal::GimbalActuatorTelemetry> frame_actuator;     ///< 相机帧内同步遥测。
  bool external_control_enabled{false};  ///< Talos 是否已启用外部云台控制。
};

[[nodiscard]] bool ControlValuesFinite(const ControlInputSnapshot& input) noexcept;

/** @brief 相机采集时刻与历史已发布云台命令的匹配结果。 */
struct MatchedGimbalCommand {
  bool valid{false};  ///< 是否找到一条有效历史命令。
  bool approximate{true};  ///< true 表示按时间近似匹配，false 表示执行器确认精确匹配。
  hal::GimbalCommand command;    ///< 匹配到的历史命令。
  double age_at_capture_s{0.0};  ///< 相机采集时命令已经生成的时间。
};

/** @brief 单周期火控的选择、反馈、求解过程和性能诊断。 */
struct FireControlDiagnostics {
  ArmorSelectionDiagnostics armor_selection;
  hal::GimbalFeedback feedback;
  hal::GimbalFeedback measured_feedback;
  bool measurement_fresh{false};
  double measurement_age_s{0.0};
  MatchedGimbalCommand matched_prior_command;
  hal::GimbalActuatorTelemetry actuator_telemetry;
  std::optional<hal::GimbalActuatorTelemetry> frame_actuator_telemetry;
  double runtime_actuator_age_s{0.0};
  double frame_actuator_age_s{0.0};
  double feedback_projection_dt_s{0.0};
  double pose_projection_dt_s{0.0};
  bool chassis_motion_valid{false};
  std::string pose_projection_status;
  std::optional<frame::FrameKinematics> projected_kinematics;
  std::uint64_t feedback_runtime_state_timestamp_ns{0};
  bool feedback_runtime_comparison_valid{false};
  double yaw_feedback_minus_runtime_actuator{0.0};
  double pitch_feedback_minus_runtime_actuator{0.0};
  bool frame_runtime_comparison_valid{false};
  double yaw_frame_minus_runtime_actuator{0.0};
  double pitch_frame_minus_runtime_actuator{0.0};
  double yaw_frame_acceleration_minus_runtime{0.0};
  double pitch_frame_acceleration_minus_runtime{0.0};
  GimbalTrajectoryDiagnostics plan;
  GimbalFeedbackSource feedback_source{GimbalFeedbackSource::NONE};
  bool solver_warm_start_reset{false};
  std::string solver_warm_start_reset_reason;
  bool solver_continued_after_failure{false};
  int solver_cycles_since_reset{0};
  bool previous_solver_residual_valid{false};
  double previous_yaw_solver_residual{0.0};
  double previous_pitch_solver_residual{0.0};
  double yaw_solver_residual_delta{0.0};
  double pitch_solver_residual_delta{0.0};
  bool fallback_expired_this_cycle{false};
  bool output_projection_cleared{false};
  std::string output_projection_clear_reason;
  bool reference_step_valid{false};
  double reference_yaw_step{0.0};
  double reference_pitch_step{0.0};
  double target_linear_speed_mps{0.0};
  double target_spin_rate_rad_s{0.0};
  double control_compute_time_us{
      0.0};  ///< Step and publication bookkeeping, excluding transport send.
  double referee_age_s{0.0};
  double control_period_s{0.0};
  double deadline_lateness_us{0.0};
  double sink_send_time_us{0.0};
  double yaw_error{0.0};
  double pitch_error{0.0};
  double trajectory_dt_s{0.0};
  double bullet_speed_mps{0.0};
  double max_yaw_velocity_rad_s{0.0};
  double max_pitch_velocity_rad_s{0.0};
  double max_yaw_acceleration_rad_s2{0.0};
  double max_pitch_acceleration_rad_s2{0.0};
};

/** @brief 单周期火控的正式输出与诊断输出。 */
struct FireControlResult {
  std::uint64_t cycle_id{0};
  const void* session_owner{
      nullptr};  ///< Process-local acknowledgement identity; not an observation.
  std::optional<PolicyDecision> decision;
  FireControlOutput output;
  FireControlDiagnostics diagnostics;
};

/** @brief Unnormalized semantic observation, world Z-up; angles rad, distances m, ages s. */
struct PolicyObservation {
  static constexpr std::uint32_t VERSION = 1;
  ArmorPredictionOutput estimate;
  hal::GimbalFeedback feedback;
  RefereeObservation referee;
  std::optional<frame::ChassisMotionObservation> chassis_motion;
  std::array<BallisticSolution, 4> candidates{};
  std::array<double, 4> facing_now_rad{};  ///< 水平面上外法线到板指向炮口的有符号角，+Z 为正。
  std::array<double, 4> facing_impact_rad{};  ///< 在各候选弹道的准确命中时域上计算的同定义角。
  std::array<bool, 9> action_mask{};
  double prediction_age_s{0.0};
  double feedback_age_s{0.0};
  double referee_age_s{0.0};
  int previous_slot{-1};
  double selected_slot_age_s{0.0};  ///< 当前槽位已持续时间；无槽位时为零，不包含仿真真值。
  std::optional<double> since_request_s;
};

/**
 * @brief 同步策略回调；输入已完成反馈融合和位姿投影，与随后 MPC 使用的快照一致。
 *
 * 引用仅在回调期间有效。回调不得重入会话；策略历史由调用者按回合、目标和模式管理。
 * ControlInputSnapshot 仅供规则选板适配；模型特征必须取 PolicyObservation 的白名单。
 */
using ControlPolicy =
    std::function<PolicyDecision(const ControlInputSnapshot&, const PolicyObservation&)>;
}  // namespace mv::modules
