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

/** @brief 单次火控计算所需的同一预测快照、坐标变换和控制使能状态。 */
struct ControlInputSnapshot {
  ArmorPredictionOutput prediction;  ///< 目标跟踪器输出的不可变正式预测快照。
  geometry::RigidTransform world_t_gimbal;           ///< gimbal 到 world 的同帧变换。
  geometry::RigidTransform gimbal_t_camera_optical;  ///< camera_optical 到 gimbal 外参。
  geometry::RigidTransform gimbal_t_muzzle;          ///< muzzle 到 gimbal 的同帧变换。
  std::optional<frame::ChassisMotionObservation> chassis_motion;  ///< 同帧底盘局部运动。
  std::optional<hal::GimbalActuatorTelemetry> frame_actuator;     ///< 相机帧内同步遥测。
  bool external_control_enabled{false};  ///< Talos 是否已启用外部云台控制。
};

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
  FireControlOutput output;
  FireControlDiagnostics diagnostics;
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
  std::optional<geometry::ArmorType> tracked_type_;  ///< 上周期装甲尺寸。
  int last_stable_slot_{-1};                         ///< 开火稳定计数对应的槽位。
  int stable_cycles_{0};  ///< 当前槽位连续进入开火窗口的周期数。
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
