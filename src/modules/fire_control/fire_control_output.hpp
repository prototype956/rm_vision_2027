#pragma once

#include "hal/gimbal/gimbal_types.hpp"
#include "modules/armor_predictor/armor_prediction_output.hpp"
#include "modules/gimbal_trajectory_planner/gimbal_trajectory_output.hpp"

#include <cstdint>
#include <string_view>

#include <optional>

namespace mv::modules {

/** @brief 本控制周期未给出开火建议的首要安全门控原因。 */
enum class FireRejectReason {
  NONE,
  AUTO_FIRE_DISABLED,
  EXTERNAL_CONTROL_DISABLED,
  TRACK_NOT_CONFIRMED,
  TEMPORARY_LOSS,
  STALE_PREDICTION,
  INVALID_FEEDBACK,
  STALE_FEEDBACK,
  TALOS_UNHEALTHY,
  NUMERICAL_INVALID,
  HIGH_UNCERTAINTY,
  NO_SHOOTABLE_ARMOR,
  BALLISTIC_UNSOLVABLE,
  MPC_FAILED,
  AIM_ERROR_TOO_LARGE,
  AIM_NOT_STABLE,
  COOLDOWN,
};

[[nodiscard]] std::string_view FireRejectReasonName(FireRejectReason reason) noexcept;

/** @brief 最终交给云台命令后端的指令来源。 */
enum class GimbalCommandSource { MPC, TRAJECTORY_FALLBACK, STOP };

[[nodiscard]] std::string_view GimbalCommandSourceName(GimbalCommandSource source) noexcept;

/** @brief 指定装甲槽位的一次低抛物线弹道解。 */
struct BallisticSolution {
  bool valid{false};
  int slot{-1};
  geometry::Vector3 target_world{geometry::Vector3::Zero()};
  double yaw{0.0};
  double pitch{0.0};
  double distance_m{0.0};
  double fly_time_s{0.0};
  double prediction_horizon_s{0.0};  ///< 产生 target_world 的预测时域，单位为秒。
};

/** @brief 控制运行时与 HAL 消费的单周期正式火控输出。 */
struct FireControlOutput {
  std::uint64_t source_sequence{0};
  std::optional<std::uint64_t> source_capture_timestamp_ns;
  std::uint64_t command_timestamp_ns{0};
  TrackerState tracker_state{TrackerState::LOST};
  std::optional<ArmorLabel> tracked_label;
  std::optional<geometry::ArmorType> tracked_type;
  double prediction_age_s{0.0};
  double feedback_age_s{0.0};
  int selected_slot{-1};
  BallisticSolution ballistic;
  geometry::RigidTransform world_t_muzzle;
  bool muzzle_pose_valid{false};
  GimbalTrajectoryOutput plan;
  PlannedGimbalPoint raw_mpc_command;
  hal::GimbalCommand command;
  GimbalCommandSource command_source{GimbalCommandSource::STOP};
  bool raw_mpc_valid{false};
  bool published_valid{false};
  bool fallback_active{false};
  double fallback_age_s{0.0};
  int fallback_source_slot{-1};
  int fallback_trajectory_index{-1};
  int fallback_remaining_points{0};
  int consecutive_mpc_failure_cycles{0};
  bool tracking_object_reset{false};
  bool command_publish_succeeded{false};
  double target_yaw{0.0};
  double target_pitch{0.0};
  double fire_yaw_window{0.0};
  double fire_pitch_window{0.0};
  bool external_control_enabled{false};
  bool command_sink_healthy{false};
  bool auto_fire_enabled{false};
  std::uint64_t talos_heartbeat_ns{0};
  bool fire_eligible{false};
  int stable_cycles{0};
  FireRejectReason reject_reason{FireRejectReason::TRACK_NOT_CONFIRMED};
};

}  // namespace mv::modules
