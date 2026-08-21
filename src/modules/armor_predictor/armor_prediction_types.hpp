#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <optional>

namespace mv::modules {

enum class TrackerState : std::uint8_t { LOST = 0, DETECTING, TRACKING, TEMP_LOST };

[[nodiscard]] const char* TrackerStateName(TrackerState state) noexcept;

/** @brief 单个二维装甲候选与预测槽位的像素关联诊断。 */
struct ArmorAssociation {
  std::size_t input_index{0};
  int slot{-1};            ///< 通过像素门控并进入本帧试更新的槽位。
  int candidate_slot{-1};  ///< 无论是否通过门控，代价最小的可见候选槽位。
  bool accepted{false};    ///< 本关联是否随通过 NIS 门控的后验正式提交。
  double gate{0.0};        ///< 本次关联实际使用的像素组合代价门限。
  double center_error_px{0.0};
  double edge_angle_error_rad{0.0};
  double perimeter_ratio_error{0.0};
  double total_cost{0.0};
  std::array<cv::Point2f, 4> observed_corners{};
  std::array<cv::Point2f, 4> predicted_corners{};
  std::string rejection_reason;
};

/** @brief 单根独立灯条与预测 `(slot,left/right)` 身份的关联诊断。 */
struct LightbarAssociation {
  std::size_t input_index{0};
  int slot{-1};
  int candidate_slot{-1};
  bool left{true};
  bool candidate_left{true};
  bool accepted{false};
  bool duplicate_full_armor{false};
  double center_error_px{0.0};
  double endpoint_distance_ratio{0.0};
  double angle_error_rad{0.0};
  double log_length_error{0.0};
  double total_cost{0.0};
  cv::Point2f observed_top{};
  cv::Point2f observed_bottom{};
  cv::Point2f predicted_top{};
  cv::Point2f predicted_bottom{};
  std::string rejection_reason;
};

struct PredictedArmorPose {
  int slot{0};
  geometry::RigidTransform world_t_armor;
};

/** @brief 某一未来时刻的完整车体姿态和四块装甲预测。 */
struct PredictionHorizon {
  double seconds{0.0};
  geometry::Vector3 center_world{geometry::Vector3::Zero()};
  geometry::Quaternion orientation_world{geometry::Quaternion::Identity()};
  double yaw{0.0};
  std::array<PredictedArmorPose, 4> armors{};
};

/** @brief 单帧13维 ESEKF 状态、具名物理量及图像更新诊断。 */
struct ArmorPredictionResult {
  std::uint64_t sequence{0};
  std::optional<std::uint64_t> source_capture_timestamp_ns;
  std::chrono::steady_clock::time_point source_receive_steady_time{};
  TrackerState state{TrackerState::LOST};
  std::optional<ArmorLabel> label;
  std::optional<hal::CameraFrame::ArmorType> type;
  double dt_s{0.0};
  /** @brief cx,vx,cy,vy,cz,vz,rot_x,rot_y,rot_z,vyaw,log_r1,log_r2,h。 */
  std::array<double, 13> state_vector{};
  std::array<double, 13> covariance_diagonal{};
  geometry::Vector3 center_world{geometry::Vector3::Zero()};
  geometry::Vector3 velocity_world{geometry::Vector3::Zero()};
  geometry::Quaternion orientation_world{geometry::Quaternion::Identity()};
  double yaw_velocity_rad_s{0.0};
  std::array<double, 2> radii_m{};
  double height_offset_m{0.0};
  double armor_tilt_rad{0.0};
  Eigen::Matrix3d center_covariance_world{Eigen::Matrix3d::Zero()};
  double yaw_variance_rad2{0.0};
  std::vector<ArmorAssociation> associations;
  std::vector<LightbarAssociation> lightbar_associations;
  std::vector<double> innovation;
  std::optional<double> nis;
  std::optional<double> nis_per_dof;
  int esekf_iterations{0};
  double estimation_elapsed_ms{0.0};
  std::optional<double> truth_center_error_m;
  std::optional<double> truth_yaw_error_rad;
  std::optional<double> truth_yaw_equivalent_error_rad;
  std::optional<double> truth_yaw_velocity_error_rad_s;
  bool maneuver_active{false};
  std::string maneuver_phase{"idle"};
  std::string maneuver_trigger;
  int maneuver_evidence_frames{0};
  double maneuver_evidence_cost{0.0};
  double maneuver_confirmation_remaining_s{0.0};
  double maneuver_remaining_s{0.0};
  double yaw_process_variance_used{0.0};
  std::optional<double> trial_yaw_velocity_update_rad_s;
  double association_gate_used{0.0};
  int accepted_association_count{0};
  int rejected_association_count{0};
  int detected_lightbar_count{0};
  int deduplicated_lightbar_count{0};
  int matched_lightbar_count{0};
  int accepted_lightbar_count{0};
  int rejected_lightbar_count{0};
  int light_only_pair_count{0};  ///< 无完整装甲时，同槽左右灯条完整配对数量。
  bool light_only_update{false};
  bool light_only_update_blocked{false};
  std::string light_only_rejection_reason;
  bool light_fusion_used{false};
  bool armor_fallback_used{false};
  std::uint64_t reset_count{0};
  std::vector<PredictionHorizon> horizons;
  std::string reset_reason;  ///< 最近一次安全重置原因；结合 reset_count 判断是否为新事件。
};

/** @brief 从具名名义状态按匀速和车体系 z 轴匀角速模型外推。 */
[[nodiscard]] PredictionHorizon ExtrapolatePrediction(const ArmorPredictionResult& prediction,
                                                      double seconds);

}  // namespace mv::modules
