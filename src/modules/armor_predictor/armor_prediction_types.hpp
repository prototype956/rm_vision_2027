#pragma once

#include "modules/armor_predictor/armor_prediction_output.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <optional>

namespace mv::modules {

struct ArmorAssociation {
  std::size_t input_index{0};
  int slot{-1};
  int candidate_slot{-1};
  bool accepted{false};
  double gate{0.0};
  double center_error_px{0.0};
  double edge_angle_error_rad{0.0};
  double perimeter_ratio_error{0.0};
  double total_cost{0.0};
  std::array<cv::Point2f, 4> observed_corners{};
  std::array<cv::Point2f, 4> predicted_corners{};
  std::string rejection_reason;
};

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

/** @brief 单帧预测过程的关联、滤波、机动和健康诊断。 */
struct ArmorPredictionDiagnostics {
  double dt_s{0.0};
  std::vector<ArmorAssociation> associations;
  std::vector<LightbarAssociation> lightbar_associations;
  std::vector<double> innovation;
  std::optional<double> nis;
  std::optional<double> nis_per_dof;
  int esekf_iterations{0};
  double estimation_elapsed_ms{0.0};
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
  int light_only_pair_count{0};
  bool light_only_update{false};
  bool light_only_update_blocked{false};
  std::string light_only_rejection_reason;
  bool light_fusion_used{false};
  bool armor_fallback_used{false};
  std::uint64_t reset_count{0};
  std::string reset_reason;
};

struct ArmorPredictionResult {
  ArmorPredictionOutput output;
  ArmorPredictionDiagnostics diagnostics;
};

}  // namespace mv::modules
