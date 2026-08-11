#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <optional>
#include <span>
#include <yaml-cpp/yaml.h>

namespace mv::modules {

struct ArmorPnpConfig {
  double small_width_m{0.135};
  double large_width_m{0.225};
  double height_m{0.055};
  double min_distance_m{0.1};
  double max_distance_m{30.0};
  double truth_match_min_iou{0.05};
  double truth_match_max_center_distance_ratio{0.75};
  double truth_match_max_corner_distance_ratio{0.75};
};

[[nodiscard]] ArmorPnpConfig ParseArmorPnpConfig(const YAML::Node& root);

enum class PnpInputSource : std::uint8_t {
  GROUND_TRUTH = 0,
  DETECTION_RAW = 1,
  DETECTION_REFINED = 2,
};
[[nodiscard]] const char* PnpInputSourceName(PnpInputSource source) noexcept;
enum class PnpStatus : std::uint8_t {
  SUCCESS = 0,
  INVALID_INPUT,
  NO_SOLUTION,
  NEGATIVE_DEPTH,
  BACK_FACING,
  OUT_OF_RANGE,
};

[[nodiscard]] const char* PnpStatusName(PnpStatus status) noexcept;

struct ArmorPoseEstimate {
  PnpInputSource source{PnpInputSource::DETECTION_RAW};
  std::size_t input_index{0};
  std::optional<std::uint64_t> truth_id;
  std::uint8_t label{0};
  hal::CameraFrame::ArmorType type{hal::CameraFrame::ArmorType::SMALL};
  double width_m{0.0};
  double height_m{0.0};
  geometry::RigidTransform camera_t_armor;
  std::array<cv::Point2f, 4> image_corners{};
  std::array<cv::Point2f, 4> reprojected_corners{};
  std::size_t candidate_index{0};
  std::optional<double> candidate_rmse_gap_px;
  double reprojection_rmse_px{0.0};
  double image_width_px{0.0};
  double image_height_px{0.0};
  double distance_m{0.0};
  double viewing_angle_deg{0.0};
  std::optional<double> truth_distance_m;
  std::optional<double> truth_viewing_angle_deg;
  std::optional<hal::CameraFrame::ArmorType> truth_type;
  std::optional<double> mean_corner_error_px;
  std::array<double, 4> corner_errors_px{};
  std::array<double, 4> corner_delta_u_px{};
  std::array<double, 4> corner_delta_v_px{};
  std::optional<double> position_error_m;
  std::optional<std::array<double, 3>> position_error_camera_m;
  std::optional<double> depth_error_m;
  std::optional<double> signed_depth_error_m;
  std::optional<double> rotation_error_deg;
  std::optional<double> position_jitter_m;
};

struct ArmorPnpAttempt {
  PnpInputSource source{PnpInputSource::DETECTION_RAW};
  std::size_t input_index{0};
  PnpStatus status{PnpStatus::INVALID_INPUT};
  std::optional<ArmorPoseEstimate> estimate;
  std::optional<CornerRefinementResult> refinement;
};

struct PnpPercentiles {
  std::size_t samples{0};
  double p50{0.0};
  double p95{0.0};
};

struct PnpSourceSummary {
  PnpPercentiles reprojection_rmse_px;
  PnpPercentiles mean_corner_error_px;
  PnpPercentiles position_error_m;
  PnpPercentiles depth_error_m;
  PnpPercentiles rotation_error_deg;
  PnpPercentiles position_jitter_m;
};

struct CornerRefinementSummary {
  std::size_t attempted{0};
  std::size_t succeeded{0};
  std::size_t fallback{0};
  std::size_t fully_refined{0};
  std::size_t full_fallback{0};
  std::map<std::string, std::size_t> failure_reasons;
  PnpPercentiles elapsed_ms;
};

struct PnpSolveSummary {
  std::size_t attempted{0};
  std::size_t succeeded{0};
  std::size_t candidate_switches{0};
  std::map<std::string, std::size_t> rejection_reasons;
};

struct ArmorPnpFrameResult {
  std::uint64_t summary_sequence{0};
  std::vector<ArmorPnpAttempt> attempts;
  PnpSourceSummary ground_truth_summary;
  PnpSourceSummary detection_raw_summary;
  PnpSourceSummary detection_refined_success_summary;
  PnpSourceSummary detection_refined_with_fallback_summary;
  std::map<std::string, PnpSourceSummary> raw_distance_groups;
  std::map<std::string, PnpSourceSummary> refined_success_distance_groups;
  std::map<std::string, PnpSourceSummary> refined_with_fallback_distance_groups;
  std::map<std::string, PnpSourceSummary> raw_angle_groups;
  std::map<std::string, PnpSourceSummary> refined_success_angle_groups;
  std::map<std::string, PnpSourceSummary> refined_with_fallback_angle_groups;
  std::map<std::string, PnpSourceSummary> raw_size_groups;
  std::map<std::string, PnpSourceSummary> refined_success_size_groups;
  std::map<std::string, PnpSourceSummary> refined_with_fallback_size_groups;
  PnpSolveSummary raw_solve_summary;
  PnpSolveSummary refined_solve_summary;
  CornerRefinementSummary refinement_summary;
};

struct PnpMetricSamples {
  std::vector<double> reprojection;
  std::vector<double> corner;
  std::vector<double> position;
  std::vector<double> depth;
  std::vector<double> rotation;
  std::vector<double> jitter;
};

class ArmorPnp final {
 public:
  explicit ArmorPnp(ArmorPnpConfig config);

  [[nodiscard]] ArmorPnpAttempt Solve(std::span<const cv::Point2f, 4> image_corners,
                                      hal::CameraFrame::ArmorType type,
                                      const hal::CameraFrame::Calibration& calibration,
                                      PnpInputSource source, std::size_t input_index,
                                      std::uint8_t label = 0) const;

  [[nodiscard]] ArmorPnpFrameResult ProcessFrame(
      const hal::CameraFrame& frame, std::span<const ArmorDetection> detections,
      std::span<const CornerRefinementResult> refinements);

 private:
  ArmorPnpConfig config_;
  PnpMetricSamples truth_samples_;
  PnpMetricSamples raw_samples_;
  PnpMetricSamples refined_success_samples_;
  PnpMetricSamples refined_with_fallback_samples_;
  std::map<std::string, PnpMetricSamples> raw_distance_samples_;
  std::map<std::string, PnpMetricSamples> refined_success_distance_samples_;
  std::map<std::string, PnpMetricSamples> refined_with_fallback_distance_samples_;
  std::map<std::string, PnpMetricSamples> raw_angle_samples_;
  std::map<std::string, PnpMetricSamples> refined_success_angle_samples_;
  std::map<std::string, PnpMetricSamples> refined_with_fallback_angle_samples_;
  std::map<std::string, PnpMetricSamples> raw_size_samples_;
  std::map<std::string, PnpMetricSamples> refined_success_size_samples_;
  std::map<std::string, PnpMetricSamples> refined_with_fallback_size_samples_;
  std::map<std::pair<PnpInputSource, std::uint64_t>, geometry::Vector3> previous_error_;
  std::map<std::pair<PnpInputSource, std::uint64_t>, std::size_t> previous_candidate_;
  std::map<std::pair<PnpInputSource, std::uint64_t>, std::uint64_t> previous_sequence_;
  PnpSolveSummary raw_solve_summary_;
  PnpSolveSummary refined_solve_summary_;
  CornerRefinementSummary refinement_summary_;
  PnpSolveSummary raw_solve_snapshot_;
  PnpSolveSummary refined_solve_snapshot_;
  CornerRefinementSummary refinement_snapshot_;
  std::vector<double> refinement_elapsed_samples_;
  PnpSourceSummary truth_summary_;
  PnpSourceSummary raw_summary_;
  PnpSourceSummary refined_success_summary_;
  PnpSourceSummary refined_with_fallback_summary_;
  std::map<std::string, PnpSourceSummary> raw_distance_summaries_;
  std::map<std::string, PnpSourceSummary> refined_success_distance_summaries_;
  std::map<std::string, PnpSourceSummary> refined_with_fallback_distance_summaries_;
  std::map<std::string, PnpSourceSummary> raw_angle_summaries_;
  std::map<std::string, PnpSourceSummary> refined_success_angle_summaries_;
  std::map<std::string, PnpSourceSummary> refined_with_fallback_angle_summaries_;
  std::map<std::string, PnpSourceSummary> raw_size_summaries_;
  std::map<std::string, PnpSourceSummary> refined_success_size_summaries_;
  std::map<std::string, PnpSourceSummary> refined_with_fallback_size_summaries_;
  std::uint64_t summary_sequence_{0};
  bool summary_initialized_{false};
};

[[nodiscard]] hal::CameraFrame::ArmorType ArmorTypeForLabel(ArmorLabel label) noexcept;

}  // namespace mv::modules
