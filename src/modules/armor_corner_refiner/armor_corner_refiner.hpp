#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>
#include <span>
#include <yaml-cpp/yaml.h>

namespace mv::modules {

enum class CornerRefinementMode : std::uint8_t {
  RAW = 0,
  PERCENTILE_PCA_SHADOW,
  GRADIENT_AXIS_SHADOW,
};

[[nodiscard]] const char* CornerRefinementModeName(CornerRefinementMode mode) noexcept;

struct ArmorCornerRefinerConfig {
  CornerRefinementMode mode{CornerRefinementMode::GRADIENT_AXIS_SHADOW};
  // percentile_pca_shadow baseline (schema v1 behavior).
  double axial_expansion{0.20};
  double lateral_expansion{0.50};
  int min_roi_area_px{24};
  int min_support_pixels{8};
  double color_quantile{0.70};
  double brightness_quantile{0.60};
  double min_color_difference{4.0};
  double min_brightness{24.0};
  int morphology_kernel{3};
  double min_axis_ratio{2.0};
  double max_center_offset_ratio{0.75};
  double max_parallel_angle_deg{45.0};
  double max_length_ratio{3.0};
  double endpoint_low_quantile{0.02};
  double endpoint_high_quantile{0.98};
  double endpoint_band_ratio{0.12};
  double max_corner_move_height_ratio{0.35};
  double min_polygon_area_px{4.0};
  // gradient_axis_shadow. This path deliberately has no color mask, morphology, connected
  // components, or percentile-derived endpoints.
  double gradient_axial_expansion{0.10};
  double gradient_lateral_half_width_ratio{0.18};
  double gradient_lateral_half_width_min_px{3.0};
  double gradient_lateral_half_width_max_px{12.0};
  double gradient_min_light_length_px{12.0};
  double gradient_short_light_threshold_px{30.0};
  double gradient_min_estimated_width_px{3.0};
  double gradient_max_axis_angle_deg{15.0};
  double gradient_max_center_offset_ratio{0.20};
  double gradient_search_start_ratio{0.40};
  double gradient_search_end_ratio{0.60};
  int gradient_short_scan_lines{3};
  int gradient_long_scan_lines{5};
  double gradient_sample_step_px{0.25};
  double gradient_smoothing_sigma_px{0.75};
  double gradient_min_strength_gray{6.0};
  double gradient_min_contrast_ratio{0.15};
  double gradient_min_inner_brightness_ratio{0.20};
  double gradient_max_secondary_peak_ratio{0.85};
  double gradient_max_profile_peak_spread_px{1.0};
  double gradient_max_corner_move_px{2.0};
};

[[nodiscard]] ArmorCornerRefinerConfig ParseArmorCornerRefinerConfig(const YAML::Node& root);

enum class CornerRefinementStatus : std::uint8_t {
  SUCCESS = 0,
  INVALID_IMAGE,
  INVALID_GEOMETRY,
  ROI_TOO_SMALL,
  LEFT_STRIP_NOT_FOUND,
  RIGHT_STRIP_NOT_FOUND,
  PCA_DEGENERATE,
  STRIP_PAIR_INVALID,
  CORNER_MOVE_TOO_LARGE,
  PNP_FALLBACK,
  MODE_RAW,
  INCOMPLETE_ENDPOINTS,
};

[[nodiscard]] const char* CornerRefinementStatusName(CornerRefinementStatus status) noexcept;

struct RefinedLightStrip {
  cv::Point2f center{};
  cv::Point2f axis{};
  cv::Point2f top{};
  cv::Point2f bottom{};
  std::size_t support_pixels{0};
  double confidence{0.0};
  double estimated_length_px{0.0};
  double estimated_width_px{0.0};
  double axis_ratio{0.0};
  double axis_deviation_deg{0.0};
  double center_offset_px{0.0};
  double background_brightness{0.0};
  double contrast{0.0};
};

enum class EndpointRefinementStatus : std::uint8_t {
  APPLIED = 0,
  MODE_RAW,
  LIGHT_TOO_SHORT,
  ROI_TOO_SMALL,
  NO_BRIGHTNESS_SUPPORT,
  PCA_DEGENERATE,
  LIGHT_TOO_NARROW,
  AXIS_RATIO_TOO_LOW,
  AXIS_DEVIATION_TOO_LARGE,
  CENTER_OFFSET_TOO_LARGE,
  GRADIENT_TOO_WEAK,
  BRIGHT_SIDE_TOO_DARK,
  INSUFFICIENT_SCAN_LINES,
  PEAK_AMBIGUOUS,
  PROFILE_UNSTABLE,
  MOVE_TOO_LARGE,
};

[[nodiscard]] const char* EndpointRefinementStatusName(EndpointRefinementStatus status) noexcept;

enum class EndpointRevertedBy : std::uint8_t {
  NONE = 0,
  ARMOR_ATOMIC,
  GEOMETRY,
  PNP,
};

[[nodiscard]] const char* EndpointRevertedByName(EndpointRevertedBy reason) noexcept;

struct EndpointRefinementDiagnostic {
  bool applied{false};
  bool fallback{true};
  bool candidate_valid{false};
  EndpointRefinementStatus status{EndpointRefinementStatus::MODE_RAW};
  EndpointRevertedBy reverted_by{EndpointRevertedBy::NONE};
  double gradient_strength{0.0};
  double secondary_gradient_strength{0.0};
  double secondary_peak_ratio{0.0};
  double inner_brightness{0.0};
  double bright_side_threshold{0.0};
  std::size_t valid_scan_lines{0};
  double profile_peak_spread_px{0.0};
  cv::Point2f original{};
  cv::Point2f candidate{};
  cv::Point2f final{};
  double movement_px{0.0};
  double requested_movement_px{0.0};
  cv::Point2f search_start{};
  cv::Point2f search_end{};
  std::array<cv::Point2f, 5> scan_candidates{};
  std::array<bool, 5> scan_candidate_present{};
  std::array<bool, 5> scan_candidate_valid{};
};

struct CornerRefinementResult {
  CornerRefinementMode mode{CornerRefinementMode::GRADIENT_AXIS_SHADOW};
  std::array<cv::Point2f, 4> original_corners{};
  std::array<cv::Point2f, 4> refined_corners{};
  std::array<cv::Point2f, 4> corner_displacements{};
  std::array<RefinedLightStrip, 2> strips{};
  std::array<EndpointRefinementDiagnostic, 4> endpoints{};
  CornerRefinementStatus status{CornerRefinementStatus::INVALID_GEOMETRY};
  bool success{false};
  bool fallback{true};
  double confidence{0.0};
  double elapsed_ms{0.0};
};

class ArmorCornerRefiner final {
 public:
  explicit ArmorCornerRefiner(ArmorCornerRefinerConfig config);

  [[nodiscard]] CornerRefinementResult Refine(const cv::Mat& bgr_image,
                                              std::span<const cv::Point2f, 4> corners,
                                              ArmorColor enemy_color,
                                              hal::CameraFrame::ArmorType armor_type) const;

 private:
  [[nodiscard]] CornerRefinementResult RefinePercentile(const cv::Mat& bgr_image,
                                                        std::span<const cv::Point2f, 4> corners,
                                                        ArmorColor enemy_color) const;
  [[nodiscard]] CornerRefinementResult RefineGradient(
      const cv::Mat& bgr_image, std::span<const cv::Point2f, 4> corners) const;
  ArmorCornerRefinerConfig config_;
};

}  // namespace mv::modules
