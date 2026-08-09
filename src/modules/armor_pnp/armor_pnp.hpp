#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
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
};

[[nodiscard]] ArmorPnpConfig ParseArmorPnpConfig(const YAML::Node& root);

enum class PnpInputSource : std::uint8_t { GROUND_TRUTH = 0, DETECTION = 1 };
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
  PnpInputSource source{PnpInputSource::DETECTION};
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
  double reprojection_rmse_px{0.0};
  double distance_m{0.0};
  double viewing_angle_deg{0.0};
  std::optional<double> mean_corner_error_px;
  std::array<double, 4> corner_errors_px{};
  std::optional<double> position_error_m;
  std::optional<double> depth_error_m;
  std::optional<double> rotation_error_deg;
};

struct ArmorPnpAttempt {
  PnpInputSource source{PnpInputSource::DETECTION};
  std::size_t input_index{0};
  PnpStatus status{PnpStatus::INVALID_INPUT};
  std::optional<ArmorPoseEstimate> estimate;
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
};

struct ArmorPnpFrameResult {
  std::vector<ArmorPnpAttempt> attempts;
  PnpSourceSummary ground_truth_summary;
  PnpSourceSummary detection_summary;
};

struct PnpMetricSamples {
  std::vector<double> reprojection;
  std::vector<double> corner;
  std::vector<double> position;
  std::vector<double> depth;
  std::vector<double> rotation;
};

class ArmorPnp final {
 public:
  explicit ArmorPnp(ArmorPnpConfig config);

  [[nodiscard]] ArmorPnpAttempt Solve(std::span<const cv::Point2f, 4> image_corners,
                                      hal::CameraFrame::ArmorType type,
                                      const hal::CameraFrame::Calibration& calibration,
                                      PnpInputSource source, std::size_t input_index,
                                      std::uint8_t label = 0) const;

  [[nodiscard]] ArmorPnpFrameResult ProcessFrame(const hal::CameraFrame& frame,
                                                 std::span<const ArmorDetection> detections);

 private:
  ArmorPnpConfig config_;
  PnpMetricSamples truth_samples_;
  PnpMetricSamples detection_samples_;
  PnpSourceSummary truth_summary_;
  PnpSourceSummary detection_summary_;
};

[[nodiscard]] hal::CameraFrame::ArmorType ArmorTypeForLabel(ArmorLabel label) noexcept;

}  // namespace mv::modules
