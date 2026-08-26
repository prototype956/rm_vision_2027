#pragma once

#include "modules/armor_detector/armor_detector_output.hpp"
#include "modules/armor_pnp/armor_pnp_output.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <optional>

namespace mv::modules {

enum class PnpStatus : std::uint8_t {
  SUCCESS = 0,
  INVALID_INPUT,
  NO_SOLUTION,
  NEGATIVE_DEPTH,
  BACK_FACING,
  OUT_OF_RANGE,
};

[[nodiscard]] const char* PnpStatusName(PnpStatus status) noexcept;

/** @brief 一次成功位姿的候选选择和重投影诊断。 */
struct ArmorPoseDiagnostics {
  std::size_t input_index{0};
  std::array<cv::Point2f, 4> image_corners{};
  std::array<cv::Point2f, 4> reprojected_corners{};
  std::size_t candidate_index{0};
  std::optional<double> candidate_rmse_gap_px;
  double reprojection_rmse_px{0.0};
  double image_width_px{0.0};
  double image_height_px{0.0};
  double distance_m{0.0};
  double viewing_angle_deg{0.0};
};

struct ArmorPnpAttemptDiagnostics {
  std::size_t input_index{0};
  PnpStatus status{PnpStatus::INVALID_INPUT};
  std::optional<ArmorPoseDiagnostics> pose;
};

struct PnpPercentiles {
  std::size_t samples{0};
  double p50{0.0};
  double p95{0.0};
};

struct PnpDetectionSummary {
  PnpPercentiles reprojection_rmse_px;
};

struct CornerRefinementSummary {
  std::size_t attempted{0};
  std::size_t succeeded{0};
  std::size_t fallback{0};
  std::map<std::string, std::size_t> failure_reasons;
  PnpPercentiles elapsed_ms;
};

struct PnpSolveSummary {
  std::size_t attempted{0};
  std::size_t succeeded{0};
  std::map<std::string, std::size_t> rejection_reasons;
};

struct ArmorPnpDiagnostics {
  std::uint64_t summary_sequence{0};
  std::vector<ArmorPnpAttemptDiagnostics> attempts;
  PnpDetectionSummary detection_summary;
  PnpSolveSummary solve_summary;
  CornerRefinementSummary refinement_summary;
};

struct ArmorPnpResult {
  ArmorPnpOutput output;
  ArmorPnpDiagnostics diagnostics;
};

struct ArmorPnpSolveResult {
  std::optional<ArmorPoseEstimate> output;
  ArmorPnpAttemptDiagnostics diagnostics;
};

}  // namespace mv::modules
