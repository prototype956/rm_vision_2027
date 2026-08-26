#pragma once

#include "frame/frame_types.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"
#include "simulation/simulation_frame_data.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <optional>
#include <span>

namespace mv::tool::simulation_evaluation {

enum class PnpEvaluationSource : std::uint8_t { GROUND_TRUTH = 0, DETECTION = 1 };

[[nodiscard]] const char* PnpEvaluationSourceName(PnpEvaluationSource source) noexcept;

/** @brief 保留正式估计并附加同帧仿真真值误差的展示与统计 DTO。 */
struct EvaluatedArmorPose {
  PnpEvaluationSource source{PnpEvaluationSource::DETECTION};
  std::size_t input_index{0};
  std::optional<std::uint64_t> truth_id;
  std::uint8_t label{0};
  geometry::ArmorType type{geometry::ArmorType::SMALL};
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
  std::optional<geometry::ArmorType> truth_type;
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

struct PnpEvaluationAttempt {
  PnpEvaluationSource source{PnpEvaluationSource::DETECTION};
  std::size_t input_index{0};
  modules::PnpStatus status{modules::PnpStatus::INVALID_INPUT};
  std::optional<EvaluatedArmorPose> estimate;
  std::optional<modules::CornerRefinementDiagnostics> refinement;
};

struct EvaluationPercentiles {
  std::size_t samples{0};
  double p50{0.0};
  double p95{0.0};
};

struct PnpSourceSummary {
  EvaluationPercentiles reprojection_rmse_px;
  EvaluationPercentiles mean_corner_error_px;
  EvaluationPercentiles position_error_m;
  EvaluationPercentiles depth_error_m;
  EvaluationPercentiles rotation_error_deg;
  EvaluationPercentiles position_jitter_m;
};

struct PnpSolveSummary {
  std::size_t attempted{0};
  std::size_t succeeded{0};
  std::size_t candidate_switches{0};
  std::map<std::string, std::size_t> rejection_reasons;
};

struct CornerRefinementSummary {
  std::size_t attempted{0};
  std::size_t succeeded{0};
  std::size_t fallback{0};
  std::map<std::string, std::size_t> failure_reasons;
  EvaluationPercentiles elapsed_ms;
  EvaluationPercentiles raw_mean_corner_error_px;
  EvaluationPercentiles final_mean_corner_error_px;
};

/** @brief 与旧 Foxglove PnP Schema 一一对应的独立仿真评估结果。 */
struct PnpEvaluationResult {
  std::uint64_t summary_sequence{0};
  std::vector<PnpEvaluationAttempt> attempts;
  PnpSourceSummary ground_truth_summary;
  PnpSourceSummary detection_summary;
  std::map<std::string, PnpSourceSummary> distance_groups;
  std::map<std::string, PnpSourceSummary> angle_groups;
  std::map<std::string, PnpSourceSummary> size_groups;
  PnpSolveSummary solve_summary;
  CornerRefinementSummary refinement_summary;
};

struct PredictionEvaluationResult {
  std::optional<std::uint64_t> truth_target_id;
  std::optional<geometry::Vector3> truth_position_world;
  std::optional<double> truth_yaw_velocity_rad_s;
  std::optional<double> center_error_m;
  std::optional<double> yaw_error_rad;
  std::optional<double> yaw_equivalent_error_rad;
  std::optional<double> yaw_velocity_error_rad_s;
};

struct SimulationEvaluationInput {
  const frame::FrameStamp& stamp;
  const frame::CameraModel& camera_model;
  const frame::FrameKinematics& kinematics;
  const simulation::SimulationFrameData& simulation;
  std::span<const modules::ArmorDetection> detections;
  std::span<const modules::CornerRefinementOutput> refinements;
  std::span<const modules::CornerRefinementDiagnostics> refinement_diagnostics;
  const modules::ArmorPnpOutput& pnp;
  const modules::ArmorPnpDiagnostics& pnp_diagnostics;
  const modules::ArmorPredictionOutput& prediction;
};

struct SimulationEvaluationResult {
  std::uint64_t sequence{0};
  PnpEvaluationResult pnp;
  PredictionEvaluationResult prediction;
  double elapsed_ms{0.0};
};

/** @brief 在没有仿真评估时将正式 PnP 健康数据提升为兼容诊断结果。 */
[[nodiscard]] PnpEvaluationResult MakePnpDiagnosticResult(
    const modules::ArmorPnpOutput& output, const modules::ArmorPnpDiagnostics& diagnostics,
    std::span<const modules::CornerRefinementDiagnostics> refinements);

}  // namespace mv::tool::simulation_evaluation
