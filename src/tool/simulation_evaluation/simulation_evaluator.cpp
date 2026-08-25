#include "tool/simulation_evaluation/simulation_evaluator.hpp"

#include "modules/armor_pnp/armor_pnp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <numbers>
#include <opencv2/imgproc.hpp>

namespace mv::tool::simulation_evaluation {
namespace {

constexpr double K_RAD_TO_DEG = 180.0 / std::numbers::pi;

struct VisibleArmorTruth {
  const simulation::GroundTruthArmor* armor{nullptr};
  std::array<cv::Point2f, 4> pixels{};
};

struct MetricSamples {
  std::vector<double> reprojection;
  std::vector<double> corner;
  std::vector<double> position;
  std::vector<double> depth;
  std::vector<double> rotation;
  std::vector<double> jitter;
};

EvaluationPercentiles Percentiles(const std::vector<double>& samples) {
  if (samples.empty())
    return {};
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  const auto AT = [&](double fraction) {
    const auto INDEX =
        static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(sorted.size())) - 1.0);
    return sorted[std::min(INDEX, sorted.size() - 1)];
  };
  return {.samples = sorted.size(), .p50 = AT(0.50), .p95 = AT(0.95)};
}

PnpSourceSummary Summarize(const MetricSamples& samples) {
  return {.reprojection_rmse_px = Percentiles(samples.reprojection),
          .mean_corner_error_px = Percentiles(samples.corner),
          .position_error_m = Percentiles(samples.position),
          .depth_error_m = Percentiles(samples.depth),
          .rotation_error_deg = Percentiles(samples.rotation),
          .position_jitter_m = Percentiles(samples.jitter)};
}

std::map<std::string, PnpSourceSummary> SummarizeGroups(
    const std::map<std::string, MetricSamples>& groups) {
  std::map<std::string, PnpSourceSummary> result;
  for (const auto& [name, samples] : groups)
    result.emplace(name, Summarize(samples));
  return result;
}

std::string DistanceGroup(double distance) {
  if (distance < 3.0)
    return "lt_3m";
  if (distance < 5.0)
    return "3_5m";
  if (distance < 7.0)
    return "5_7m";
  if (distance < 9.0)
    return "7_9m";
  return "ge_9m";
}

std::string AngleGroup(double angle) {
  if (angle < 10.0)
    return "lt_10deg";
  if (angle < 20.0)
    return "10_20deg";
  if (angle < 30.0)
    return "20_30deg";
  if (angle < 45.0)
    return "30_45deg";
  return "ge_45deg";
}

std::string SizeGroup(geometry::ArmorType type) {
  return type == geometry::ArmorType::LARGE ? "large" : "small";
}

bool LabelsMatch(modules::ArmorLabel detection, std::uint8_t truth) {
  if (detection == modules::ArmorLabel::BASE_SMALL || detection == modules::ArmorLabel::BASE_BIG)
    return truth == 7;
  return static_cast<std::uint8_t>(detection) == truth;
}

std::uint8_t TeamForColor(modules::ArmorColor color) {
  return color == modules::ArmorColor::RED ? 0 : 1;
}

double PolygonIntersection(const std::array<cv::Point2f, 4>& left,
                           const std::array<cv::Point2f, 4>& right) {
  std::vector<cv::Point2f> intersection;
  return cv::intersectConvexConvex(std::vector<cv::Point2f>(left.begin(), left.end()),
                                   std::vector<cv::Point2f>(right.begin(), right.end()),
                                   intersection);
}

double PolygonArea(const std::array<cv::Point2f, 4>& polygon) {
  return std::abs(cv::contourArea(std::vector<cv::Point2f>(polygon.begin(), polygon.end())));
}

cv::Point2f PolygonCenter(const std::array<cv::Point2f, 4>& polygon) {
  cv::Point2f center{};
  for (const auto& point : polygon)
    center += point;
  return center * 0.25F;
}

double PolygonDiagonal(const std::array<cv::Point2f, 4>& polygon) {
  const auto BOUNDS = cv::boundingRect(std::vector<cv::Point2f>(polygon.begin(), polygon.end()));
  return std::hypot(static_cast<double>(BOUNDS.width), static_cast<double>(BOUNDS.height));
}

std::optional<std::array<cv::Point2f, 4>> ProjectVisibleTruth(
    const simulation::GroundTruthArmor& armor, const frame::CameraModel& camera_model,
    const frame::FrameKinematics& kinematics) {
  const auto WORLD_T_CAMERA =
      geometry::Compose(kinematics.world_t_gimbal, kinematics.gimbal_t_camera_optical);
  const auto CAMERA_T_WORLD = geometry::Inverse(WORLD_T_CAMERA);
  const auto CAMERA_T_ARMOR = geometry::Compose(CAMERA_T_WORLD, armor.world_t_armor);
  const auto NORMAL_CAMERA = geometry::TransformVector(CAMERA_T_ARMOR, geometry::Vector3::UnitZ());
  if (NORMAL_CAMERA.dot(CAMERA_T_ARMOR.translation) >= 0.0)
    return std::nullopt;
  std::array<cv::Point2f, 4> projected{};
  for (std::size_t index = 0; index < projected.size(); ++index) {
    const auto POINT = geometry::TransformPoint(CAMERA_T_WORLD, armor.corners_world[index]);
    if (POINT.z() <= 0.0)
      return std::nullopt;
    projected[index] =
        cv::Point2f(static_cast<float>(camera_model.fx * POINT.x() / POINT.z() + camera_model.cx),
                    static_cast<float>(camera_model.fy * POINT.y() / POINT.z() + camera_model.cy));
  }
  const auto BOUNDS =
      cv::boundingRect(std::vector<cv::Point2f>(projected.begin(), projected.end()));
  const cv::Rect IMAGE_BOUNDS(0, 0, static_cast<int>(camera_model.width),
                              static_cast<int>(camera_model.height));
  if ((BOUNDS & IMAGE_BOUNDS).empty())
    return std::nullopt;
  return projected;
}

std::vector<std::size_t> MatchDetectionsToTruth(std::span<const modules::ArmorDetection> detections,
                                                std::span<const VisibleArmorTruth> truths,
                                                const SimulationEvaluationConfig& config) {
  constexpr double FORBIDDEN_COST = 1.0e6;
  constexpr double UNMATCHED_COST = 10.0;
  const std::size_t ROWS = detections.size();
  const std::size_t TRUTH_COLUMNS = truths.size();
  const std::size_t COLUMNS = TRUTH_COLUMNS + ROWS;
  std::vector<std::size_t> matches(ROWS, TRUTH_COLUMNS);
  if (ROWS == 0 || TRUTH_COLUMNS == 0)
    return matches;
  std::vector<std::vector<double>> costs(ROWS, std::vector<double>(COLUMNS, UNMATCHED_COST));
  for (std::size_t row = 0; row < ROWS; ++row) {
    const auto& detection = detections[row];
    const double DETECTION_AREA = PolygonArea(detection.corners);
    const double DETECTION_DIAGONAL = PolygonDiagonal(detection.corners);
    for (std::size_t column = 0; column < TRUTH_COLUMNS; ++column) {
      const auto& truth = truths[column];
      if (truth.armor->team != TeamForColor(detection.color) ||
          !LabelsMatch(detection.label, truth.armor->label)) {
        costs[row][column] = FORBIDDEN_COST;
        continue;
      }
      const double TRUTH_AREA = PolygonArea(truth.pixels);
      const double INTERSECTION = PolygonIntersection(detection.corners, truth.pixels);
      const double UNION_AREA = DETECTION_AREA + TRUTH_AREA - INTERSECTION;
      const double IOU = UNION_AREA > 1.0e-6 ? INTERSECTION / UNION_AREA : 0.0;
      const double SCALE = std::max({1.0, DETECTION_DIAGONAL, PolygonDiagonal(truth.pixels)});
      const double CENTER_RATIO =
          cv::norm(PolygonCenter(detection.corners) - PolygonCenter(truth.pixels)) / SCALE;
      double mean_corner_distance = 0.0;
      for (std::size_t corner = 0; corner < 4; ++corner)
        mean_corner_distance += cv::norm(detection.corners[corner] - truth.pixels[corner]);
      const double CORNER_RATIO = mean_corner_distance / (4.0 * SCALE);
      if (IOU < config.truth_match_min_iou ||
          CENTER_RATIO > config.truth_match_max_center_distance_ratio ||
          CORNER_RATIO > config.truth_match_max_corner_distance_ratio) {
        costs[row][column] = FORBIDDEN_COST;
        continue;
      }
      costs[row][column] = 2.0 * (1.0 - IOU) + CENTER_RATIO + CORNER_RATIO;
    }
  }
  std::vector<double> u(ROWS + 1), v(COLUMNS + 1);
  std::vector<std::size_t> p(COLUMNS + 1), way(COLUMNS + 1);
  for (std::size_t row = 1; row <= ROWS; ++row) {
    p[0] = row;
    std::size_t column0 = 0;
    std::vector<double> min_value(COLUMNS + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(COLUMNS + 1, false);
    do {
      used[column0] = true;
      const std::size_t ROW0 = p[column0];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t column1 = 0;
      for (std::size_t column = 1; column <= COLUMNS; ++column) {
        if (used[column])
          continue;
        const double CURRENT = costs[ROW0 - 1][column - 1] - u[ROW0] - v[column];
        if (CURRENT < min_value[column]) {
          min_value[column] = CURRENT;
          way[column] = column0;
        }
        if (min_value[column] < delta) {
          delta = min_value[column];
          column1 = column;
        }
      }
      for (std::size_t column = 0; column <= COLUMNS; ++column) {
        if (used[column]) {
          u[p[column]] += delta;
          v[column] -= delta;
        } else {
          min_value[column] -= delta;
        }
      }
      column0 = column1;
    } while (p[column0] != 0);
    do {
      const std::size_t COLUMN1 = way[column0];
      p[column0] = p[COLUMN1];
      column0 = COLUMN1;
    } while (column0 != 0);
  }
  for (std::size_t column = 1; column <= COLUMNS; ++column) {
    if (p[column] == 0 || column > TRUTH_COLUMNS ||
        costs[p[column] - 1][column - 1] >= UNMATCHED_COST) {
      continue;
    }
    matches[p[column] - 1] = column - 1;
  }
  return matches;
}

EvaluatedArmorPose ConvertEstimate(const modules::ArmorPoseEstimate& value,
                                   PnpEvaluationSource source) {
  EvaluatedArmorPose result;
  result.source = source;
  result.input_index = value.input_index;
  result.label = value.label;
  result.type = value.type;
  result.width_m = value.width_m;
  result.height_m = value.height_m;
  result.camera_t_armor = value.camera_t_armor;
  result.image_corners = value.image_corners;
  result.reprojected_corners = value.reprojected_corners;
  result.candidate_index = value.candidate_index;
  result.candidate_rmse_gap_px = value.candidate_rmse_gap_px;
  result.reprojection_rmse_px = value.reprojection_rmse_px;
  result.image_width_px = value.image_width_px;
  result.image_height_px = value.image_height_px;
  result.distance_m = value.distance_m;
  result.viewing_angle_deg = value.viewing_angle_deg;
  return result;
}

PnpEvaluationAttempt ConvertAttempt(const modules::ArmorPnpAttempt& value,
                                    PnpEvaluationSource source) {
  PnpEvaluationAttempt result{.source = source,
                              .input_index = value.input_index,
                              .status = value.status,
                              .estimate = std::nullopt,
                              .refinement = value.refinement};
  if (value.estimate)
    result.estimate = ConvertEstimate(*value.estimate, source);
  return result;
}

void AddTruthErrors(EvaluatedArmorPose& estimate, const simulation::GroundTruthArmor& truth,
                    const frame::FrameKinematics& kinematics,
                    const std::array<cv::Point2f, 4>& truth_pixels) {
  const auto WORLD_T_CAMERA =
      geometry::Compose(kinematics.world_t_gimbal, kinematics.gimbal_t_camera_optical);
  const auto ACTUAL = geometry::Compose(geometry::Inverse(WORLD_T_CAMERA), truth.world_t_armor);
  const auto POSITION_ERROR = estimate.camera_t_armor.translation - ACTUAL.translation;
  estimate.truth_id = truth.id;
  estimate.truth_distance_m = ACTUAL.translation.norm();
  estimate.truth_viewing_angle_deg =
      std::acos(std::clamp(-geometry::TransformVector(ACTUAL, geometry::Vector3::UnitZ())
                                .dot(ACTUAL.translation.normalized()),
                           -1.0, 1.0)) *
      K_RAD_TO_DEG;
  estimate.truth_type = truth.type;
  estimate.position_error_m = POSITION_ERROR.norm();
  estimate.position_error_camera_m =
      std::array<double, 3>{POSITION_ERROR.x(), POSITION_ERROR.y(), POSITION_ERROR.z()};
  estimate.signed_depth_error_m = POSITION_ERROR.z();
  estimate.depth_error_m = std::abs(POSITION_ERROR.z());
  estimate.rotation_error_deg =
      estimate.camera_t_armor.rotation.angularDistance(ACTUAL.rotation) * K_RAD_TO_DEG;
  double sum = 0.0;
  for (std::size_t corner = 0; corner < truth_pixels.size(); ++corner) {
    estimate.corner_delta_u_px[corner] = estimate.image_corners[corner].x - truth_pixels[corner].x;
    estimate.corner_delta_v_px[corner] = estimate.image_corners[corner].y - truth_pixels[corner].y;
    estimate.corner_errors_px[corner] =
        cv::norm(estimate.image_corners[corner] - truth_pixels[corner]);
    sum += estimate.corner_errors_px[corner];
  }
  estimate.mean_corner_error_px = sum / static_cast<double>(truth_pixels.size());
}

void AddSamples(MetricSamples& samples, const EvaluatedArmorPose& estimate) {
  samples.reprojection.push_back(estimate.reprojection_rmse_px);
  if (estimate.mean_corner_error_px)
    samples.corner.push_back(*estimate.mean_corner_error_px);
  if (estimate.position_error_m)
    samples.position.push_back(*estimate.position_error_m);
  if (estimate.depth_error_m)
    samples.depth.push_back(*estimate.depth_error_m);
  if (estimate.rotation_error_deg)
    samples.rotation.push_back(*estimate.rotation_error_deg);
  if (estimate.position_jitter_m)
    samples.jitter.push_back(*estimate.position_jitter_m);
}

double HeadingYaw(const geometry::Quaternion& orientation) {
  const auto HEADING = orientation * geometry::Vector3::UnitX();
  return std::atan2(HEADING.y(), HEADING.x());
}

double WrapAngle(double angle) {
  return std::remainder(angle, 2.0 * std::numbers::pi);
}

double WrapFourArmorYaw(double angle) {
  return std::remainder(angle, std::numbers::pi * 0.5);
}

}  // namespace

struct SimulationEvaluator::Impl {
  Impl(SimulationEvaluationConfig evaluation_config, modules::ArmorPnpConfig pnp_config)
      : config(evaluation_config), solver(pnp_config) {}

  SimulationEvaluationConfig config;
  modules::ArmorPnpSolver solver;
  MetricSamples truth_samples;
  MetricSamples detection_samples;
  std::map<std::string, MetricSamples> distance_samples;
  std::map<std::string, MetricSamples> angle_samples;
  std::map<std::string, MetricSamples> size_samples;
  std::map<std::pair<PnpEvaluationSource, std::uint64_t>, geometry::Vector3> previous_error;
  std::map<std::pair<PnpEvaluationSource, std::uint64_t>, std::size_t> previous_candidate;
  std::map<std::pair<PnpEvaluationSource, std::uint64_t>, std::uint64_t> previous_sequence;
  PnpSolveSummary solve_summary;
  CornerRefinementSummary refinement_summary;
  PnpEvaluationResult snapshot;
  std::vector<double> refinement_elapsed_samples;
  std::vector<double> raw_corner_error_samples;
  std::vector<double> final_corner_error_samples;
  bool snapshot_initialized{false};

  void RecordAttempts(std::span<PnpEvaluationAttempt> attempts, std::uint64_t sequence) {
    for (auto& attempt : attempts) {
      if (!attempt.estimate)
        continue;
      auto& estimate = *attempt.estimate;
      if (estimate.truth_id && estimate.position_error_camera_m) {
        const auto VALUES = *estimate.position_error_camera_m;
        const geometry::Vector3 ERROR(VALUES[0], VALUES[1], VALUES[2]);
        const auto KEY = std::make_pair(attempt.source, *estimate.truth_id);
        const auto PREVIOUS = previous_error.find(KEY);
        const auto PREVIOUS_SEQUENCE = previous_sequence.find(KEY);
        const bool CONSECUTIVE =
            PREVIOUS_SEQUENCE != previous_sequence.end() &&
            PREVIOUS_SEQUENCE->second != std::numeric_limits<std::uint64_t>::max() &&
            PREVIOUS_SEQUENCE->second + 1 == sequence;
        if (PREVIOUS != previous_error.end() && CONSECUTIVE)
          estimate.position_jitter_m = (ERROR - PREVIOUS->second).norm();
        previous_error[KEY] = ERROR;
        const auto PREVIOUS_CANDIDATE = previous_candidate.find(KEY);
        if (PREVIOUS_CANDIDATE != previous_candidate.end() && CONSECUTIVE &&
            PREVIOUS_CANDIDATE->second != estimate.candidate_index &&
            attempt.source != PnpEvaluationSource::GROUND_TRUTH) {
          ++solve_summary.candidate_switches;
        }
        previous_candidate[KEY] = estimate.candidate_index;
        previous_sequence[KEY] = sequence;
      }
      if (attempt.source == PnpEvaluationSource::GROUND_TRUTH) {
        AddSamples(truth_samples, estimate);
      } else {
        AddSamples(detection_samples, estimate);
        if (estimate.truth_distance_m && estimate.truth_viewing_angle_deg && estimate.truth_type) {
          AddSamples(distance_samples[DistanceGroup(*estimate.truth_distance_m)], estimate);
          AddSamples(angle_samples[AngleGroup(*estimate.truth_viewing_angle_deg)], estimate);
          AddSamples(size_samples[SizeGroup(*estimate.truth_type)], estimate);
        }
      }
    }
  }

  void PopulateSnapshot(std::uint64_t sequence, PnpEvaluationResult& result) {
    if (!snapshot_initialized || sequence % 100 == 0) {
      snapshot_initialized = true;
      snapshot.summary_sequence = sequence;
      snapshot.ground_truth_summary = Summarize(truth_samples);
      snapshot.detection_summary = Summarize(detection_samples);
      snapshot.distance_groups = SummarizeGroups(distance_samples);
      snapshot.angle_groups = SummarizeGroups(angle_samples);
      snapshot.size_groups = SummarizeGroups(size_samples);
      refinement_summary.elapsed_ms = Percentiles(refinement_elapsed_samples);
      refinement_summary.raw_mean_corner_error_px = Percentiles(raw_corner_error_samples);
      refinement_summary.final_mean_corner_error_px = Percentiles(final_corner_error_samples);
      snapshot.solve_summary = solve_summary;
      snapshot.refinement_summary = refinement_summary;
    }
    result.summary_sequence = snapshot.summary_sequence;
    result.ground_truth_summary = snapshot.ground_truth_summary;
    result.detection_summary = snapshot.detection_summary;
    result.distance_groups = snapshot.distance_groups;
    result.angle_groups = snapshot.angle_groups;
    result.size_groups = snapshot.size_groups;
    result.solve_summary = snapshot.solve_summary;
    result.refinement_summary = snapshot.refinement_summary;
  }
};

const char* PnpEvaluationSourceName(PnpEvaluationSource source) noexcept {
  return source == PnpEvaluationSource::GROUND_TRUTH ? "ground_truth" : "detection";
}

PnpEvaluationResult MakePnpDiagnosticResult(const modules::ArmorPnpFrameResult& formal) {
  PnpEvaluationResult result;
  result.summary_sequence = formal.summary_sequence;
  result.attempts.reserve(formal.attempts.size());
  for (const auto& attempt : formal.attempts)
    result.attempts.push_back(ConvertAttempt(attempt, PnpEvaluationSource::DETECTION));
  const auto& reprojection = formal.detection_summary.reprojection_rmse_px;
  result.detection_summary.reprojection_rmse_px = {
      .samples = reprojection.samples, .p50 = reprojection.p50, .p95 = reprojection.p95};
  result.solve_summary.attempted = formal.solve_summary.attempted;
  result.solve_summary.succeeded = formal.solve_summary.succeeded;
  result.solve_summary.rejection_reasons = formal.solve_summary.rejection_reasons;
  result.refinement_summary.attempted = formal.refinement_summary.attempted;
  result.refinement_summary.succeeded = formal.refinement_summary.succeeded;
  result.refinement_summary.fallback = formal.refinement_summary.fallback;
  result.refinement_summary.failure_reasons = formal.refinement_summary.failure_reasons;
  const auto& elapsed = formal.refinement_summary.elapsed_ms;
  result.refinement_summary.elapsed_ms = {
      .samples = elapsed.samples, .p50 = elapsed.p50, .p95 = elapsed.p95};
  return result;
}

SimulationEvaluator::SimulationEvaluator(SimulationEvaluationConfig config,
                                         modules::ArmorPnpConfig pnp_config)
    : impl_(std::make_unique<Impl>(config, pnp_config)) {}

SimulationEvaluator::~SimulationEvaluator() = default;

SimulationEvaluationResult SimulationEvaluator::Evaluate(const SimulationEvaluationInput& input) {
  const auto START = std::chrono::steady_clock::now();
  SimulationEvaluationResult result;
  result.sequence = input.stamp.sequence;
  if (input.refinements.size() == input.detections.size()) {
    std::vector<VisibleArmorTruth> visible_truth;
    for (std::size_t index = 0; index < input.simulation.armors.size(); ++index) {
      const auto& armor = input.simulation.armors[index];
      const auto PIXELS = ProjectVisibleTruth(armor, input.camera_model, input.kinematics);
      if (!PIXELS)
        continue;
      visible_truth.push_back({&armor, *PIXELS});
      auto attempt = ConvertAttempt(
          impl_->solver.Solve(*PIXELS, armor.type, input.camera_model, index, armor.label),
          PnpEvaluationSource::GROUND_TRUTH);
      if (attempt.estimate)
        AddTruthErrors(*attempt.estimate, armor, input.kinematics, *PIXELS);
      result.pnp.attempts.push_back(std::move(attempt));
    }
    const auto MATCHES = MatchDetectionsToTruth(input.detections, visible_truth, impl_->config);
    for (std::size_t index = 0; index < input.detections.size(); ++index) {
      const auto FORMAL =
          std::find_if(input.pnp.attempts.begin(), input.pnp.attempts.end(),
                       [index](const auto& attempt) { return attempt.input_index == index; });
      modules::ArmorPnpAttempt missing;
      missing.input_index = index;
      auto attempt = ConvertAttempt(FORMAL == input.pnp.attempts.end() ? missing : *FORMAL,
                                    PnpEvaluationSource::DETECTION);
      const auto& refinement = input.refinements[index];
      ++impl_->refinement_summary.attempted;
      impl_->refinement_elapsed_samples.push_back(refinement.elapsed_ms);
      if (refinement.success && !refinement.fallback) {
        ++impl_->refinement_summary.succeeded;
      } else {
        ++impl_->refinement_summary.fallback;
        ++impl_->refinement_summary
              .failure_reasons[modules::CornerRefinementStatusName(refinement.status)];
      }
      ++impl_->solve_summary.attempted;
      if (attempt.estimate) {
        ++impl_->solve_summary.succeeded;
      } else {
        ++impl_->solve_summary.rejection_reasons[modules::PnpStatusName(attempt.status)];
      }
      const std::size_t BEST_TRUTH = MATCHES[index];
      if (BEST_TRUTH < visible_truth.size()) {
        const auto& truth = visible_truth[BEST_TRUTH];
        const auto& final_corners =
            refinement.success ? refinement.refined_corners : refinement.original_corners;
        double raw_error = 0.0;
        double final_error = 0.0;
        for (std::size_t corner = 0; corner < 4; ++corner) {
          raw_error += cv::norm(refinement.original_corners[corner] - truth.pixels[corner]);
          final_error += cv::norm(final_corners[corner] - truth.pixels[corner]);
        }
        impl_->raw_corner_error_samples.push_back(raw_error / 4.0);
        impl_->final_corner_error_samples.push_back(final_error / 4.0);
        if (attempt.estimate)
          AddTruthErrors(*attempt.estimate, *truth.armor, input.kinematics, truth.pixels);
      }
      result.pnp.attempts.push_back(std::move(attempt));
    }
    impl_->RecordAttempts(result.pnp.attempts, input.stamp.sequence);
    impl_->PopulateSnapshot(input.stamp.sequence, result.pnp);
  }

  if (input.prediction.state != modules::TrackerState::LOST && input.prediction.label) {
    const auto BEST = std::min_element(
        input.simulation.targets.begin(), input.simulation.targets.end(),
        [&](const auto& left, const auto& right) {
          const double LEFT_PENALTY =
              left.armor_label == static_cast<std::uint8_t>(*input.prediction.label) ? 0.0 : 1.0e6;
          const double RIGHT_PENALTY =
              right.armor_label == static_cast<std::uint8_t>(*input.prediction.label) ? 0.0 : 1.0e6;
          return LEFT_PENALTY +
                     (left.position_world - input.prediction.center_world).squaredNorm() <
                 RIGHT_PENALTY +
                     (right.position_world - input.prediction.center_world).squaredNorm();
        });
    if (BEST != input.simulation.targets.end() &&
        BEST->armor_label == static_cast<std::uint8_t>(*input.prediction.label)) {
      auto& prediction = result.prediction;
      prediction.truth_target_id = BEST->id;
      prediction.truth_position_world = BEST->position_world;
      prediction.truth_yaw_velocity_rad_s = BEST->yaw_velocity;
      prediction.center_error_m = (input.prediction.center_world - BEST->position_world).norm();
      prediction.yaw_error_rad =
          WrapAngle(HeadingYaw(input.prediction.orientation_world) - BEST->yaw);
      prediction.yaw_equivalent_error_rad = WrapFourArmorYaw(*prediction.yaw_error_rad);
      prediction.yaw_velocity_error_rad_s =
          input.prediction.yaw_velocity_rad_s - BEST->yaw_velocity;
    }
  }
  result.elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - START).count();
  return result;
}

}  // namespace mv::tool::simulation_evaluation
