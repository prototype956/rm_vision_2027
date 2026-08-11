#include "modules/armor_pnp/armor_pnp.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Geometry>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::modules {
namespace {

constexpr double K_RAD_TO_DEG = 180.0 / std::numbers::pi;

std::array<cv::Point3d, 4> ObjectPoints(hal::CameraFrame::ArmorType type,
                                        const ArmorPnpConfig& config) {
  const double width =
      type == hal::CameraFrame::ArmorType::LARGE ? config.large_width_m : config.small_width_m;
  const double half_width = width * 0.5;
  const double half_height = config.height_m * 0.5;
  return {cv::Point3d(-half_width, half_height, 0.0), cv::Point3d(half_width, half_height, 0.0),
          cv::Point3d(half_width, -half_height, 0.0), cv::Point3d(-half_width, -half_height, 0.0)};
}

cv::Matx33d CameraMatrix(const hal::CameraFrame::Calibration& value) {
  return {value.fx, 0.0, value.cx, 0.0, value.fy, value.cy, 0.0, 0.0, 1.0};
}

cv::Vec<double, 5> Distortion(const hal::CameraFrame::Calibration& value) {
  return {value.distortion[0], value.distortion[1], value.distortion[2], value.distortion[3],
          value.distortion[4]};
}

geometry::RigidTransform ToTransform(const cv::Mat& rvec, const cv::Mat& tvec) {
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  Eigen::Matrix3d eigen_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      eigen_rotation(row, column) = rotation.at<double>(row, column);
    }
  }
  return {
      .translation = geometry::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)),
      .rotation = geometry::Quaternion(eigen_rotation).normalized()};
}

bool Finite(const cv::Point2f& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

std::optional<std::array<cv::Point2f, 4>> ProjectTruth(
    const hal::CameraFrame::GroundTruthArmor& armor,
    const hal::CameraFrame::FrameGeometry& geometry) {
  const auto world_t_camera =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  const auto camera_t_world = geometry::Inverse(world_t_camera);
  const auto camera_t_armor = geometry::Compose(camera_t_world, armor.world_t_armor);
  const auto normal_camera = geometry::TransformVector(camera_t_armor, geometry::Vector3::UnitZ());
  if (normal_camera.dot(camera_t_armor.translation) >= 0.0) {
    return std::nullopt;
  }
  std::array<cv::Point2f, 4> projected{};
  for (std::size_t index = 0; index < projected.size(); ++index) {
    const auto point = geometry::TransformPoint(camera_t_world, armor.corners_world[index]);
    if (point.z() <= 0.0) {
      return std::nullopt;
    }
    projected[index] =
        cv::Point2f(static_cast<float>(geometry.calibration.fx * point.x() / point.z() +
                                       geometry.calibration.cx),
                    static_cast<float>(geometry.calibration.fy * point.y() / point.z() +
                                       geometry.calibration.cy));
  }
  const auto bounds =
      cv::boundingRect(std::vector<cv::Point2f>(projected.begin(), projected.end()));
  const cv::Rect image_bounds(0, 0, static_cast<int>(geometry.calibration.width),
                              static_cast<int>(geometry.calibration.height));
  if ((bounds & image_bounds).empty()) {
    return std::nullopt;
  }
  return projected;
}

bool LabelsMatch(ArmorLabel detection, std::uint8_t truth) {
  if (detection == ArmorLabel::BASE_SMALL || detection == ArmorLabel::BASE_BIG) {
    return truth == 7;
  }
  return static_cast<std::uint8_t>(detection) == truth;
}

std::uint8_t TeamForColor(ArmorColor color) {
  return color == ArmorColor::RED ? 0 : 1;
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
  const auto bounds = cv::boundingRect(std::vector<cv::Point2f>(polygon.begin(), polygon.end()));
  return std::hypot(static_cast<double>(bounds.width), static_cast<double>(bounds.height));
}

struct VisibleTruth {
  const hal::CameraFrame::GroundTruthArmor* armor;
  std::array<cv::Point2f, 4> pixels;
};

std::vector<std::size_t> MatchDetectionsToTruth(std::span<const ArmorDetection> detections,
                                                std::span<const VisibleTruth> truths,
                                                const ArmorPnpConfig& config) {
  constexpr double FORBIDDEN_COST = 1.0e6;
  constexpr double UNMATCHED_COST = 10.0;
  const std::size_t rows = detections.size();
  const std::size_t truth_columns = truths.size();
  const std::size_t columns = truth_columns + rows;
  std::vector<std::size_t> matches(rows, truth_columns);
  if (rows == 0 || truth_columns == 0)
    return matches;

  std::vector<std::vector<double>> costs(rows, std::vector<double>(columns, UNMATCHED_COST));
  for (std::size_t row = 0; row < rows; ++row) {
    const auto& detection = detections[row];
    const double detection_area = PolygonArea(detection.corners);
    const double detection_diagonal = PolygonDiagonal(detection.corners);
    for (std::size_t column = 0; column < truth_columns; ++column) {
      const auto& truth = truths[column];
      if (truth.armor->team != TeamForColor(detection.color) ||
          !LabelsMatch(detection.label, truth.armor->label)) {
        costs[row][column] = FORBIDDEN_COST;
        continue;
      }
      const double truth_area = PolygonArea(truth.pixels);
      const double intersection = PolygonIntersection(detection.corners, truth.pixels);
      const double union_area = detection_area + truth_area - intersection;
      const double iou = union_area > 1.0e-6 ? intersection / union_area : 0.0;
      const double scale = std::max({1.0, detection_diagonal, PolygonDiagonal(truth.pixels)});
      const double center_ratio =
          cv::norm(PolygonCenter(detection.corners) - PolygonCenter(truth.pixels)) / scale;
      double mean_corner_distance = 0.0;
      for (std::size_t corner = 0; corner < 4; ++corner)
        mean_corner_distance += cv::norm(detection.corners[corner] - truth.pixels[corner]);
      const double corner_ratio = mean_corner_distance / (4.0 * scale);
      if (iou < config.truth_match_min_iou ||
          center_ratio > config.truth_match_max_center_distance_ratio ||
          corner_ratio > config.truth_match_max_corner_distance_ratio) {
        costs[row][column] = FORBIDDEN_COST;
        continue;
      }
      costs[row][column] = 2.0 * (1.0 - iou) + center_ratio + corner_ratio;
    }
  }

  // Rectangular Hungarian assignment. Per-detection dummy columns make "unmatched" explicit,
  // while all real truth columns remain globally one-to-one.
  std::vector<double> u(rows + 1), v(columns + 1);
  std::vector<std::size_t> p(columns + 1), way(columns + 1);
  for (std::size_t row = 1; row <= rows; ++row) {
    p[0] = row;
    std::size_t column0 = 0;
    std::vector<double> min_value(columns + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(columns + 1, false);
    do {
      used[column0] = true;
      const std::size_t row0 = p[column0];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t column1 = 0;
      for (std::size_t column = 1; column <= columns; ++column) {
        if (used[column])
          continue;
        const double current = costs[row0 - 1][column - 1] - u[row0] - v[column];
        if (current < min_value[column]) {
          min_value[column] = current;
          way[column] = column0;
        }
        if (min_value[column] < delta) {
          delta = min_value[column];
          column1 = column;
        }
      }
      for (std::size_t column = 0; column <= columns; ++column) {
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
      const std::size_t column1 = way[column0];
      p[column0] = p[column1];
      column0 = column1;
    } while (column0 != 0);
  }
  for (std::size_t column = 1; column <= columns; ++column) {
    if (p[column] == 0 || column > truth_columns ||
        costs[p[column] - 1][column - 1] >= UNMATCHED_COST)
      continue;
    matches[p[column] - 1] = column - 1;
  }
  return matches;
}

void AddTruthErrors(ArmorPoseEstimate& estimate, const hal::CameraFrame::GroundTruthArmor& truth,
                    const hal::CameraFrame::FrameGeometry& geometry,
                    const std::array<cv::Point2f, 4>& truth_pixels) {
  const auto world_t_camera =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  const auto actual = geometry::Compose(geometry::Inverse(world_t_camera), truth.world_t_armor);
  const auto position_error = estimate.camera_t_armor.translation - actual.translation;
  estimate.truth_id = truth.id;
  estimate.truth_distance_m = actual.translation.norm();
  estimate.truth_viewing_angle_deg =
      std::acos(std::clamp(-geometry::TransformVector(actual, geometry::Vector3::UnitZ())
                                .dot(actual.translation.normalized()),
                           -1.0, 1.0)) *
      K_RAD_TO_DEG;
  estimate.truth_type = truth.type;
  estimate.position_error_m = position_error.norm();
  estimate.position_error_camera_m =
      std::array<double, 3>{position_error.x(), position_error.y(), position_error.z()};
  estimate.signed_depth_error_m = position_error.z();
  estimate.depth_error_m = std::abs(position_error.z());
  estimate.rotation_error_deg =
      estimate.camera_t_armor.rotation.angularDistance(actual.rotation) * K_RAD_TO_DEG;
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

PnpPercentiles Percentiles(const std::vector<double>& samples) {
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

PnpSourceSummary Summarize(const PnpMetricSamples& samples) {
  return {.reprojection_rmse_px = Percentiles(samples.reprojection),
          .mean_corner_error_px = Percentiles(samples.corner),
          .position_error_m = Percentiles(samples.position),
          .depth_error_m = Percentiles(samples.depth),
          .rotation_error_deg = Percentiles(samples.rotation),
          .position_jitter_m = Percentiles(samples.jitter)};
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

std::string SizeGroup(hal::CameraFrame::ArmorType type) {
  return type == hal::CameraFrame::ArmorType::LARGE ? "large" : "small";
}

void AddSamples(PnpMetricSamples& samples, const ArmorPoseEstimate& estimate) {
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

std::map<std::string, PnpSourceSummary> SummarizeGroups(
    const std::map<std::string, PnpMetricSamples>& groups) {
  std::map<std::string, PnpSourceSummary> result;
  for (const auto& [name, samples] : groups)
    result.emplace(name, Summarize(samples));
  return result;
}

}  // namespace

ArmorPnpConfig ParseArmorPnpConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "armor PnP config";
  ConfigLoader::RejectUnknownKeys(
      root,
      {"schema_version", "small_width_m", "large_width_m", "height_m", "min_distance_m",
       "max_distance_m", "truth_match_min_iou", "truth_match_max_center_distance_ratio",
       "truth_match_max_corner_distance_ratio"},
      CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 2) {
    throw ConfigError("armor PnP config schema_version must be 2");
  }
  ArmorPnpConfig config{
      .small_width_m = ConfigLoader::Require<double>(root, "small_width_m", CONTEXT),
      .large_width_m = ConfigLoader::Require<double>(root, "large_width_m", CONTEXT),
      .height_m = ConfigLoader::Require<double>(root, "height_m", CONTEXT),
      .min_distance_m = ConfigLoader::Require<double>(root, "min_distance_m", CONTEXT),
      .max_distance_m = ConfigLoader::Require<double>(root, "max_distance_m", CONTEXT),
      .truth_match_min_iou = ConfigLoader::Require<double>(root, "truth_match_min_iou", CONTEXT),
      .truth_match_max_center_distance_ratio =
          ConfigLoader::Require<double>(root, "truth_match_max_center_distance_ratio", CONTEXT),
      .truth_match_max_corner_distance_ratio =
          ConfigLoader::Require<double>(root, "truth_match_max_corner_distance_ratio", CONTEXT)};
  if (!(config.small_width_m > 0.0 && config.large_width_m > config.small_width_m &&
        config.height_m > 0.0 && config.min_distance_m > 0.0 &&
        config.max_distance_m > config.min_distance_m && config.truth_match_min_iou >= 0.0 &&
        config.truth_match_min_iou <= 1.0 && config.truth_match_max_center_distance_ratio > 0.0 &&
        config.truth_match_max_corner_distance_ratio > 0.0)) {
    throw ConfigError("armor PnP dimensions or distance range are invalid");
  }
  return config;
}

const char* PnpStatusName(PnpStatus status) noexcept {
  switch (status) {
    case PnpStatus::SUCCESS:
      return "success";
    case PnpStatus::INVALID_INPUT:
      return "invalid_input";
    case PnpStatus::NO_SOLUTION:
      return "no_solution";
    case PnpStatus::NEGATIVE_DEPTH:
      return "negative_depth";
    case PnpStatus::BACK_FACING:
      return "back_facing";
    case PnpStatus::OUT_OF_RANGE:
      return "out_of_range";
  }
  return "unknown";
}

const char* PnpInputSourceName(PnpInputSource source) noexcept {
  switch (source) {
    case PnpInputSource::GROUND_TRUTH:
      return "ground_truth";
    case PnpInputSource::DETECTION_RAW:
      return "detection_raw";
    case PnpInputSource::DETECTION_REFINED:
      return "detection_refined";
  }
  return "unknown";
}

hal::CameraFrame::ArmorType ArmorTypeForLabel(ArmorLabel label) noexcept {
  return label == ArmorLabel::ONE || label == ArmorLabel::BASE_BIG
             ? hal::CameraFrame::ArmorType::LARGE
             : hal::CameraFrame::ArmorType::SMALL;
}

ArmorPnp::ArmorPnp(ArmorPnpConfig config) : config_(std::move(config)) {}

ArmorPnpAttempt ArmorPnp::Solve(std::span<const cv::Point2f, 4> image_corners,
                                hal::CameraFrame::ArmorType type,
                                const hal::CameraFrame::Calibration& calibration,
                                PnpInputSource source, std::size_t input_index,
                                std::uint8_t label) const {
  ArmorPnpAttempt result{.source = source,
                         .input_index = input_index,
                         .status = PnpStatus::INVALID_INPUT,
                         .estimate = std::nullopt,
                         .refinement = std::nullopt};
  if (calibration.fx <= 0.0 || calibration.fy <= 0.0 ||
      !std::all_of(image_corners.begin(), image_corners.end(), Finite)) {
    return result;
  }
  const auto object_points = ObjectPoints(type, config_);
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  try {
    const int count = cv::solvePnPGeneric(
        std::vector<cv::Point3d>(object_points.begin(), object_points.end()),
        std::vector<cv::Point2f>(image_corners.begin(), image_corners.end()),
        CameraMatrix(calibration), Distortion(calibration), rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    if (count <= 0) {
      result.status = PnpStatus::NO_SOLUTION;
      return result;
    }
  } catch (const cv::Exception&) {
    result.status = PnpStatus::NO_SOLUTION;
    return result;
  }

  double best_rmse = std::numeric_limits<double>::infinity();
  double second_rmse = std::numeric_limits<double>::infinity();
  PnpStatus last_rejection = PnpStatus::NO_SOLUTION;
  std::optional<ArmorPoseEstimate> best;
  for (std::size_t candidate = 0; candidate < rvecs.size(); ++candidate) {
    const auto pose = ToTransform(rvecs[candidate], tvecs[candidate]);
    bool positive = true;
    for (const auto& object_point : object_points) {
      const geometry::Vector3 point =
          geometry::TransformPoint(pose, {object_point.x, object_point.y, object_point.z});
      positive = positive && point.z() > 0.0;
    }
    if (!positive) {
      last_rejection = PnpStatus::NEGATIVE_DEPTH;
      continue;
    }
    if (geometry::TransformVector(pose, geometry::Vector3::UnitZ()).dot(pose.translation) >= 0.0) {
      last_rejection = PnpStatus::BACK_FACING;
      continue;
    }
    const double distance = pose.translation.norm();
    if (distance < config_.min_distance_m || distance > config_.max_distance_m) {
      last_rejection = PnpStatus::OUT_OF_RANGE;
      continue;
    }
    std::vector<cv::Point2d> projected;
    cv::projectPoints(std::vector<cv::Point3d>(object_points.begin(), object_points.end()),
                      rvecs[candidate], tvecs[candidate], CameraMatrix(calibration),
                      Distortion(calibration), projected);
    double squared_error = 0.0;
    for (std::size_t corner = 0; corner < projected.size(); ++corner) {
      const double dx = projected[corner].x - image_corners[corner].x;
      const double dy = projected[corner].y - image_corners[corner].y;
      squared_error += dx * dx + dy * dy;
    }
    const double rmse = std::sqrt(squared_error / static_cast<double>(projected.size()));
    if (rmse >= best_rmse) {
      second_rmse = std::min(second_rmse, rmse);
      continue;
    }
    second_rmse = best_rmse;
    ArmorPoseEstimate estimate{
        .source = source,
        .input_index = input_index,
        .truth_id = std::nullopt,
        .label = label,
        .type = type,
        .width_m = type == hal::CameraFrame::ArmorType::LARGE ? config_.large_width_m
                                                              : config_.small_width_m,
        .height_m = config_.height_m,
        .camera_t_armor = pose,
        .candidate_index = candidate,
        .candidate_rmse_gap_px = std::nullopt,
        .reprojection_rmse_px = rmse,
        .image_width_px = 0.5 * (cv::norm(image_corners[1] - image_corners[0]) +
                                 cv::norm(image_corners[2] - image_corners[3])),
        .image_height_px = 0.5 * (cv::norm(image_corners[3] - image_corners[0]) +
                                  cv::norm(image_corners[2] - image_corners[1])),
        .distance_m = distance,
        .viewing_angle_deg =
            std::acos(std::clamp(-geometry::TransformVector(pose, geometry::Vector3::UnitZ())
                                      .dot(pose.translation.normalized()),
                                 -1.0, 1.0)) *
            K_RAD_TO_DEG,
        .truth_distance_m = std::nullopt,
        .truth_viewing_angle_deg = std::nullopt,
        .truth_type = std::nullopt,
        .mean_corner_error_px = std::nullopt,
        .position_error_m = std::nullopt,
        .position_error_camera_m = std::nullopt,
        .depth_error_m = std::nullopt,
        .signed_depth_error_m = std::nullopt,
        .rotation_error_deg = std::nullopt,
        .position_jitter_m = std::nullopt};
    std::copy(image_corners.begin(), image_corners.end(), estimate.image_corners.begin());
    for (std::size_t corner = 0; corner < projected.size(); ++corner) {
      estimate.reprojected_corners[corner] = cv::Point2f(projected[corner]);
    }
    best_rmse = rmse;
    best = std::move(estimate);
  }
  if (!best) {
    result.status = last_rejection;
    return result;
  }
  result.status = PnpStatus::SUCCESS;
  if (std::isfinite(second_rmse))
    best->candidate_rmse_gap_px = second_rmse - best_rmse;
  result.estimate = std::move(best);
  return result;
}

ArmorPnpFrameResult ArmorPnp::ProcessFrame(const hal::CameraFrame& frame,
                                           std::span<const ArmorDetection> detections,
                                           std::span<const CornerRefinementResult> refinements) {
  ArmorPnpFrameResult result;
  if (!frame.geometry || refinements.size() != detections.size())
    return result;
  const auto& geometry = *frame.geometry;
  std::vector<VisibleTruth> visible_truth;
  for (std::size_t index = 0; index < geometry.armors.size(); ++index) {
    const auto pixels = ProjectTruth(geometry.armors[index], geometry);
    if (!pixels)
      continue;
    visible_truth.push_back({&geometry.armors[index], *pixels});
    auto attempt = Solve(*pixels, geometry.armors[index].type, geometry.calibration,
                         PnpInputSource::GROUND_TRUTH, index, geometry.armors[index].label);
    if (attempt.estimate) {
      AddTruthErrors(*attempt.estimate, geometry.armors[index], geometry, *pixels);
    }
    result.attempts.push_back(std::move(attempt));
  }

  const auto truth_matches = MatchDetectionsToTruth(detections, visible_truth, config_);
  for (std::size_t index = 0; index < detections.size(); ++index) {
    const auto& detection = detections[index];
    const auto type = ArmorTypeForLabel(detection.label);
    auto raw = Solve(detection.corners, type, geometry.calibration, PnpInputSource::DETECTION_RAW,
                     index, static_cast<std::uint8_t>(detection.label));
    ++raw_solve_summary_.attempted;
    if (raw.estimate) {
      ++raw_solve_summary_.succeeded;
    } else {
      ++raw_solve_summary_.rejection_reasons[PnpStatusName(raw.status)];
    }

    const std::size_t best_truth = truth_matches[index];
    if (best_truth < visible_truth.size()) {
      if (raw.estimate)
        AddTruthErrors(*raw.estimate, *visible_truth[best_truth].armor, geometry,
                       visible_truth[best_truth].pixels);
    }

    CornerRefinementResult refinement = refinements[index];
    ++refinement_summary_.attempted;
    refinement_elapsed_samples_.push_back(refinement.elapsed_ms);
    ArmorPnpAttempt refined;
    if (refinement.success && !refinement.fallback) {
      refined = Solve(refinement.refined_corners, type, geometry.calibration,
                      PnpInputSource::DETECTION_REFINED, index,
                      static_cast<std::uint8_t>(detection.label));
      if (!refined.estimate) {
        refinement.success = false;
        refinement.fallback = true;
        refinement.status = CornerRefinementStatus::PNP_FALLBACK;
        refinement.refined_corners = refinement.original_corners;
        for (std::size_t corner = 0; corner < refinement.endpoints.size(); ++corner) {
          auto& endpoint = refinement.endpoints[corner];
          if (endpoint.applied) {
            endpoint.applied = false;
            endpoint.fallback = true;
            endpoint.reverted_by = EndpointRevertedBy::PNP;
          }
          endpoint.final = refinement.original_corners[corner];
          endpoint.movement_px = 0.0;
          refinement.corner_displacements[corner] = {};
        }
        refined = raw;
        refined.source = PnpInputSource::DETECTION_REFINED;
        if (refined.estimate)
          refined.estimate->source = PnpInputSource::DETECTION_REFINED;
      }
    } else {
      // A fallback must be numerically identical to the official raw chain. Copying the solved
      // attempt also avoids relying on a second OpenCV call returning bit-identical candidates.
      refined = raw;
      refined.source = PnpInputSource::DETECTION_REFINED;
      if (refined.estimate)
        refined.estimate->source = PnpInputSource::DETECTION_REFINED;
    }
    if (refinement.success && !refinement.fallback) {
      ++refinement_summary_.succeeded;
      ++refinement_summary_.fully_refined;
    } else {
      ++refinement_summary_.fallback;
      ++refinement_summary_.full_fallback;
      ++refinement_summary_.failure_reasons[CornerRefinementStatusName(refinement.status)];
    }
    ++refined_solve_summary_.attempted;
    if (refined.estimate) {
      ++refined_solve_summary_.succeeded;
    } else {
      ++refined_solve_summary_.rejection_reasons[PnpStatusName(refined.status)];
    }
    refined.refinement = refinement;
    if (best_truth < visible_truth.size() && refined.estimate)
      AddTruthErrors(*refined.estimate, *visible_truth[best_truth].armor, geometry,
                     visible_truth[best_truth].pixels);
    result.attempts.push_back(std::move(raw));
    result.attempts.push_back(std::move(refined));
  }
  for (auto& attempt : result.attempts) {
    if (!attempt.estimate)
      continue;
    auto& estimate = *attempt.estimate;
    if (estimate.truth_id && estimate.position_error_camera_m) {
      const auto values = *estimate.position_error_camera_m;
      const geometry::Vector3 error(values[0], values[1], values[2]);
      const auto key = std::make_pair(attempt.source, *estimate.truth_id);
      const auto previous = previous_error_.find(key);
      const auto previous_sequence = previous_sequence_.find(key);
      const bool consecutive =
          previous_sequence != previous_sequence_.end() &&
          previous_sequence->second != std::numeric_limits<std::uint64_t>::max() &&
          previous_sequence->second + 1 == frame.sequence;
      if (previous != previous_error_.end() && consecutive)
        estimate.position_jitter_m = (error - previous->second).norm();
      previous_error_[key] = error;
      const auto previous_candidate = previous_candidate_.find(key);
      if (previous_candidate != previous_candidate_.end() && consecutive &&
          previous_candidate->second != estimate.candidate_index) {
        auto& health = attempt.source == PnpInputSource::DETECTION_RAW ? raw_solve_summary_
                                                                       : refined_solve_summary_;
        if (attempt.source != PnpInputSource::GROUND_TRUTH)
          ++health.candidate_switches;
      }
      previous_candidate_[key] = estimate.candidate_index;
      previous_sequence_[key] = frame.sequence;
    }
    if (attempt.source == PnpInputSource::GROUND_TRUTH) {
      AddSamples(truth_samples_, estimate);
    } else if (attempt.source == PnpInputSource::DETECTION_RAW) {
      AddSamples(raw_samples_, estimate);
      if (estimate.truth_distance_m && estimate.truth_viewing_angle_deg && estimate.truth_type) {
        AddSamples(raw_distance_samples_[DistanceGroup(*estimate.truth_distance_m)], estimate);
        AddSamples(raw_angle_samples_[AngleGroup(*estimate.truth_viewing_angle_deg)], estimate);
        AddSamples(raw_size_samples_[SizeGroup(*estimate.truth_type)], estimate);
      }
    } else {
      AddSamples(refined_with_fallback_samples_, estimate);
      const bool refinement_succeeded =
          attempt.refinement && attempt.refinement->success && !attempt.refinement->fallback;
      if (refinement_succeeded)
        AddSamples(refined_success_samples_, estimate);
      if (estimate.truth_distance_m && estimate.truth_viewing_angle_deg && estimate.truth_type) {
        const auto distance_group = DistanceGroup(*estimate.truth_distance_m);
        const auto angle_group = AngleGroup(*estimate.truth_viewing_angle_deg);
        const auto size_group = SizeGroup(*estimate.truth_type);
        AddSamples(refined_with_fallback_distance_samples_[distance_group], estimate);
        AddSamples(refined_with_fallback_angle_samples_[angle_group], estimate);
        AddSamples(refined_with_fallback_size_samples_[size_group], estimate);
        if (refinement_succeeded) {
          AddSamples(refined_success_distance_samples_[distance_group], estimate);
          AddSamples(refined_success_angle_samples_[angle_group], estimate);
          AddSamples(refined_success_size_samples_[size_group], estimate);
        }
      }
    }
  }
  if (!summary_initialized_ || frame.sequence % 100 == 0) {
    summary_initialized_ = true;
    summary_sequence_ = frame.sequence;
    truth_summary_ = Summarize(truth_samples_);
    raw_summary_ = Summarize(raw_samples_);
    refined_success_summary_ = Summarize(refined_success_samples_);
    refined_with_fallback_summary_ = Summarize(refined_with_fallback_samples_);
    raw_distance_summaries_ = SummarizeGroups(raw_distance_samples_);
    refined_success_distance_summaries_ = SummarizeGroups(refined_success_distance_samples_);
    refined_with_fallback_distance_summaries_ =
        SummarizeGroups(refined_with_fallback_distance_samples_);
    raw_angle_summaries_ = SummarizeGroups(raw_angle_samples_);
    refined_success_angle_summaries_ = SummarizeGroups(refined_success_angle_samples_);
    refined_with_fallback_angle_summaries_ = SummarizeGroups(refined_with_fallback_angle_samples_);
    raw_size_summaries_ = SummarizeGroups(raw_size_samples_);
    refined_success_size_summaries_ = SummarizeGroups(refined_success_size_samples_);
    refined_with_fallback_size_summaries_ = SummarizeGroups(refined_with_fallback_size_samples_);
    refinement_summary_.elapsed_ms = Percentiles(refinement_elapsed_samples_);
    raw_solve_snapshot_ = raw_solve_summary_;
    refined_solve_snapshot_ = refined_solve_summary_;
    refinement_snapshot_ = refinement_summary_;
  }
  result.summary_sequence = summary_sequence_;
  result.ground_truth_summary = truth_summary_;
  result.detection_raw_summary = raw_summary_;
  result.detection_refined_success_summary = refined_success_summary_;
  result.detection_refined_with_fallback_summary = refined_with_fallback_summary_;
  result.raw_distance_groups = raw_distance_summaries_;
  result.refined_success_distance_groups = refined_success_distance_summaries_;
  result.refined_with_fallback_distance_groups = refined_with_fallback_distance_summaries_;
  result.raw_angle_groups = raw_angle_summaries_;
  result.refined_success_angle_groups = refined_success_angle_summaries_;
  result.refined_with_fallback_angle_groups = refined_with_fallback_angle_summaries_;
  result.raw_size_groups = raw_size_summaries_;
  result.refined_success_size_groups = refined_success_size_summaries_;
  result.refined_with_fallback_size_groups = refined_with_fallback_size_summaries_;
  result.raw_solve_summary = raw_solve_snapshot_;
  result.refined_solve_summary = refined_solve_snapshot_;
  result.refinement_summary = refinement_snapshot_;
  return result;
}

}  // namespace mv::modules
