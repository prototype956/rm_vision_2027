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

void AddTruthErrors(ArmorPoseEstimate& estimate, const hal::CameraFrame::GroundTruthArmor& truth,
                    const hal::CameraFrame::FrameGeometry& geometry,
                    const std::array<cv::Point2f, 4>& truth_pixels) {
  const auto world_t_camera =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  const auto actual = geometry::Compose(geometry::Inverse(world_t_camera), truth.world_t_armor);
  estimate.truth_id = truth.id;
  estimate.position_error_m = (estimate.camera_t_armor.translation - actual.translation).norm();
  estimate.depth_error_m =
      std::abs(estimate.camera_t_armor.translation.z() - actual.translation.z());
  estimate.rotation_error_deg =
      estimate.camera_t_armor.rotation.angularDistance(actual.rotation) * K_RAD_TO_DEG;
  double sum = 0.0;
  for (std::size_t corner = 0; corner < truth_pixels.size(); ++corner) {
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
          .rotation_error_deg = Percentiles(samples.rotation)};
}

}  // namespace

ArmorPnpConfig ParseArmorPnpConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "armor PnP config";
  ConfigLoader::RejectUnknownKeys(root,
                                  {"schema_version", "small_width_m", "large_width_m", "height_m",
                                   "min_distance_m", "max_distance_m"},
                                  CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1) {
    throw ConfigError("armor PnP config schema_version must be 1");
  }
  ArmorPnpConfig config{
      .small_width_m = ConfigLoader::Require<double>(root, "small_width_m", CONTEXT),
      .large_width_m = ConfigLoader::Require<double>(root, "large_width_m", CONTEXT),
      .height_m = ConfigLoader::Require<double>(root, "height_m", CONTEXT),
      .min_distance_m = ConfigLoader::Require<double>(root, "min_distance_m", CONTEXT),
      .max_distance_m = ConfigLoader::Require<double>(root, "max_distance_m", CONTEXT)};
  if (!(config.small_width_m > 0.0 && config.large_width_m > config.small_width_m &&
        config.height_m > 0.0 && config.min_distance_m > 0.0 &&
        config.max_distance_m > config.min_distance_m)) {
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
  ArmorPnpAttempt result{.source = source, .input_index = input_index, .estimate = std::nullopt};
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
    if (rmse >= best_rmse)
      continue;
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
        .reprojection_rmse_px = rmse,
        .distance_m = distance,
        .viewing_angle_deg =
            std::acos(std::clamp(-geometry::TransformVector(pose, geometry::Vector3::UnitZ())
                                      .dot(pose.translation.normalized()),
                                 -1.0, 1.0)) *
            K_RAD_TO_DEG,
        .mean_corner_error_px = std::nullopt,
        .position_error_m = std::nullopt,
        .depth_error_m = std::nullopt,
        .rotation_error_deg = std::nullopt};
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
  result.estimate = std::move(best);
  return result;
}

ArmorPnpFrameResult ArmorPnp::ProcessFrame(const hal::CameraFrame& frame,
                                           std::span<const ArmorDetection> detections) {
  ArmorPnpFrameResult result;
  if (!frame.geometry)
    return result;
  const auto& geometry = *frame.geometry;
  struct VisibleTruth {
    const hal::CameraFrame::GroundTruthArmor* armor;
    std::array<cv::Point2f, 4> pixels;
  };
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

  std::vector<bool> truth_used(visible_truth.size(), false);
  for (std::size_t index = 0; index < detections.size(); ++index) {
    const auto& detection = detections[index];
    auto attempt =
        Solve(detection.corners, ArmorTypeForLabel(detection.label), geometry.calibration,
              PnpInputSource::DETECTION, index, static_cast<std::uint8_t>(detection.label));
    if (attempt.estimate) {
      std::size_t best_truth = visible_truth.size();
      double best_error = std::numeric_limits<double>::infinity();
      for (std::size_t truth_index = 0; truth_index < visible_truth.size(); ++truth_index) {
        const auto& truth = visible_truth[truth_index];
        if (truth_used[truth_index] || truth.armor->team != TeamForColor(detection.color) ||
            !LabelsMatch(detection.label, truth.armor->label) ||
            PolygonIntersection(detection.corners, truth.pixels) <= 0.0)
          continue;
        double error = 0.0;
        for (std::size_t corner = 0; corner < 4; ++corner) {
          error += cv::norm(detection.corners[corner] - truth.pixels[corner]);
        }
        if (error < best_error) {
          best_error = error;
          best_truth = truth_index;
        }
      }
      if (best_truth < visible_truth.size()) {
        truth_used[best_truth] = true;
        AddTruthErrors(*attempt.estimate, *visible_truth[best_truth].armor, geometry,
                       visible_truth[best_truth].pixels);
      }
    }
    result.attempts.push_back(std::move(attempt));
  }
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate)
      continue;
    auto& samples =
        attempt.source == PnpInputSource::GROUND_TRUTH ? truth_samples_ : detection_samples_;
    samples.reprojection.push_back(attempt.estimate->reprojection_rmse_px);
    if (attempt.estimate->mean_corner_error_px)
      samples.corner.push_back(*attempt.estimate->mean_corner_error_px);
    if (attempt.estimate->position_error_m)
      samples.position.push_back(*attempt.estimate->position_error_m);
    if (attempt.estimate->depth_error_m)
      samples.depth.push_back(*attempt.estimate->depth_error_m);
    if (attempt.estimate->rotation_error_deg)
      samples.rotation.push_back(*attempt.estimate->rotation_error_deg);
  }
  if (frame.sequence % 100 == 0) {
    truth_summary_ = Summarize(truth_samples_);
    detection_summary_ = Summarize(detection_samples_);
  }
  result.ground_truth_summary = truth_summary_;
  result.detection_summary = detection_summary_;
  return result;
}

}  // namespace mv::modules
