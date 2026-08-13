#include "modules/armor_pnp/detail/ippe_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <optional>

namespace mv::modules::detail {
namespace {

constexpr double K_RAD_TO_DEG = 180.0 / std::numbers::pi;

std::array<cv::Point3d, 4> ObjectPoints(hal::CameraFrame::ArmorType type,
                                        const ArmorPnpConfig& config) {
  const double width =
      type == hal::CameraFrame::ArmorType::LARGE ? config.large_width_m : config.small_width_m;
  const double half_width = width * 0.5;
  const double half_height = config.height_m * 0.5;
  // armor 坐标系中的 TL、TR、BR、BL 顺序必须与检测角点顺序严格一致。
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
    for (int column = 0; column < 3; ++column)
      eigen_rotation(row, column) = rotation.at<double>(row, column);
  }
  return {
      .translation = geometry::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)),
      .rotation = geometry::Quaternion(eigen_rotation).normalized()};
}

bool Finite(const cv::Point2f& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

}  // namespace

ArmorPnpAttempt SolveIppe(const ArmorPnpConfig& config,
                          std::span<const cv::Point2f, 4> image_corners,
                          hal::CameraFrame::ArmorType type,
                          const hal::CameraFrame::Calibration& calibration, PnpInputSource source,
                          std::size_t input_index, std::uint8_t label) {
  ArmorPnpAttempt result{.source = source,
                         .input_index = input_index,
                         .status = PnpStatus::INVALID_INPUT,
                         .estimate = std::nullopt,
                         .refinement = std::nullopt};
  if (calibration.fx <= 0.0 || calibration.fy <= 0.0 ||
      !std::all_of(image_corners.begin(), image_corners.end(), Finite)) {
    return result;
  }
  const auto object_points = ObjectPoints(type, config);
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  try {
    // 平面目标使用 IPPE 保留多个可能姿态，再由物理约束与重投影误差统一筛选。
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
    if (distance < config.min_distance_m || distance > config.max_distance_m) {
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
    // 候选选择只依据同一输入四角下的 RMSE，次优间隔单独保留用于歧义诊断。
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
        .width_m = type == hal::CameraFrame::ArmorType::LARGE ? config.large_width_m
                                                              : config.small_width_m,
        .height_m = config.height_m,
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
    for (std::size_t corner = 0; corner < projected.size(); ++corner)
      estimate.reprojected_corners[corner] = cv::Point2f(projected[corner]);
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

}  // namespace mv::modules::detail
