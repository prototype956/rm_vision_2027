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

std::array<cv::Point3d, 4> ObjectPoints(geometry::ArmorType type, const ArmorPnpConfig& config) {
  const double WIDTH =
      type == geometry::ArmorType::LARGE ? config.large_width_m : config.small_width_m;
  const double HALF_WIDTH = WIDTH * 0.5;
  const double HALF_HEIGHT = config.height_m * 0.5;
  // armor 坐标系中的 TL、TR、BR、BL 顺序必须与检测角点顺序严格一致。
  return {cv::Point3d(-HALF_WIDTH, HALF_HEIGHT, 0.0), cv::Point3d(HALF_WIDTH, HALF_HEIGHT, 0.0),
          cv::Point3d(HALF_WIDTH, -HALF_HEIGHT, 0.0), cv::Point3d(-HALF_WIDTH, -HALF_HEIGHT, 0.0)};
}

cv::Matx33d CameraMatrix(const frame::CameraModel& value) {
  return {value.fx, 0.0, value.cx, 0.0, value.fy, value.cy, 0.0, 0.0, 1.0};
}

cv::Vec<double, 5> Distortion(const frame::CameraModel& value) {
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

ArmorPnpSolveResult SolveIppe(const ArmorPnpConfig& config,
                              std::span<const cv::Point2f, 4> image_corners,
                              geometry::ArmorType type, const frame::CameraModel& calibration,
                              std::size_t input_index, std::uint8_t label) {
  ArmorPnpSolveResult result;
  result.diagnostics.input_index = input_index;
  if (calibration.fx <= 0.0 || calibration.fy <= 0.0 ||
      !std::all_of(image_corners.begin(), image_corners.end(), Finite)) {
    return result;
  }
  const auto OBJECT_POINTS = ObjectPoints(type, config);
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  try {
    // 平面目标使用 IPPE 保留多个可能姿态，再由物理约束与重投影误差统一筛选。
    const int COUNT = cv::solvePnPGeneric(
        std::vector<cv::Point3d>(OBJECT_POINTS.begin(), OBJECT_POINTS.end()),
        std::vector<cv::Point2f>(image_corners.begin(), image_corners.end()),
        CameraMatrix(calibration), Distortion(calibration), rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    if (COUNT <= 0) {
      result.diagnostics.status = PnpStatus::NO_SOLUTION;
      return result;
    }
  } catch (const cv::Exception&) {
    result.diagnostics.status = PnpStatus::NO_SOLUTION;
    return result;
  }

  double best_rmse = std::numeric_limits<double>::infinity();
  double second_rmse = std::numeric_limits<double>::infinity();
  PnpStatus last_rejection = PnpStatus::NO_SOLUTION;
  std::optional<ArmorPoseEstimate> best;
  std::optional<ArmorPoseDiagnostics> best_diagnostics;
  for (std::size_t candidate = 0; candidate < rvecs.size(); ++candidate) {
    const auto POSE = ToTransform(rvecs[candidate], tvecs[candidate]);
    bool positive = true;
    for (const auto& object_point : OBJECT_POINTS) {
      const geometry::Vector3 POINT =
          geometry::TransformPoint(POSE, {object_point.x, object_point.y, object_point.z});
      positive = positive && POINT.z() > 0.0;
    }
    if (!positive) {
      last_rejection = PnpStatus::NEGATIVE_DEPTH;
      continue;
    }
    if (geometry::TransformVector(POSE, geometry::Vector3::UnitZ()).dot(POSE.translation) >= 0.0) {
      last_rejection = PnpStatus::BACK_FACING;
      continue;
    }
    const double DISTANCE = POSE.translation.norm();
    if (DISTANCE < config.min_distance_m || DISTANCE > config.max_distance_m) {
      last_rejection = PnpStatus::OUT_OF_RANGE;
      continue;
    }
    std::vector<cv::Point2d> projected;
    cv::projectPoints(std::vector<cv::Point3d>(OBJECT_POINTS.begin(), OBJECT_POINTS.end()),
                      rvecs[candidate], tvecs[candidate], CameraMatrix(calibration),
                      Distortion(calibration), projected);
    double squared_error = 0.0;
    for (std::size_t corner = 0; corner < projected.size(); ++corner) {
      const double DX = projected[corner].x - image_corners[corner].x;
      const double DY = projected[corner].y - image_corners[corner].y;
      squared_error += DX * DX + DY * DY;
    }
    const double RMSE = std::sqrt(squared_error / static_cast<double>(projected.size()));
    // 候选选择只依据同一输入四角下的 RMSE，次优间隔单独保留用于歧义诊断。
    if (RMSE >= best_rmse) {
      second_rmse = std::min(second_rmse, RMSE);
      continue;
    }
    second_rmse = best_rmse;
    ArmorPoseEstimate estimate{
        .input_index = input_index,
        .label = label,
        .type = type,
        .width_m = type == geometry::ArmorType::LARGE ? config.large_width_m : config.small_width_m,
        .height_m = config.height_m,
        .camera_t_armor = POSE};
    ArmorPoseDiagnostics diagnostics{
        .input_index = input_index,
        .candidate_index = candidate,
        .candidate_rmse_gap_px = std::nullopt,
        .reprojection_rmse_px = RMSE,
        .image_width_px = 0.5 * (cv::norm(image_corners[1] - image_corners[0]) +
                                 cv::norm(image_corners[2] - image_corners[3])),
        .image_height_px = 0.5 * (cv::norm(image_corners[3] - image_corners[0]) +
                                  cv::norm(image_corners[2] - image_corners[1])),
        .distance_m = DISTANCE,
        .viewing_angle_deg =
            std::acos(std::clamp(-geometry::TransformVector(POSE, geometry::Vector3::UnitZ())
                                      .dot(POSE.translation.normalized()),
                                 -1.0, 1.0)) *
            K_RAD_TO_DEG};
    std::copy(image_corners.begin(), image_corners.end(), diagnostics.image_corners.begin());
    for (std::size_t corner = 0; corner < projected.size(); ++corner)
      diagnostics.reprojected_corners[corner] = cv::Point2f(projected[corner]);
    best_rmse = RMSE;
    best = std::move(estimate);
    best_diagnostics = diagnostics;
  }
  if (!best) {
    result.diagnostics.status = last_rejection;
    return result;
  }
  result.diagnostics.status = PnpStatus::SUCCESS;
  if (std::isfinite(second_rmse))
    best_diagnostics->candidate_rmse_gap_px = second_rmse - best_rmse;
  result.output = std::move(best);
  result.diagnostics.pose = best_diagnostics;
  return result;
}

}  // namespace mv::modules::detail
