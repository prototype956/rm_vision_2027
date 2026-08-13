#include "modules/armor_pnp/detail/truth_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <numbers>
#include <opencv2/imgproc.hpp>

namespace mv::modules::detail {
namespace {

constexpr double K_RAD_TO_DEG = 180.0 / std::numbers::pi;

bool LabelsMatch(ArmorLabel detection, std::uint8_t truth) {
  if (detection == ArmorLabel::BASE_SMALL || detection == ArmorLabel::BASE_BIG)
    return truth == 7;
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

}  // namespace

std::optional<std::array<cv::Point2f, 4>> ProjectVisibleTruth(
    const hal::CameraFrame::GroundTruthArmor& armor,
    const hal::CameraFrame::FrameGeometry& geometry) {
  const auto world_t_camera =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  const auto camera_t_world = geometry::Inverse(world_t_camera);
  const auto camera_t_armor = geometry::Compose(camera_t_world, armor.world_t_armor);
  // 背面装甲和相机后方角点不能形成有物理意义的 PnP 基准，投影前直接剔除。
  const auto normal_camera = geometry::TransformVector(camera_t_armor, geometry::Vector3::UnitZ());
  if (normal_camera.dot(camera_t_armor.translation) >= 0.0)
    return std::nullopt;
  std::array<cv::Point2f, 4> projected{};
  for (std::size_t index = 0; index < projected.size(); ++index) {
    const auto point = geometry::TransformPoint(camera_t_world, armor.corners_world[index]);
    if (point.z() <= 0.0)
      return std::nullopt;
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
  if ((bounds & image_bounds).empty())
    return std::nullopt;
  return projected;
}

std::vector<std::size_t> MatchDetectionsToTruth(std::span<const ArmorDetection> detections,
                                                std::span<const VisibleArmorTruth> truths,
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
      // IoU 权重为 2，中心和同索引角点距离共同消除相邻同标签目标的歧义。
      costs[row][column] = 2.0 * (1.0 - iou) + center_ratio + corner_ratio;
    }
  }

  // 每个检测附加一个虚拟列显式表示未匹配，真实真值列仍保持全局一对一。
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
  // 所有有符号误差统一定义为 estimate - truth，并在 camera_optical 坐标系表达。
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

}  // namespace mv::modules::detail
