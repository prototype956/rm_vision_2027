/**
 * @file detection_publisher.cpp
 * @brief DetectionPublisher 实现
 *
 * 2D（ImageAnnotations）：
 *   - 每块装甲板 → PointsAnnotation（LINE_LOOP），颜色区分 RED/BLUE
 *   - 标签文字   → TextAnnotation，显示 "R-1 120cm" 格式
 *
 * 3D（SceneUpdate）：
 *   - 仅 is_solved=true 的装甲板参与
 *   - CubePrimitive：按物理尺寸（SMALL/BIG）放置在云台坐标系
 *   - TextPrimitive：显示标签，位于方块正上方 0.05m 处
 */
#include "tool/foxglove/detail/detection_publisher.hpp"

#include "tool/foxglove/detail/utils.hpp"

#include <array>
#include <string>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace mv::tool::detail {

namespace {

std::array<cv::Point2f, 4> StableRectCorners(const mv::Detection& det) {
  std::array<cv::Point2f, 4> corners{};
  std::vector<cv::Point2f> points;
  points.reserve(4);
  for (const auto& point : det.points) {
    points.push_back(point);
  }
  const cv::RotatedRect rect = cv::minAreaRect(points);
  rect.points(corners.data());
  return corners;
}

cv::Point2f TopLeftByBounding(const std::array<cv::Point2f, 4>& corners) {
  float min_x = corners[0].x;
  float min_y = corners[0].y;
  for (const auto& point : corners) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
  }
  return {min_x, min_y};
}

foxglove::schemas::Quaternion FacingOriginQuaternion(const Eigen::Vector3d& point_in_gimbal) {
  if (point_in_gimbal.norm() < 1e-6) {
    return {0.0, 0.0, 0.0, 1.0};
  }

  const Eigen::Vector3d NORMAL_TO_GIMBAL = (-point_in_gimbal).normalized();
  const Eigen::Quaterniond Q =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), NORMAL_TO_GIMBAL);
  return {Q.x(), Q.y(), Q.z(), Q.w()};
}

}  // namespace

DetectionPublisher::DetectionPublisher(foxglove::Context ctx) : ctx_(std::move(ctx)) {}

void DetectionPublisher::EnsureChannels() {
  if (!annot_ch_.has_value()) {
    auto res = foxglove::schemas::ImageAnnotationsChannel::create("detections/annotations", ctx_);
    if (res.has_value()) {
      annot_ch_.emplace(std::move(res.value()));
    } else {
      spdlog::error("[DetectionPublisher] Failed to create annotations channel: {}",
                    foxglove::strerror(res.error()));
    }
  }
  if (!scene_ch_.has_value()) {
    auto res = foxglove::schemas::SceneUpdateChannel::create("detections/3d", ctx_);
    if (res.has_value()) {
      scene_ch_.emplace(std::move(res.value()));
    } else {
      spdlog::error("[DetectionPublisher] Failed to create 3d scene channel: {}",
                    foxglove::strerror(res.error()));
    }
  }
}

void DetectionPublisher::Publish(const std::vector<mv::Detection>& dets, uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);
  EnsureChannels();

  // ── 2D ImageAnnotations ──────────────────────────────────────────────────

  if (annot_ch_.has_value()) {
    foxglove::schemas::ImageAnnotations annot_msg;
    foxglove::schemas::Timestamp ts = ToTs(ts_ns);

    for (const auto& det : dets) {
      foxglove::schemas::Color box_color = ArmorColorToFox(det.color);
      const auto stable_corners = StableRectCorners(det);

      // 显示层使用稳定矩形框，避免端点抖动时显示成梯形。
      foxglove::schemas::PointsAnnotation box;
      box.timestamp = ts;
      box.type = foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
      box.thickness = 2.0;
      box.outline_color = box_color;
      for (const auto& point : stable_corners) {
        box.points.push_back({static_cast<double>(point.x), static_cast<double>(point.y)});
      }
      annot_msg.points.push_back(std::move(box));

      // 叠加原始角点，用于观察角点抖动，不影响稳定框观感。
      foxglove::schemas::PointsAnnotation raw_corners;
      raw_corners.timestamp = ts;
      raw_corners.type = foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
      raw_corners.thickness = 3.0;
      raw_corners.outline_color = {1.0F, 1.0F, 1.0F, 0.85F};
      for (const auto& point : det.points) {
        raw_corners.points.push_back({static_cast<double>(point.x), static_cast<double>(point.y)});
      }
      annot_msg.points.push_back(std::move(raw_corners));

      // 中心点（POINTS）
      foxglove::schemas::PointsAnnotation center_dot;
      center_dot.timestamp = ts;
      center_dot.type = foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
      center_dot.thickness = 5.0;
      center_dot.outline_color = ColorYellow();
      cv::Point2f ctr = det.Center();
      center_dot.points.push_back({static_cast<double>(ctr.x), static_cast<double>(ctr.y)});
      annot_msg.points.push_back(std::move(center_dot));

      // 标签文字
      foxglove::schemas::TextAnnotation label;
      label.timestamp = ts;
      label.text = ArmorLabel(det);
      label.font_size = 14.0;
      label.text_color = ColorWhite();
      label.background_color = {box_color.r, box_color.g, box_color.b, 0.5};
      // 位置：稳定矩形左上角上方 18px
      const cv::Point2f TOP_LEFT = TopLeftByBounding(stable_corners);
      label.position = foxglove::schemas::Point2{static_cast<double>(TOP_LEFT.x),
                                                 static_cast<double>(TOP_LEFT.y) - 18.0};
      annot_msg.texts.push_back(std::move(label));
    }

    annot_ch_->log(annot_msg, ts_ns);
  }

  // ── 3D SceneUpdate ───────────────────────────────────────────────────────

  if (scene_ch_.has_value()) {
    foxglove::schemas::SceneUpdate scene_msg;

    for (size_t i = 0; i < dets.size(); ++i) {
      const auto& det = dets[i];
      if (!det.is_solved) {
        continue;
      }

      foxglove::schemas::SceneEntity entity;
      entity.frame_id = "gimbal";
      entity.id = "armor_" + std::to_string(i);
      entity.timestamp = ToTs(ts_ns);
      entity.lifetime = foxglove::schemas::Duration{0, 300'000'000};  // 300ms 自动消失
      entity.frame_locked = false;

      const double x = det.xyz_in_gimbal.x();
      const double y = det.xyz_in_gimbal.y();
      const double z = det.xyz_in_gimbal.z();
      const auto ORIENTATION = FacingOriginQuaternion(det.xyz_in_gimbal);

      // 装甲板立方体（物理尺寸）
      foxglove::schemas::CubePrimitive cube;
      cube.pose = MakePose(x, y, z, ORIENTATION.x, ORIENTATION.y, ORIENTATION.z, ORIENTATION.w);
      double half_w = (det.type == mv::ArmorType::SMALL) ? 0.135 : 0.230;
      cube.size = foxglove::schemas::Vector3{half_w, 0.055, 0.01};
      // 设置颜色时先构造 Color 值再赋给 optional 字段
      auto cube_color = ArmorColorToFox(det.color);
      cube_color.a = 0.55;
      cube.color = cube_color;
      entity.cubes.push_back(std::move(cube));

      // 3D 标签文字（TextPrimitive 只有单 color 字段）
      foxglove::schemas::TextPrimitive text;
      text.pose = MakePose(x, y + 0.07, z, 0.0, 0.0, 0.0, 1.0);
      text.text = ArmorLabel(det);
      text.font_size = 0.04;
      text.billboard = true;  // 始终面朝摄像机
      text.color = ColorWhite();
      entity.texts.push_back(std::move(text));

      scene_msg.entities.push_back(std::move(entity));
    }

    scene_ch_->log(scene_msg, ts_ns);
  }
}

}  // namespace mv::tool::detail
