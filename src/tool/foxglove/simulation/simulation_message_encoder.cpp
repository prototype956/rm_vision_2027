#include "tool/foxglove/simulation/simulation_message_encoder.hpp"

#include "geometry/rigid_transform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <fmt/format.h>

namespace mv::tool::foxglove::simulation {
namespace {

::foxglove::schemas::Vector3 ToVector(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

::foxglove::schemas::Point3 ToPoint(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

}  // namespace

::foxglove::schemas::SceneUpdate EncodeGroundTruth(
    const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::SceneUpdate update;
  // 生命周期略长于常见发布周期，目标从真值消失后无需额外发送删除实体消息。
  const ::foxglove::schemas::Duration LIFETIME{.sec = 0, .nsec = 200'000'000};
  for (const auto& target : geometry.targets) {
    ::foxglove::schemas::SceneEntity entity;
    entity.timestamp = timestamp;
    entity.frame_id = "world";
    entity.id = fmt::format("robot_{}", target.id);
    entity.lifetime = LIFETIME;
    entity.metadata = {{.key = "team", .value = std::to_string(target.team)},
                       {.key = "armor_label", .value = std::to_string(target.armor_label)}};

    ::foxglove::schemas::SpherePrimitive center;
    center.pose =
        ::foxglove::schemas::Pose{.position = ToVector(target.position_world),
                                  .orientation = ::foxglove::schemas::Quaternion{.w = 1.0}};
    center.size = ::foxglove::schemas::Vector3{.x = 0.18, .y = 0.18, .z = 0.18};
    center.color = target.team == 0
                       ? ::foxglove::schemas::Color{.r = 1.0, .g = 0.1, .b = 0.1, .a = 0.8}
                       : ::foxglove::schemas::Color{.r = 0.1, .g = 0.3, .b = 1.0, .a = 0.8};
    entity.spheres.push_back(std::move(center));

    ::foxglove::schemas::LinePrimitive heading;
    heading.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    heading.thickness = 0.025;
    heading.color = ::foxglove::schemas::Color{.r = 1.0, .g = 1.0, .b = 1.0, .a = 1.0};
    // yaw 从 world +X 起绕 +Z 旋转；固定 0.5 米白线只表达方向，不表达速度。
    heading.points = {{.x = target.position_world.x(),
                       .y = target.position_world.y(),
                       .z = target.position_world.z()},
                      {.x = target.position_world.x() + 0.5 * std::cos(target.yaw),
                       .y = target.position_world.y() + 0.5 * std::sin(target.yaw),
                       .z = target.position_world.z()}};
    entity.lines.push_back(std::move(heading));
    update.entities.push_back(std::move(entity));
  }

  for (const auto& armor : geometry.armors) {
    ::foxglove::schemas::SceneEntity entity;
    entity.timestamp = timestamp;
    entity.frame_id = "world";
    entity.id = fmt::format("truth_armor_{}", armor.id);
    entity.lifetime = LIFETIME;
    entity.metadata = {
        {.key = "team", .value = std::to_string(armor.team)},
        {.key = "label", .value = std::to_string(armor.label)},
        {.key = "type",
         .value = armor.type == hal::CameraFrame::ArmorType::LARGE ? "large" : "small"}};
    ::foxglove::schemas::LinePrimitive outline;
    outline.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
    outline.thickness = 0.012;
    outline.color = ::foxglove::schemas::Color{.r = 1.0, .g = 0.85, .b = 0.0, .a = 1.0};
    for (const auto& corner : armor.corners_world)
      outline.points.push_back(ToPoint(corner));
    entity.lines.push_back(std::move(outline));

    ::foxglove::schemas::LinePrimitive axes;
    axes.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    axes.thickness = 0.008;
    const auto origin = armor.world_t_armor.translation;
    axes.points = {ToPoint(origin),
                   ToPoint(origin + geometry::TransformVector(armor.world_t_armor, {0.08, 0, 0})),
                   ToPoint(origin),
                   ToPoint(origin + geometry::TransformVector(armor.world_t_armor, {0, 0.08, 0})),
                   ToPoint(origin),
                   ToPoint(origin + geometry::TransformVector(armor.world_t_armor, {0, 0, 0.08}))};
    axes.colors = {{.r = 1.0, .a = 1.0}, {.r = 1.0, .a = 1.0}, {.g = 1.0, .a = 1.0},
                   {.g = 1.0, .a = 1.0}, {.b = 1.0, .a = 1.0}, {.b = 1.0, .a = 1.0}};
    entity.lines.push_back(std::move(axes));
    update.entities.push_back(std::move(entity));
  }
  return update;
}

::foxglove::schemas::ImageAnnotations EncodeProjectionAnnotations(
    const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  const auto WORLD_T_CAMERA =
      mv::geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  // 投影需要 camera_t_world，因此对同帧 world_t_camera 求逆后变换每个探针。
  const auto CAMERA_T_WORLD = mv::geometry::Inverse(WORLD_T_CAMERA);
  const auto& calibration = geometry.calibration;
  for (const auto& armor : geometry.armors) {
    const auto camera_t_armor = mv::geometry::Compose(CAMERA_T_WORLD, armor.world_t_armor);
    const auto normal_camera =
        mv::geometry::TransformVector(camera_t_armor, geometry::Vector3::UnitZ());
    if (normal_camera.dot(camera_t_armor.translation) >= 0.0) {
      continue;
    }
    ::foxglove::schemas::PointsAnnotation polygon;
    polygon.timestamp = timestamp;
    polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    polygon.outline_color = ::foxglove::schemas::Color{.r = 1.0, .g = 0.85, .b = 0.0, .a = 1.0};
    polygon.thickness = 3.0;
    bool visible = true;
    double min_u = std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (const auto& corner : armor.corners_world) {
      const auto camera_point = mv::geometry::TransformPoint(CAMERA_T_WORLD, corner);
      if (camera_point.z() <= 0.0) {
        visible = false;
        break;
      }
      const double u = calibration.fx * camera_point.x() / camera_point.z() + calibration.cx;
      const double v = calibration.fy * camera_point.y() / camera_point.z() + calibration.cy;
      min_u = std::min(min_u, u);
      min_v = std::min(min_v, v);
      max_u = std::max(max_u, u);
      max_v = std::max(max_v, v);
      polygon.points.push_back({.x = u, .y = v});
    }
    const bool intersects_image =
        max_u >= 0.0 && max_v >= 0.0 && min_u < calibration.width && min_v < calibration.height;
    if (visible && intersects_image)
      annotations.points.push_back(std::move(polygon));
  }
  return annotations;
}

}  // namespace mv::tool::foxglove::simulation
