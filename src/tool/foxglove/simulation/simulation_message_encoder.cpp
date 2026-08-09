#include "tool/foxglove/simulation/simulation_message_encoder.hpp"

#include "geometry/rigid_transform.hpp"

#include <cmath>
#include <string>
#include <utility>

#include <fmt/format.h>

namespace mv::tool::foxglove::simulation {
namespace {

::foxglove::schemas::Vector3 ToVector(const geometry::Vector3& value) {
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

  for (const auto& probe : geometry.projection_probes) {
    ::foxglove::schemas::SceneEntity entity;
    entity.timestamp = timestamp;
    entity.frame_id = "world";
    entity.id = fmt::format("projection_probe_{}", probe.id);
    entity.lifetime = LIFETIME;
    ::foxglove::schemas::SpherePrimitive marker;
    marker.pose =
        ::foxglove::schemas::Pose{.position = ToVector(probe.position_world),
                                  .orientation = ::foxglove::schemas::Quaternion{.w = 1.0}};
    marker.size = ::foxglove::schemas::Vector3{.x = 0.04, .y = 0.04, .z = 0.04};
    marker.color = ::foxglove::schemas::Color{.r = 1.0, .g = 0.9, .b = 0.0, .a = 1.0};
    entity.spheres.push_back(std::move(marker));
    update.entities.push_back(std::move(entity));
  }
  return update;
}

::foxglove::schemas::ImageAnnotations EncodeProjectionAnnotations(
    const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  ::foxglove::schemas::PointsAnnotation points;
  points.timestamp = timestamp;
  points.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
  points.outline_color = ::foxglove::schemas::Color{.r = 1.0, .g = 0.9, .b = 0.0, .a = 1.0};
  points.thickness = 7.0;

  const auto WORLD_T_CAMERA =
      mv::geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  // 投影需要 camera_t_world，因此对同帧 world_t_camera 求逆后变换每个探针。
  const auto CAMERA_T_WORLD = mv::geometry::Inverse(WORLD_T_CAMERA);
  const auto& calibration = geometry.calibration;
  for (const auto& probe : geometry.projection_probes) {
    const auto CAMERA_POINT = mv::geometry::TransformPoint(CAMERA_T_WORLD, probe.position_world);
    if (CAMERA_POINT.z() <= 0.0) {
      continue;
    }
    const double U = calibration.fx * CAMERA_POINT.x() / CAMERA_POINT.z() + calibration.cx;
    const double V = calibration.fy * CAMERA_POINT.y() / CAMERA_POINT.z() + calibration.cy;
    if (U >= 0.0 && V >= 0.0 && U < calibration.width && V < calibration.height) {
      points.points.push_back({.x = U, .y = V});
    }
  }
  annotations.points.push_back(std::move(points));
  return annotations;
}

}  // namespace mv::tool::foxglove::simulation
