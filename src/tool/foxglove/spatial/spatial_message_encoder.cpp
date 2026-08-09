#include "tool/foxglove/spatial/spatial_message_encoder.hpp"

#include <string>
#include <utility>

namespace mv::tool::foxglove::spatial {
namespace {

::foxglove::schemas::Vector3 ToVector(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

::foxglove::schemas::Quaternion ToQuaternion(const geometry::Quaternion& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z(), .w = value.w()};
}

::foxglove::schemas::FrameTransform MakeTransform(std::string parent, std::string child,
                                                  const geometry::RigidTransform& transform,
                                                  const ::foxglove::schemas::Timestamp& timestamp) {
  return {.timestamp = timestamp,
          .parent_frame_id = std::move(parent),
          .child_frame_id = std::move(child),
          .translation = ToVector(transform.translation),
          .rotation = ToQuaternion(transform.rotation)};
}

}  // namespace

::foxglove::schemas::FrameTransforms EncodeTransforms(
    const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::FrameTransforms message;
  // FrameGeometry 使用 parent_t_child 命名，方向可直接映射为 Foxglove parent/child。
  message.transforms.push_back(
      MakeTransform("world", "gimbal", geometry.world_t_gimbal, timestamp));
  message.transforms.push_back(
      MakeTransform("gimbal", "camera_optical", geometry.gimbal_t_camera_optical, timestamp));
  return message;
}

::foxglove::schemas::CameraCalibration EncodeCalibration(
    const hal::CameraFrame::Calibration& calibration,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::CameraCalibration message;
  message.timestamp = timestamp;
  message.frame_id = "camera_optical";
  message.width = calibration.width;
  message.height = calibration.height;
  message.distortion_model = "plumb_bob";
  message.d.assign(calibration.distortion.begin(), calibration.distortion.end());
  message.k = {
      calibration.fx, 0.0, calibration.cx, 0.0, calibration.fy, calibration.cy, 0.0, 0.0, 1.0};
  message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  message.p = {calibration.fx,
               0.0,
               calibration.cx,
               0.0,
               0.0,
               calibration.fy,
               calibration.cy,
               0.0,
               0.0,
               0.0,
               1.0,
               0.0};
  return message;
}

::foxglove::schemas::SceneUpdate EncodeFrustum(const hal::CameraFrame::Calibration& calibration,
                                               const ::foxglove::schemas::Timestamp& timestamp) {
  // 用针孔模型反投影四个图像角点，固定深度只影响显示尺寸，不改变视场角。
  constexpr double DEPTH = 1.0;
  const auto RAY = [&](double u, double v) {
    return ::foxglove::schemas::Point3{.x = (u - calibration.cx) / calibration.fx * DEPTH,
                                       .y = (v - calibration.cy) / calibration.fy * DEPTH,
                                       .z = DEPTH};
  };
  const ::foxglove::schemas::Point3 ORIGIN{};
  const auto TOP_LEFT = RAY(0.0, 0.0);
  const auto TOP_RIGHT = RAY(calibration.width, 0.0);
  const auto BOTTOM_RIGHT = RAY(calibration.width, calibration.height);
  const auto BOTTOM_LEFT = RAY(0.0, calibration.height);

  ::foxglove::schemas::LinePrimitive lines;
  lines.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
  lines.thickness = 0.01;
  lines.color = ::foxglove::schemas::Color{.r = 0.1, .g = 1.0, .b = 0.2, .a = 0.9};
  lines.points = {ORIGIN,       TOP_LEFT,    ORIGIN,      TOP_RIGHT, ORIGIN,    BOTTOM_RIGHT,
                  ORIGIN,       BOTTOM_LEFT, TOP_LEFT,    TOP_RIGHT, TOP_RIGHT, BOTTOM_RIGHT,
                  BOTTOM_RIGHT, BOTTOM_LEFT, BOTTOM_LEFT, TOP_LEFT};

  ::foxglove::schemas::SceneEntity entity;
  entity.timestamp = timestamp;
  entity.frame_id = "camera_optical";
  entity.id = "camera_frustum";
  entity.frame_locked = true;
  entity.lines.push_back(std::move(lines));
  ::foxglove::schemas::SceneUpdate update;
  update.entities.push_back(std::move(entity));
  return update;
}

}  // namespace mv::tool::foxglove::spatial
