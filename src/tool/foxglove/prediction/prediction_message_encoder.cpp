#include "tool/foxglove/prediction/prediction_message_encoder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#include <fmt/format.h>
#include <numbers>

namespace mv::tool::foxglove::prediction {
namespace {

::foxglove::schemas::Point3 Point(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

::foxglove::schemas::Vector3 Vector(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

::foxglove::schemas::Color HorizonColor(std::size_t index) {
  // 从当前到未来使用绿、青、紫、洋红，透明度随预测距离增加而降低。
  constexpr std::array<::foxglove::schemas::Color, 4> COLORS{
      ::foxglove::schemas::Color{.r = 0.1, .g = 1.0, .b = 0.2, .a = 1.0},
      ::foxglove::schemas::Color{.r = 0.1, .g = 0.8, .b = 1.0, .a = 0.9},
      ::foxglove::schemas::Color{.r = 0.7, .g = 0.3, .b = 1.0, .a = 0.8},
      ::foxglove::schemas::Color{.r = 1.0, .g = 0.2, .b = 0.8, .a = 0.7}};
  return COLORS[std::min(index, COLORS.size() - 1)];
}

std::string NumberArray(const auto& values) {
  std::string output = "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0)
      output += ',';
    output += fmt::format("{:.9g}", values[index]);
  }
  output += ']';
  return output;
}

void AddTimestampCarrier(::foxglove::schemas::ImageAnnotations& annotations,
                         const ::foxglove::schemas::Timestamp& timestamp) {
  // ImageAnnotations 没有顶层时间戳，用不可见点集承载帧时间并触发旧标注清除。
  ::foxglove::schemas::PointsAnnotation carrier;
  carrier.timestamp = timestamp;
  carrier.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
  carrier.thickness = 0.0;
  annotations.points.push_back(std::move(carrier));
}

std::optional<::foxglove::schemas::Point2> ProjectPoint(
    const geometry::Vector3& point_camera, const hal::CameraFrame::Calibration& calibration) {
  if (!point_camera.allFinite() || point_camera.z() <= 0.0)
    return std::nullopt;
  const double X = point_camera.x() / point_camera.z();
  const double Y = point_camera.y() / point_camera.z();
  const double R2 = X * X + Y * Y;
  const double R4 = R2 * R2;
  const double R6 = R4 * R2;
  const double K1 = calibration.distortion[0];
  const double K2 = calibration.distortion[1];
  const double P1 = calibration.distortion[2];
  const double P2 = calibration.distortion[3];
  const double K3 = calibration.distortion[4];
  // 与 HAL 标定契约一致，按 plumb_bob 的 k1,k2,p1,p2,k3 顺序应用畸变。
  const double RADIAL = 1.0 + K1 * R2 + K2 * R4 + K3 * R6;
  const double DISTORTED_X = X * RADIAL + 2.0 * P1 * X * Y + P2 * (R2 + 2.0 * X * X);
  const double DISTORTED_Y = Y * RADIAL + P1 * (R2 + 2.0 * Y * Y) + 2.0 * P2 * X * Y;
  const double U = calibration.fx * DISTORTED_X + calibration.cx;
  const double V = calibration.fy * DISTORTED_Y + calibration.cy;
  if (!std::isfinite(U) || !std::isfinite(V))
    return std::nullopt;
  return ::foxglove::schemas::Point2{.x = U, .y = V};
}

const modules::PredictionHorizon* FindHorizon(const modules::ArmorPredictionResult& result,
                                              ImagePredictionHorizon requested) {
  const double TARGET = requested == ImagePredictionHorizon::CURRENT ? 0.0 : 0.1;
  const auto FOUND = std::find_if(result.horizons.begin(), result.horizons.end(),
                                  [TARGET](const modules::PredictionHorizon& value) {
                                    return std::abs(value.seconds - TARGET) < 1.0e-9;
                                  });
  return FOUND == result.horizons.end() ? nullptr : &*FOUND;
}

}  // namespace

::foxglove::schemas::SceneUpdate EncodeScene(const modules::ArmorPredictionResult& result,
                                             const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::SceneUpdate update;
  if (result.state == modules::TrackerState::LOST || result.horizons.empty())
    return update;
  const ::foxglove::schemas::Duration LIFETIME{.sec = 0, .nsec = 200'000'000};
  const auto& current = result.horizons.front();
  // prediction_target 聚合当前中心、速度、双半径和本帧观测关联，便于联合诊断。
  ::foxglove::schemas::SceneEntity target;
  target.timestamp = timestamp;
  target.frame_id = "world";
  target.id = "prediction_target";
  target.lifetime = LIFETIME;
  target.metadata = {{.key = "state", .value = modules::TrackerStateName(result.state)}};
  ::foxglove::schemas::SpherePrimitive center;
  center.pose = ::foxglove::schemas::Pose{.position = Vector(current.center_world),
                                          .orientation = ::foxglove::schemas::Quaternion{.w = 1.0}};
  center.size = {.x = 0.12, .y = 0.12, .z = 0.12};
  center.color = {.r = 0.1, .g = 1.0, .b = 0.2, .a = 0.9};
  target.spheres.push_back(center);
  ::foxglove::schemas::LinePrimitive velocity;
  velocity.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
  velocity.thickness = 0.02;
  velocity.color = {.r = 0.2, .g = 1.0, .b = 0.2, .a = 1.0};
  velocity.points = {Point(current.center_world),
                     Point(current.center_world + result.velocity_world)};
  target.lines.push_back(std::move(velocity));

  const std::array<geometry::Vector3, 3> BODY_AXES{
      current.orientation_world * geometry::Vector3::UnitX(),
      current.orientation_world * geometry::Vector3::UnitY(),
      current.orientation_world * geometry::Vector3::UnitZ()};
  const std::array<::foxglove::schemas::Color, 3> AXIS_COLORS{
      ::foxglove::schemas::Color{.r = 1.0, .g = 0.1, .b = 0.1, .a = 1.0},
      ::foxglove::schemas::Color{.r = 0.1, .g = 1.0, .b = 0.1, .a = 1.0},
      ::foxglove::schemas::Color{.r = 0.1, .g = 0.3, .b = 1.0, .a = 1.0}};
  for (int axis = 0; axis < 3; ++axis) {
    ::foxglove::schemas::LinePrimitive line;
    line.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    line.thickness = 0.012;
    line.color = AXIS_COLORS[axis];
    line.points = {Point(current.center_world),
                   Point(current.center_world + 0.3 * BODY_AXES[axis])};
    target.lines.push_back(std::move(line));
  }

  for (int pair = 0; pair < 2; ++pair) {
    ::foxglove::schemas::LinePrimitive ring;
    ring.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
    ring.thickness = 0.006;
    ring.color = pair == 0 ? ::foxglove::schemas::Color{.r = 0.2, .g = 1.0, .b = 0.3, .a = 0.55}
                           : ::foxglove::schemas::Color{.r = 0.1, .g = 0.7, .b = 1.0, .a = 0.55};
    const double RADIUS = result.radii_m[pair];
    geometry::Vector3 ring_center = current.center_world;
    if (pair == 1)
      ring_center += result.height_offset_m * BODY_AXES[2];
    constexpr int SEGMENTS = 48;
    for (int index = 0; index < SEGMENTS; ++index) {
      const double ANGLE = 2.0 * std::numbers::pi * static_cast<double>(index) / SEGMENTS;
      ring.points.push_back(Point(ring_center + RADIUS * (std::cos(ANGLE) * BODY_AXES[0] +
                                                          std::sin(ANGLE) * BODY_AXES[1])));
    }
    target.lines.push_back(std::move(ring));
  }

  update.entities.push_back(std::move(target));

  const double WIDTH = result.type == hal::CameraFrame::ArmorType::LARGE ? 0.225 : 0.135;
  constexpr double HEIGHT = 0.055;
  for (std::size_t horizon_index = 0; horizon_index < result.horizons.size(); ++horizon_index) {
    // 每个时域使用稳定 entity id，Foxglove 可原位更新而不会留下历史拖影。
    const auto& horizon = result.horizons[horizon_index];
    ::foxglove::schemas::SceneEntity entity;
    entity.timestamp = timestamp;
    entity.frame_id = "world";
    entity.id = fmt::format("prediction_{:.0f}ms", horizon.seconds * 1000.0);
    entity.lifetime = LIFETIME;
    entity.metadata = {{.key = "horizon_s", .value = fmt::format("{:.3f}", horizon.seconds)}};
    for (const auto& armor : horizon.armors) {
      const auto& pose = armor.world_t_armor;
      const auto X_AXIS = geometry::TransformVector(pose, geometry::Vector3::UnitX());
      const auto Y_AXIS = geometry::TransformVector(pose, geometry::Vector3::UnitY());
      ::foxglove::schemas::LinePrimitive outline;
      outline.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
      outline.thickness = horizon_index == 0 ? 0.014 : 0.008;
      outline.color = HorizonColor(horizon_index);
      outline.points = {Point(pose.translation - X_AXIS * WIDTH * 0.5 + Y_AXIS * HEIGHT * 0.5),
                        Point(pose.translation + X_AXIS * WIDTH * 0.5 + Y_AXIS * HEIGHT * 0.5),
                        Point(pose.translation + X_AXIS * WIDTH * 0.5 - Y_AXIS * HEIGHT * 0.5),
                        Point(pose.translation - X_AXIS * WIDTH * 0.5 - Y_AXIS * HEIGHT * 0.5)};
      entity.lines.push_back(std::move(outline));
    }
    update.entities.push_back(std::move(entity));
  }
  return update;
}

std::string EncodeState(const modules::ArmorPredictionResult& result,
                        const ::foxglove::schemas::Timestamp& timestamp) {
  std::string associations = "[";
  for (std::size_t index = 0; index < result.associations.size(); ++index) {
    if (index != 0)
      associations += ',';
    const auto& value = result.associations[index];
    associations += fmt::format(
        "{{\"input_index\":{},\"slot\":{},\"candidate_slot\":{},\"accepted\":{},"
        "\"gate\":{:.9g},\"center_error_px\":{:.9g},"
        "\"edge_angle_error_rad\":{:.9g},\"perimeter_ratio_error\":{:.9g},"
        "\"total_cost\":{:.9g},\"rejection_reason\":\"{}\"}}",
        value.input_index, value.slot, value.candidate_slot, value.accepted, value.gate,
        value.center_error_px, value.edge_angle_error_rad, value.perimeter_ratio_error,
        value.total_cost, value.rejection_reason);
  }
  associations += ']';
  std::string lightbar_associations = "[";
  for (std::size_t index = 0; index < result.lightbar_associations.size(); ++index) {
    if (index != 0)
      lightbar_associations += ',';
    const auto& value = result.lightbar_associations[index];
    lightbar_associations += fmt::format(
        "{{\"input_index\":{},\"slot\":{},\"candidate_slot\":{},\"left\":{},"
        "\"candidate_left\":{},\"accepted\":{},\"duplicate_full_armor\":{},"
        "\"center_error_px\":{:.9g},\"endpoint_distance_ratio\":{:.9g},"
        "\"angle_error_rad\":{:.9g},\"log_length_error\":{:.9g},"
        "\"total_cost\":{:.9g},\"observed_top\":[{:.9g},{:.9g}],"
        "\"observed_bottom\":[{:.9g},{:.9g}],\"predicted_top\":[{:.9g},{:.9g}],"
        "\"predicted_bottom\":[{:.9g},{:.9g}],\"rejection_reason\":\"{}\"}}",
        value.input_index, value.slot, value.candidate_slot, value.left, value.candidate_left,
        value.accepted, value.duplicate_full_armor, value.center_error_px,
        value.endpoint_distance_ratio, value.angle_error_rad, value.log_length_error,
        value.total_cost, value.observed_top.x, value.observed_top.y, value.observed_bottom.x,
        value.observed_bottom.y, value.predicted_top.x, value.predicted_top.y,
        value.predicted_bottom.x, value.predicted_bottom.y, value.rejection_reason);
  }
  lightbar_associations += ']';
  const auto NIS = result.nis ? fmt::format("{:.9g}", *result.nis) : "null";
  const auto NIS_PER_DOF = result.nis_per_dof ? fmt::format("{:.9g}", *result.nis_per_dof) : "null";
  const auto TRIAL_YAW_UPDATE = result.trial_yaw_velocity_update_rad_s
                                    ? fmt::format("{:.9g}", *result.trial_yaw_velocity_update_rad_s)
                                    : "null";
  const auto CENTER_ERROR =
      result.truth_center_error_m ? fmt::format("{:.9g}", *result.truth_center_error_m) : "null";
  const auto YAW_ERROR =
      result.truth_yaw_error_rad ? fmt::format("{:.9g}", *result.truth_yaw_error_rad) : "null";
  const auto EQUIVALENT_YAW_ERROR =
      result.truth_yaw_equivalent_error_rad
          ? fmt::format("{:.9g}", *result.truth_yaw_equivalent_error_rad)
          : "null";
  const auto YAW_RATE_ERROR = result.truth_yaw_velocity_error_rad_s
                                  ? fmt::format("{:.9g}", *result.truth_yaw_velocity_error_rad_s)
                                  : "null";
  const int LABEL = result.label ? static_cast<int>(*result.label) : -1;
  return fmt::format(
      "{{\"timestamp\":{{\"sec\":{},\"nsec\":{}}},\"sequence\":{},"
      "\"tracker_state\":\"{}\",\"label\":{},\"dt_s\":{:.9g},"
      "\"state_order\":[\"cx\",\"vx\",\"cy\",\"vy\",\"cz\",\"vz\",\"rot_x\","
      "\"rot_y\",\"rot_z\",\"vyaw\",\"log_r1\",\"log_r2\",\"h\"],"
      "\"state\":{},\"covariance_diagonal\":{},\"innovation\":{},\"nis\":{},"
      "\"nis_per_dof\":{},"
      "\"iterations\":{},\"estimation_elapsed_ms\":{:.9g},"
      "\"radii_m\":{},\"height_offset_m\":{:.9g},\"yaw_variance_rad2\":{:.9g},"
      "\"truth_center_error_m\":{},\"truth_yaw_error_rad\":{},"
      "\"truth_yaw_equivalent_error_rad\":{},\"truth_yaw_velocity_error_rad_s\":{},"
      "\"maneuver_active\":{},\"maneuver_phase\":\"{}\","
      "\"maneuver_trigger\":\"{}\",\"maneuver_evidence_frames\":{},"
      "\"maneuver_evidence_cost\":{:.9g},"
      "\"maneuver_confirmation_remaining_s\":{:.9g},"
      "\"maneuver_remaining_s\":{:.9g},\"yaw_process_variance_used\":{:.9g},"
      "\"trial_yaw_velocity_update_rad_s\":{},\"association_gate_used\":{:.9g},"
      "\"accepted_association_count\":{},\"rejected_association_count\":{},"
      "\"associations\":{},\"lightbar_associations\":{},"
      "\"detected_lightbar_count\":{},\"deduplicated_lightbar_count\":{},"
      "\"matched_lightbar_count\":{},\"accepted_lightbar_count\":{},"
      "\"rejected_lightbar_count\":{},\"light_only_pair_count\":{},"
      "\"light_only_update\":{},\"light_only_update_blocked\":{},"
      "\"light_only_rejection_reason\":\"{}\","
      "\"light_fusion_used\":{},\"armor_fallback_used\":{},"
      "\"reset_count\":{},\"reset_reason\":\"{}\"}}",
      timestamp.sec, timestamp.nsec, result.sequence, modules::TrackerStateName(result.state),
      LABEL, result.dt_s, NumberArray(result.state_vector), NumberArray(result.covariance_diagonal),
      NumberArray(result.innovation), NIS, NIS_PER_DOF, result.esekf_iterations,
      result.estimation_elapsed_ms, NumberArray(result.radii_m), result.height_offset_m,
      result.yaw_variance_rad2, CENTER_ERROR, YAW_ERROR, EQUIVALENT_YAW_ERROR, YAW_RATE_ERROR,
      result.maneuver_active, result.maneuver_phase, result.maneuver_trigger,
      result.maneuver_evidence_frames, result.maneuver_evidence_cost,
      result.maneuver_confirmation_remaining_s, result.maneuver_remaining_s,
      result.yaw_process_variance_used, TRIAL_YAW_UPDATE, result.association_gate_used,
      result.accepted_association_count, result.rejected_association_count, associations,
      lightbar_associations, result.detected_lightbar_count, result.deduplicated_lightbar_count,
      result.matched_lightbar_count, result.accepted_lightbar_count, result.rejected_lightbar_count,
      result.light_only_pair_count, result.light_only_update, result.light_only_update_blocked,
      result.light_only_rejection_reason, result.light_fusion_used, result.armor_fallback_used,
      result.reset_count, result.reset_reason);
}

::foxglove::schemas::SceneUpdate EncodeTruthOverlay(
    const modules::ArmorPredictionResult& result, const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::SceneUpdate update;
  if (!result.label || result.horizons.empty())
    return update;
  const auto& center = result.horizons.front().center_world;
  const hal::CameraFrame::GroundTruthTarget* best = nullptr;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto& target : geometry.targets) {
    if (target.armor_label != static_cast<std::uint8_t>(*result.label))
      continue;
    // 仿真目标没有与检测稳定共享的 ID，因此在同标签集合中选择中心最近者用于展示。
    const double DISTANCE = (target.position_world - center).norm();
    if (DISTANCE < best_distance) {
      best = &target;
      best_distance = DISTANCE;
    }
  }
  if (!best)
    return update;
  ::foxglove::schemas::SceneEntity entity;
  entity.timestamp = timestamp;
  entity.frame_id = "world";
  entity.id = "prediction_truth_error";
  entity.lifetime = {.sec = 0, .nsec = 200'000'000};
  entity.metadata = {{.key = "center_error_m", .value = fmt::format("{:.6f}", best_distance)},
                     {.key = "truth_yaw_rate", .value = fmt::format("{:.6f}", best->yaw_velocity)}};
  ::foxglove::schemas::LinePrimitive error;
  error.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
  error.thickness = 0.018;
  error.color = {.r = 1.0, .g = 0.1, .b = 0.1, .a = 1.0};
  error.points = {Point(center), Point(best->position_world)};
  entity.lines.push_back(std::move(error));
  update.entities.push_back(std::move(entity));
  return update;
}

::foxglove::schemas::ImageAnnotations EncodeAnnotations(
    const modules::ArmorPredictionResult& result, const hal::CameraFrame::FrameGeometry& geometry,
    ImagePredictionHorizon requested, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  // ImageAnnotations 没有顶层时间戳；即使 LOST 也发布载体，让 Foxglove 清除上一帧框。
  AddTimestampCarrier(annotations, timestamp);
  if (result.state == modules::TrackerState::LOST || !result.type)
    return annotations;
  const auto* horizon = FindHorizon(result, requested);
  if (!horizon)
    return annotations;

  const auto WORLD_T_CAMERA =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  const auto CAMERA_T_WORLD = geometry::Inverse(WORLD_T_CAMERA);
  const auto& calibration = geometry.calibration;
  const double WIDTH = *result.type == hal::CameraFrame::ArmorType::LARGE ? 0.225 : 0.135;
  constexpr double HEIGHT = 0.055;
  const std::array<geometry::Vector3, 4> LOCAL_CORNERS{
      geometry::Vector3(-WIDTH * 0.5, HEIGHT * 0.5, 0.0),
      geometry::Vector3(WIDTH * 0.5, HEIGHT * 0.5, 0.0),
      geometry::Vector3(WIDTH * 0.5, -HEIGHT * 0.5, 0.0),
      geometry::Vector3(-WIDTH * 0.5, -HEIGHT * 0.5, 0.0)};
  const bool FUTURE = requested == ImagePredictionHorizon::FUTURE_100_MS;

  if (!FUTURE) {
    for (const auto& association : result.associations) {
      const auto ADD_OUTLINE = [&](const std::array<cv::Point2f, 4>& corners,
                                   const ::foxglove::schemas::Color& color, double thickness) {
        ::foxglove::schemas::PointsAnnotation polygon;
        polygon.timestamp = timestamp;
        polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
        polygon.outline_color = color;
        polygon.thickness = thickness;
        for (const auto& corner : corners)
          polygon.points.push_back({.x = corner.x, .y = corner.y});
        annotations.points.push_back(std::move(polygon));
      };
      ADD_OUTLINE(association.observed_corners, {.r = 1.0, .g = 0.65, .b = 0.0, .a = 1.0}, 2.0);
      if (association.candidate_slot >= 0) {
        const auto COLOR = association.accepted
                               ? ::foxglove::schemas::Color{.r = 0.0, .g = 0.8, .b = 1.0, .a = 1.0}
                               : ::foxglove::schemas::Color{.r = 1.0, .g = 0.1, .b = 0.8, .a = 1.0};
        ADD_OUTLINE(association.predicted_corners, COLOR, 2.0);
      }
    }
  }

  for (const auto& armor : horizon->armors) {
    const auto CAMERA_T_ARMOR = geometry::Compose(CAMERA_T_WORLD, armor.world_t_armor);
    const auto NORMAL_CAMERA =
        geometry::TransformVector(CAMERA_T_ARMOR, geometry::Vector3::UnitZ());
    // 法向与相机到装甲向量反向时为正面；背面仍以弱样式绘制以观察四槽位结构。
    const bool FRONT = NORMAL_CAMERA.dot(CAMERA_T_ARMOR.translation) < 0.0;
    std::array<::foxglove::schemas::Point2, 4> pixels{};
    double min_u = std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    bool valid = true;
    for (std::size_t index = 0; index < LOCAL_CORNERS.size(); ++index) {
      const auto CAMERA_POINT = geometry::TransformPoint(CAMERA_T_ARMOR, LOCAL_CORNERS[index]);
      const auto PROJECTED = ProjectPoint(CAMERA_POINT, calibration);
      if (!PROJECTED) {
        valid = false;
        break;
      }
      pixels[index] = *PROJECTED;
      min_u = std::min(min_u, PROJECTED->x);
      min_v = std::min(min_v, PROJECTED->y);
      max_u = std::max(max_u, PROJECTED->x);
      max_v = std::max(max_v, PROJECTED->y);
    }
    const bool INTERSECTS = max_u >= 0.0 && max_v >= 0.0 &&
                            min_u < static_cast<double>(calibration.width) &&
                            min_v < static_cast<double>(calibration.height);
    if (!valid || !INTERSECTS)
      continue;

    const double ALPHA = FRONT ? 1.0 : 0.35;
    const ::foxglove::schemas::Color COLOR =
        FUTURE ? ::foxglove::schemas::Color{.r = 0.75, .g = 0.25, .b = 1.0, .a = ALPHA}
               : ::foxglove::schemas::Color{.r = 0.1, .g = 1.0, .b = 0.2, .a = ALPHA};
    ::foxglove::schemas::PointsAnnotation polygon;
    polygon.timestamp = timestamp;
    polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    polygon.outline_color = COLOR;
    polygon.thickness = FRONT ? 3.0 : 1.5;
    polygon.points.assign(pixels.begin(), pixels.end());
    annotations.points.push_back(std::move(polygon));

    ::foxglove::schemas::TextAnnotation text;
    text.timestamp = timestamp;
    text.position = {.x = std::clamp(min_u, 0.0, static_cast<double>(calibration.width)),
                     .y = std::clamp(min_v - 3.0, 0.0, static_cast<double>(calibration.height))};
    text.text =
        fmt::format("slot {} {}ms {}", armor.slot, FUTURE ? 100 : 0, FRONT ? "front" : "back");
    text.font_size = 12.0;
    text.text_color = COLOR;
    text.background_color = {.a = FRONT ? 0.65 : 0.35};
    annotations.texts.push_back(std::move(text));
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeEmptyAnnotations(
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  AddTimestampCarrier(annotations, timestamp);
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeSelectedArmorAnnotations(
    const modules::ArmorPredictionResult& result, const hal::CameraFrame::FrameGeometry& geometry,
    const modules::ArmorSelectionSnapshot& selection,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  AddTimestampCarrier(annotations, timestamp);
  if (result.state == modules::TrackerState::LOST || !result.type)
    return annotations;
  const auto* horizon = FindHorizon(result, ImagePredictionHorizon::CURRENT);
  if (!horizon)
    return annotations;

  const auto WORLD_T_CAMERA =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  const auto CAMERA_T_WORLD = geometry::Inverse(WORLD_T_CAMERA);
  const double WIDTH = *result.type == hal::CameraFrame::ArmorType::LARGE ? 0.225 : 0.135;
  constexpr double HEIGHT = 0.055;
  const std::array<geometry::Vector3, 4> LOCAL_CORNERS{
      geometry::Vector3(-WIDTH * 0.5, HEIGHT * 0.5, 0.0),
      geometry::Vector3(WIDTH * 0.5, HEIGHT * 0.5, 0.0),
      geometry::Vector3(WIDTH * 0.5, -HEIGHT * 0.5, 0.0),
      geometry::Vector3(-WIDTH * 0.5, -HEIGHT * 0.5, 0.0)};

  auto add_slot = [&](int slot, bool selected) {
    if (slot < 0 || slot >= static_cast<int>(horizon->armors.size()))
      return;
    const auto CAMERA_T_ARMOR = geometry::Compose(
        CAMERA_T_WORLD, horizon->armors[static_cast<std::size_t>(slot)].world_t_armor);
    std::array<::foxglove::schemas::Point2, 4> pixels{};
    double min_u = std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < LOCAL_CORNERS.size(); ++index) {
      const auto POINT = geometry::TransformPoint(CAMERA_T_ARMOR, LOCAL_CORNERS[index]);
      const auto PROJECTED = ProjectPoint(POINT, geometry.calibration);
      if (!PROJECTED)
        return;
      pixels[index] = *PROJECTED;
      min_u = std::min(min_u, PROJECTED->x);
      min_v = std::min(min_v, PROJECTED->y);
      max_u = std::max(max_u, PROJECTED->x);
      max_v = std::max(max_v, PROJECTED->y);
    }
    if (max_u < 0.0 || max_v < 0.0 || min_u >= static_cast<double>(geometry.calibration.width) ||
        min_v >= static_cast<double>(geometry.calibration.height)) {
      return;
    }

    const ::foxglove::schemas::Color COLOR =
        selected ? ::foxglove::schemas::Color{.r = 0.1, .g = 1.0, .b = 0.2, .a = 1.0}
                 : ::foxglove::schemas::Color{.r = 1.0, .g = 0.8, .b = 0.0, .a = 1.0};
    ::foxglove::schemas::PointsAnnotation outline;
    outline.timestamp = timestamp;
    outline.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    outline.outline_color = COLOR;
    outline.thickness = selected ? 5.0 : 3.0;
    outline.points.assign(pixels.begin(), pixels.end());
    annotations.points.push_back(std::move(outline));

    const ::foxglove::schemas::Point2 CENTER{
        .x = 0.25 * (pixels[0].x + pixels[1].x + pixels[2].x + pixels[3].x),
        .y = 0.25 * (pixels[0].y + pixels[1].y + pixels[2].y + pixels[3].y)};
    ::foxglove::schemas::PointsAnnotation cross;
    cross.timestamp = timestamp;
    cross.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
    cross.outline_color = COLOR;
    cross.thickness = selected ? 4.0 : 2.0;
    cross.points = {{.x = CENTER.x - 8.0, .y = CENTER.y},
                    {.x = CENTER.x + 8.0, .y = CENTER.y},
                    {.x = CENTER.x, .y = CENTER.y - 8.0},
                    {.x = CENTER.x, .y = CENTER.y + 8.0}};
    annotations.points.push_back(std::move(cross));

    ::foxglove::schemas::TextAnnotation text;
    text.timestamp = timestamp;
    text.position = {
        .x = std::clamp(min_u, 0.0, static_cast<double>(geometry.calibration.width)),
        .y = std::clamp(min_v - 4.0, 0.0, static_cast<double>(geometry.calibration.height))};
    text.text = selected ? fmt::format("SELECTED slot {}", slot)
                         : fmt::format("PENDING slot {} {:.0f}/{:.0f}ms", slot,
                                       selection.pending_duration_s * 1.0e3,
                                       selection.switch_confirmation_s * 1.0e3);
    text.font_size = 14.0;
    text.text_color = COLOR;
    text.background_color = {.a = 0.7};
    annotations.texts.push_back(std::move(text));
  };

  add_slot(selection.selected_slot, true);
  if (selection.pending_slot != selection.selected_slot)
    add_slot(selection.pending_slot, false);
  return annotations;
}

}  // namespace mv::tool::foxglove::prediction
