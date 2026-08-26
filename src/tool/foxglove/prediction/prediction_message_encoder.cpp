#include "tool/foxglove/prediction/prediction_message_encoder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>

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

std::optional<::foxglove::schemas::Point2> ProjectPoint(const geometry::Vector3& point_camera,
                                                        const frame::CameraModel& calibration) {
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

const modules::PredictionHorizon* FindCurrentHorizon(const modules::ArmorPredictionOutput& output) {
  constexpr double TARGET = 0.0;
  const auto FOUND = std::find_if(output.horizons.begin(), output.horizons.end(),
                                  [TARGET](const modules::PredictionHorizon& value) {
                                    return std::abs(value.seconds - TARGET) < 1.0e-9;
                                  });
  return FOUND == output.horizons.end() ? nullptr : &*FOUND;
}

::foxglove::schemas::LinePrimitive ArmorOutline(const geometry::RigidTransform& world_t_armor,
                                                geometry::ArmorType type,
                                                const ::foxglove::schemas::Color& color,
                                                double thickness) {
  const double WIDTH = type == geometry::ArmorType::LARGE ? 0.225 : 0.135;
  constexpr double HEIGHT = 0.055;
  const auto X_AXIS = geometry::TransformVector(world_t_armor, geometry::Vector3::UnitX());
  const auto Y_AXIS = geometry::TransformVector(world_t_armor, geometry::Vector3::UnitY());
  ::foxglove::schemas::LinePrimitive outline;
  outline.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
  outline.thickness = thickness;
  outline.color = color;
  outline.points = {
      Point(world_t_armor.translation - X_AXIS * WIDTH * 0.5 + Y_AXIS * HEIGHT * 0.5),
      Point(world_t_armor.translation + X_AXIS * WIDTH * 0.5 + Y_AXIS * HEIGHT * 0.5),
      Point(world_t_armor.translation + X_AXIS * WIDTH * 0.5 - Y_AXIS * HEIGHT * 0.5),
      Point(world_t_armor.translation - X_AXIS * WIDTH * 0.5 - Y_AXIS * HEIGHT * 0.5)};
  return outline;
}

struct ProjectedArmor {
  std::array<::foxglove::schemas::Point2, 4> pixels{};
  bool front_facing{false};
};

std::optional<ProjectedArmor> ProjectArmor(const geometry::RigidTransform& world_t_armor,
                                           geometry::ArmorType type,
                                           const frame::SpatialFrameView& spatial) {
  const auto WORLD_T_CAMERA =
      geometry::Compose(spatial.world_t_gimbal, spatial.gimbal_t_camera_optical);
  const auto CAMERA_T_ARMOR = geometry::Compose(geometry::Inverse(WORLD_T_CAMERA), world_t_armor);
  const double WIDTH = type == geometry::ArmorType::LARGE ? 0.225 : 0.135;
  constexpr double HEIGHT = 0.055;
  const std::array<geometry::Vector3, 4> LOCAL_CORNERS{
      geometry::Vector3(-WIDTH * 0.5, HEIGHT * 0.5, 0.0),
      geometry::Vector3(WIDTH * 0.5, HEIGHT * 0.5, 0.0),
      geometry::Vector3(WIDTH * 0.5, -HEIGHT * 0.5, 0.0),
      geometry::Vector3(-WIDTH * 0.5, -HEIGHT * 0.5, 0.0)};
  ProjectedArmor projected;
  const auto NORMAL_CAMERA = geometry::TransformVector(CAMERA_T_ARMOR, geometry::Vector3::UnitZ());
  projected.front_facing = NORMAL_CAMERA.dot(CAMERA_T_ARMOR.translation) < 0.0;
  double min_u = std::numeric_limits<double>::infinity();
  double min_v = std::numeric_limits<double>::infinity();
  double max_u = -std::numeric_limits<double>::infinity();
  double max_v = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < LOCAL_CORNERS.size(); ++index) {
    const auto CAMERA_POINT = geometry::TransformPoint(CAMERA_T_ARMOR, LOCAL_CORNERS[index]);
    const auto PIXEL = ProjectPoint(CAMERA_POINT, spatial.calibration);
    if (!PIXEL)
      return std::nullopt;
    projected.pixels[index] = *PIXEL;
    min_u = std::min(min_u, PIXEL->x);
    min_v = std::min(min_v, PIXEL->y);
    max_u = std::max(max_u, PIXEL->x);
    max_v = std::max(max_v, PIXEL->y);
  }
  const bool INTERSECTS = max_u >= 0.0 && max_v >= 0.0 &&
                          min_u < static_cast<double>(spatial.calibration.width) &&
                          min_v < static_cast<double>(spatial.calibration.height);
  return INTERSECTS ? std::optional(projected) : std::nullopt;
}

}  // namespace

::foxglove::schemas::SceneUpdate EncodeScene(const modules::ArmorPredictionOutput& output,
                                             const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::SceneUpdate update;
  const auto* current = FindCurrentHorizon(output);
  if (output.state == modules::TrackerState::LOST || !current)
    return update;
  const ::foxglove::schemas::Duration LIFETIME{.sec = 0, .nsec = 200'000'000};
  // prediction_target 聚合当前中心、速度、车体轴和双半径，便于联合诊断。
  ::foxglove::schemas::SceneEntity target;
  target.timestamp = timestamp;
  target.frame_id = "world";
  target.id = "prediction_target";
  target.lifetime = LIFETIME;
  target.metadata = {{.key = "state", .value = modules::TrackerStateName(output.state)}};
  ::foxglove::schemas::SpherePrimitive center;
  center.pose = ::foxglove::schemas::Pose{.position = Vector(current->center_world),
                                          .orientation = ::foxglove::schemas::Quaternion{.w = 1.0}};
  center.size = {.x = 0.12, .y = 0.12, .z = 0.12};
  center.color = {.r = 0.1, .g = 1.0, .b = 0.2, .a = 0.9};
  target.spheres.push_back(center);
  ::foxglove::schemas::LinePrimitive velocity;
  velocity.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
  velocity.thickness = 0.02;
  velocity.color = {.r = 0.2, .g = 1.0, .b = 0.2, .a = 1.0};
  velocity.points = {Point(current->center_world),
                     Point(current->center_world + output.velocity_world)};
  target.lines.push_back(std::move(velocity));

  const std::array<geometry::Vector3, 3> BODY_AXES{
      current->orientation_world * geometry::Vector3::UnitX(),
      current->orientation_world * geometry::Vector3::UnitY(),
      current->orientation_world * geometry::Vector3::UnitZ()};
  const std::array<::foxglove::schemas::Color, 3> AXIS_COLORS{
      ::foxglove::schemas::Color{.r = 1.0, .g = 0.1, .b = 0.1, .a = 1.0},
      ::foxglove::schemas::Color{.r = 0.1, .g = 1.0, .b = 0.1, .a = 1.0},
      ::foxglove::schemas::Color{.r = 0.1, .g = 0.3, .b = 1.0, .a = 1.0}};
  for (int axis = 0; axis < 3; ++axis) {
    ::foxglove::schemas::LinePrimitive line;
    line.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    line.thickness = 0.012;
    line.color = AXIS_COLORS[axis];
    line.points = {Point(current->center_world),
                   Point(current->center_world + 0.3 * BODY_AXES[axis])};
    target.lines.push_back(std::move(line));
  }

  for (int pair = 0; pair < 2; ++pair) {
    ::foxglove::schemas::LinePrimitive ring;
    ring.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
    ring.thickness = 0.006;
    ring.color = pair == 0 ? ::foxglove::schemas::Color{.r = 0.2, .g = 1.0, .b = 0.3, .a = 0.55}
                           : ::foxglove::schemas::Color{.r = 0.1, .g = 0.7, .b = 1.0, .a = 0.55};
    const double RADIUS = output.radii_m[pair];
    geometry::Vector3 ring_center = current->center_world;
    if (pair == 1)
      ring_center += output.height_offset_m * BODY_AXES[2];
    constexpr int SEGMENTS = 48;
    for (int index = 0; index < SEGMENTS; ++index) {
      const double ANGLE = 2.0 * std::numbers::pi * static_cast<double>(index) / SEGMENTS;
      ring.points.push_back(Point(ring_center + RADIUS * (std::cos(ANGLE) * BODY_AXES[0] +
                                                          std::sin(ANGLE) * BODY_AXES[1])));
    }
    target.lines.push_back(std::move(ring));
  }

  update.entities.push_back(std::move(target));

  if (!output.type)
    return update;

  ::foxglove::schemas::SceneEntity armors;
  armors.timestamp = timestamp;
  armors.frame_id = "world";
  armors.id = "prediction_0ms";
  armors.lifetime = LIFETIME;
  armors.metadata = {{.key = "horizon_s", .value = "0.000"}};
  constexpr ::foxglove::schemas::Color CURRENT_COLOR{.r = 0.1, .g = 1.0, .b = 0.2, .a = 1.0};
  for (const auto& armor : current->armors) {
    armors.lines.push_back(ArmorOutline(armor.world_t_armor, *output.type, CURRENT_COLOR, 0.014));
  }
  update.entities.push_back(std::move(armors));
  return update;
}

::foxglove::schemas::SceneUpdate EncodeImpactScene(
    const modules::ArmorPredictionOutput& output, const modules::ArmorImpactSnapshot* impact,
    const ::foxglove::schemas::Timestamp& timestamp) {
  constexpr std::string_view ENTITY_ID = "impact_prediction";
  ::foxglove::schemas::SceneUpdate update;
  const bool VALID = impact && output.state != modules::TrackerState::LOST &&
                     impact->source_sequence == output.sequence && impact->ballistic.valid &&
                     output.type && impact->selected_slot >= 0 && impact->selected_slot < 4 &&
                     std::isfinite(impact->ballistic.prediction_horizon_s) &&
                     impact->ballistic.prediction_horizon_s >= 0.0;
  if (!VALID) {
    update.deletions.push_back(
        {.timestamp = timestamp,
         .type = ::foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::MATCHING_ID,
         .id = std::string(ENTITY_ID)});
    return update;
  }

  const auto HORIZON =
      modules::ExtrapolatePrediction(output, impact->ballistic.prediction_horizon_s);
  ::foxglove::schemas::SceneEntity entity;
  entity.timestamp = timestamp;
  entity.frame_id = "world";
  entity.id = std::string(ENTITY_ID);
  entity.lifetime = {.sec = 0, .nsec = 200'000'000};
  entity.metadata = {
      {.key = "horizon_s", .value = fmt::format("{:.6f}", impact->ballistic.prediction_horizon_s)},
      {.key = "selected_slot", .value = std::to_string(impact->selected_slot)}};
  constexpr ::foxglove::schemas::Color OTHER_COLOR{.r = 0.75, .g = 0.25, .b = 1.0, .a = 0.35};
  constexpr ::foxglove::schemas::Color SELECTED_COLOR{.r = 0.75, .g = 0.25, .b = 1.0, .a = 1.0};
  for (std::size_t slot = 0; slot < HORIZON.armors.size(); ++slot) {
    if (static_cast<int>(slot) == impact->selected_slot)
      continue;
    entity.lines.push_back(
        ArmorOutline(HORIZON.armors[slot].world_t_armor, *output.type, OTHER_COLOR, 0.008));
  }
  const auto SELECTED_SLOT = static_cast<std::size_t>(impact->selected_slot);
  entity.lines.push_back(ArmorOutline(HORIZON.armors[SELECTED_SLOT].world_t_armor, *output.type,
                                      SELECTED_COLOR, 0.018));
  update.entities.push_back(std::move(entity));
  return update;
}

std::string EncodeState(const modules::ArmorPredictionOutput& output,
                        const modules::ArmorPredictionDiagnostics& diagnostics,
                        const simulation_evaluation::PredictionEvaluationResult* evaluation,
                        const ::foxglove::schemas::Timestamp& timestamp) {
  std::string associations = "[";
  for (std::size_t index = 0; index < diagnostics.associations.size(); ++index) {
    if (index != 0)
      associations += ',';
    const auto& value = diagnostics.associations[index];
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
  for (std::size_t index = 0; index < diagnostics.lightbar_associations.size(); ++index) {
    if (index != 0)
      lightbar_associations += ',';
    const auto& value = diagnostics.lightbar_associations[index];
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
  const auto NIS = diagnostics.nis ? fmt::format("{:.9g}", *diagnostics.nis) : "null";
  const auto NIS_PER_DOF =
      diagnostics.nis_per_dof ? fmt::format("{:.9g}", *diagnostics.nis_per_dof) : "null";
  const auto TRIAL_YAW_UPDATE =
      diagnostics.trial_yaw_velocity_update_rad_s
          ? fmt::format("{:.9g}", *diagnostics.trial_yaw_velocity_update_rad_s)
          : "null";
  const auto CENTER_ERROR = evaluation && evaluation->center_error_m
                                ? fmt::format("{:.9g}", *evaluation->center_error_m)
                                : "null";
  const auto YAW_ERROR = evaluation && evaluation->yaw_error_rad
                             ? fmt::format("{:.9g}", *evaluation->yaw_error_rad)
                             : "null";
  const auto EQUIVALENT_YAW_ERROR =
      evaluation && evaluation->yaw_equivalent_error_rad
          ? fmt::format("{:.9g}", *evaluation->yaw_equivalent_error_rad)
          : "null";
  const auto YAW_RATE_ERROR = evaluation && evaluation->yaw_velocity_error_rad_s
                                  ? fmt::format("{:.9g}", *evaluation->yaw_velocity_error_rad_s)
                                  : "null";
  const int LABEL = output.label ? static_cast<int>(*output.label) : -1;
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
      timestamp.sec, timestamp.nsec, output.sequence, modules::TrackerStateName(output.state),
      LABEL, diagnostics.dt_s, NumberArray(output.state_vector),
      NumberArray(output.covariance_diagonal), NumberArray(diagnostics.innovation), NIS,
      NIS_PER_DOF, diagnostics.esekf_iterations, diagnostics.estimation_elapsed_ms,
      NumberArray(output.radii_m), output.height_offset_m, output.yaw_variance_rad2, CENTER_ERROR,
      YAW_ERROR, EQUIVALENT_YAW_ERROR, YAW_RATE_ERROR, diagnostics.maneuver_active,
      diagnostics.maneuver_phase, diagnostics.maneuver_trigger,
      diagnostics.maneuver_evidence_frames, diagnostics.maneuver_evidence_cost,
      diagnostics.maneuver_confirmation_remaining_s, diagnostics.maneuver_remaining_s,
      diagnostics.yaw_process_variance_used, TRIAL_YAW_UPDATE, diagnostics.association_gate_used,
      diagnostics.accepted_association_count, diagnostics.rejected_association_count, associations,
      lightbar_associations, diagnostics.detected_lightbar_count,
      diagnostics.deduplicated_lightbar_count, diagnostics.matched_lightbar_count,
      diagnostics.accepted_lightbar_count, diagnostics.rejected_lightbar_count,
      diagnostics.light_only_pair_count, diagnostics.light_only_update,
      diagnostics.light_only_update_blocked, diagnostics.light_only_rejection_reason,
      diagnostics.light_fusion_used, diagnostics.armor_fallback_used, diagnostics.reset_count,
      diagnostics.reset_reason);
}

::foxglove::schemas::SceneUpdate EncodeTruthOverlay(
    const modules::ArmorPredictionOutput& output,
    const simulation_evaluation::PredictionEvaluationResult& evaluation,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::SceneUpdate update;
  if (!output.label || output.horizons.empty() || !evaluation.truth_position_world)
    return update;
  const auto& center = output.horizons.front().center_world;
  const double CENTER_ERROR = evaluation.center_error_m.value_or(0.0);
  ::foxglove::schemas::SceneEntity entity;
  entity.timestamp = timestamp;
  entity.frame_id = "world";
  entity.id = "prediction_truth_error";
  entity.lifetime = {.sec = 0, .nsec = 200'000'000};
  entity.metadata = {
      {.key = "center_error_m", .value = fmt::format("{:.6f}", CENTER_ERROR)},
      {.key = "truth_yaw_rate",
       .value = fmt::format("{:.6f}", evaluation.truth_yaw_velocity_rad_s.value_or(0.0))}};
  ::foxglove::schemas::LinePrimitive error;
  error.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
  error.thickness = 0.018;
  error.color = {.r = 1.0, .g = 0.1, .b = 0.1, .a = 1.0};
  error.points = {Point(center), Point(*evaluation.truth_position_world)};
  entity.lines.push_back(std::move(error));
  update.entities.push_back(std::move(entity));
  return update;
}

::foxglove::schemas::ImageAnnotations EncodeCurrentAnnotations(
    const modules::ArmorPredictionOutput& output,
    const modules::ArmorPredictionDiagnostics& diagnostics, const frame::SpatialFrameView* spatial,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  AddTimestampCarrier(annotations, timestamp);
  if (output.state == modules::TrackerState::LOST)
    return annotations;

  // 四槽位总览先绘制，随后叠加已接受关联框，确保关联结果始终处于最上层。
  const auto* horizon = FindCurrentHorizon(output);
  if (spatial && output.type && horizon) {
    constexpr ::foxglove::schemas::Color FRONT_COLOR{.r = 0.0, .g = 0.55, .b = 0.08, .a = 1.0};
    constexpr ::foxglove::schemas::Color BACK_COLOR{.r = 0.0, .g = 0.55, .b = 0.08, .a = 0.35};
    for (const auto& armor : horizon->armors) {
      const auto PROJECTED = ProjectArmor(armor.world_t_armor, *output.type, *spatial);
      if (!PROJECTED)
        continue;
      ::foxglove::schemas::PointsAnnotation polygon;
      polygon.timestamp = timestamp;
      polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
      polygon.outline_color = PROJECTED->front_facing ? FRONT_COLOR : BACK_COLOR;
      polygon.thickness = PROJECTED->front_facing ? 2.0 : 1.5;
      polygon.points.assign(PROJECTED->pixels.begin(), PROJECTED->pixels.end());
      annotations.points.push_back(std::move(polygon));
    }
  }

  for (const auto& association : diagnostics.associations) {
    if (!association.accepted || association.candidate_slot < 0)
      continue;
    ::foxglove::schemas::PointsAnnotation polygon;
    polygon.timestamp = timestamp;
    polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    polygon.outline_color = {.r = 0.1, .g = 1.0, .b = 0.2, .a = 1.0};
    polygon.thickness = 3.0;
    for (const auto& corner : association.predicted_corners)
      polygon.points.push_back({.x = corner.x, .y = corner.y});
    annotations.points.push_back(std::move(polygon));
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeEmptyAnnotations(
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  AddTimestampCarrier(annotations, timestamp);
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeImpactAnnotations(
    const modules::ArmorPredictionOutput& output, const frame::SpatialFrameView& spatial,
    const modules::ArmorImpactSnapshot& impact, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  AddTimestampCarrier(annotations, timestamp);
  constexpr ::foxglove::schemas::Color COLOR{.r = 0.75, .g = 0.25, .b = 1.0, .a = 1.0};

  const bool CAN_PROJECT = impact.source_sequence == output.sequence && impact.ballistic.valid &&
                           output.type && impact.selected_slot >= 0 && impact.selected_slot < 4 &&
                           std::isfinite(impact.ballistic.prediction_horizon_s) &&
                           impact.ballistic.prediction_horizon_s >= 0.0;
  if (!CAN_PROJECT)
    return annotations;

  const auto HORIZON =
      modules::ExtrapolatePrediction(output, impact.ballistic.prediction_horizon_s);
  const auto PROJECTED =
      ProjectArmor(HORIZON.armors[static_cast<std::size_t>(impact.selected_slot)].world_t_armor,
                   *output.type, spatial);
  if (!PROJECTED)
    return annotations;

  ::foxglove::schemas::PointsAnnotation polygon;
  polygon.timestamp = timestamp;
  polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
  polygon.outline_color = COLOR;
  polygon.thickness = 4.0;
  polygon.points.assign(PROJECTED->pixels.begin(), PROJECTED->pixels.end());
  annotations.points.push_back(std::move(polygon));
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeSelectedArmorAnnotations(
    const modules::ArmorPredictionOutput& output, const frame::SpatialFrameView& spatial,
    const modules::ArmorSelectionSnapshot& selection,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  AddTimestampCarrier(annotations, timestamp);
  if (output.state == modules::TrackerState::LOST || !output.type)
    return annotations;
  const auto* horizon = FindCurrentHorizon(output);
  if (!horizon)
    return annotations;

  const auto WORLD_T_CAMERA =
      geometry::Compose(spatial.world_t_gimbal, spatial.gimbal_t_camera_optical);
  const auto CAMERA_T_WORLD = geometry::Inverse(WORLD_T_CAMERA);
  const double WIDTH = *output.type == geometry::ArmorType::LARGE ? 0.225 : 0.135;
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
      const auto PROJECTED = ProjectPoint(POINT, spatial.calibration);
      if (!PROJECTED)
        return;
      pixels[index] = *PROJECTED;
      min_u = std::min(min_u, PROJECTED->x);
      min_v = std::min(min_v, PROJECTED->y);
      max_u = std::max(max_u, PROJECTED->x);
      max_v = std::max(max_v, PROJECTED->y);
    }
    if (max_u < 0.0 || max_v < 0.0 || min_u >= static_cast<double>(spatial.calibration.width) ||
        min_v >= static_cast<double>(spatial.calibration.height)) {
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
        .x = std::clamp(min_u, 0.0, static_cast<double>(spatial.calibration.width)),
        .y = std::clamp(min_v - 4.0, 0.0, static_cast<double>(spatial.calibration.height))};
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
