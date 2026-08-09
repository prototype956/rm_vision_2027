#include "tool/foxglove/pnp/pnp_message_encoder.hpp"

#include "geometry/rigid_transform.hpp"

#include <string>
#include <utility>

#include <fmt/format.h>

namespace mv::tool::foxglove::pnp {
namespace {

::foxglove::schemas::Point3 Point(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

void AddEstimateEntity(::foxglove::schemas::SceneUpdate& update,
                       const modules::ArmorPoseEstimate& estimate,
                       const geometry::RigidTransform& frame_t_armor, std::string frame_id,
                       std::string suffix, const ::foxglove::schemas::Timestamp& timestamp) {
  const double width = estimate.width_m;
  const double height = estimate.height_m;
  const std::array<geometry::Vector3, 4> local{geometry::Vector3(-width * 0.5, height * 0.5, 0),
                                               geometry::Vector3(width * 0.5, height * 0.5, 0),
                                               geometry::Vector3(width * 0.5, -height * 0.5, 0),
                                               geometry::Vector3(-width * 0.5, -height * 0.5, 0)};
  ::foxglove::schemas::SceneEntity entity;
  entity.timestamp = timestamp;
  entity.frame_id = std::move(frame_id);
  entity.id =
      fmt::format("pnp_{}_{}_{}", static_cast<int>(estimate.source), estimate.input_index, suffix);
  entity.lifetime = {.sec = 0, .nsec = 200'000'000};
  entity.metadata = {
      {.key = "source",
       .value = estimate.source == modules::PnpInputSource::GROUND_TRUTH ? "ground_truth_corners"
                                                                         : "detection_corners"},
      {.key = "rmse_px", .value = fmt::format("{:.4f}", estimate.reprojection_rmse_px)}};
  ::foxglove::schemas::LinePrimitive outline;
  outline.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
  outline.thickness = 0.01;
  outline.color = estimate.source == modules::PnpInputSource::GROUND_TRUTH
                      ? ::foxglove::schemas::Color{.g = 1.0, .a = 1.0}
                      : ::foxglove::schemas::Color{.g = 0.8, .b = 1.0, .a = 1.0};
  for (const auto& corner : local) {
    outline.points.push_back(Point(geometry::TransformPoint(frame_t_armor, corner)));
  }
  entity.lines.push_back(std::move(outline));
  update.entities.push_back(std::move(entity));
}

std::string OptionalNumber(const std::optional<double>& value) {
  return value ? fmt::format("{:.9g}", *value) : "null";
}

std::string PercentilesJson(const modules::PnpPercentiles& value) {
  return fmt::format("{{\"samples\":{},\"p50\":{:.9g},\"p95\":{:.9g}}}", value.samples, value.p50,
                     value.p95);
}

std::string SummaryJson(const modules::PnpSourceSummary& value) {
  return fmt::format(
      "{{\"reprojection_rmse_px\":{},\"mean_corner_error_px\":{},"
      "\"position_error_m\":{},\"depth_error_m\":{},\"rotation_error_deg\":{}}}",
      PercentilesJson(value.reprojection_rmse_px), PercentilesJson(value.mean_corner_error_px),
      PercentilesJson(value.position_error_m), PercentilesJson(value.depth_error_m),
      PercentilesJson(value.rotation_error_deg));
}

}  // namespace

::foxglove::schemas::SceneUpdate EncodeEstimates(const modules::ArmorPnpFrameResult& result,
                                                 const hal::CameraFrame::FrameGeometry& geometry,
                                                 const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::SceneUpdate update;
  const auto world_t_camera =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate)
      continue;
    const auto& estimate = *attempt.estimate;
    AddEstimateEntity(update, estimate, estimate.camera_t_armor, "camera_optical", "camera",
                      timestamp);
    AddEstimateEntity(update, estimate, geometry::Compose(world_t_camera, estimate.camera_t_armor),
                      "world", "world", timestamp);
    ::foxglove::schemas::SceneEntity ray_entity;
    ray_entity.timestamp = timestamp;
    ray_entity.frame_id = "camera_optical";
    ray_entity.id =
        fmt::format("pnp_ray_{}_{}", static_cast<int>(estimate.source), estimate.input_index);
    ray_entity.lifetime = {.sec = 0, .nsec = 200'000'000};
    ::foxglove::schemas::LinePrimitive ray;
    ray.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    ray.thickness = 0.006;
    ray.color = {.r = 0.8, .g = 0.3, .b = 1.0, .a = 0.8};
    ray.points = {{}, Point(estimate.camera_t_armor.translation)};
    ray_entity.lines.push_back(std::move(ray));
    update.entities.push_back(std::move(ray_entity));
  }
  return update;
}

::foxglove::schemas::ImageAnnotations EncodeAnnotations(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate || attempt.source != modules::PnpInputSource::DETECTION)
      continue;
    ::foxglove::schemas::PointsAnnotation polygon;
    polygon.timestamp = timestamp;
    polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    polygon.outline_color = {.r = 0.2, .g = 1.0, .b = 0.2, .a = 1.0};
    polygon.thickness = 2.0;
    for (const auto& point : attempt.estimate->reprojected_corners) {
      polygon.points.push_back({.x = point.x, .y = point.y});
    }
    annotations.points.push_back(std::move(polygon));
  }
  return annotations;
}

std::string EncodeStats(const modules::ArmorPnpFrameResult& result, std::uint64_t sequence,
                        const ::foxglove::schemas::Timestamp& timestamp) {
  std::string attempts;
  std::size_t successes = 0;
  for (const auto& attempt : result.attempts) {
    if (!attempts.empty())
      attempts += ',';
    if (attempt.estimate)
      ++successes;
    const auto& estimate = attempt.estimate;
    attempts += fmt::format(
        "{{\"source\":\"{}\",\"input_index\":{},\"status\":\"{}\","
        "\"truth_id\":{},\"candidate_index\":{},\"reprojection_rmse_px\":{},"
        "\"distance_m\":{},\"viewing_angle_deg\":{},\"corner_errors_px\":[{},{},{},{}],"
        "\"mean_corner_error_px\":{},\"position_error_m\":{},\"depth_error_m\":{},"
        "\"rotation_error_deg\":{}}}",
        attempt.source == modules::PnpInputSource::GROUND_TRUTH ? "ground_truth" : "detection",
        attempt.input_index, modules::PnpStatusName(attempt.status),
        estimate && estimate->truth_id ? std::to_string(*estimate->truth_id) : "null",
        estimate ? std::to_string(estimate->candidate_index) : "null",
        estimate ? fmt::format("{:.9g}", estimate->reprojection_rmse_px) : "null",
        estimate ? fmt::format("{:.9g}", estimate->distance_m) : "null",
        estimate ? fmt::format("{:.9g}", estimate->viewing_angle_deg) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_errors_px[0]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_errors_px[1]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_errors_px[2]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_errors_px[3]) : "null",
        estimate ? OptionalNumber(estimate->mean_corner_error_px) : "null",
        estimate ? OptionalNumber(estimate->position_error_m) : "null",
        estimate ? OptionalNumber(estimate->depth_error_m) : "null",
        estimate ? OptionalNumber(estimate->rotation_error_deg) : "null");
  }
  return fmt::format(
      "{{\"timestamp\":{{\"sec\":{},\"nsec\":{}}},\"sequence\":{},"
      "\"attempted\":{},\"successful\":{},\"summary\":{{\"ground_truth\":{},"
      "\"detection\":{}}},\"attempts\":[{}]}}",
      timestamp.sec, timestamp.nsec, sequence, result.attempts.size(), successes,
      SummaryJson(result.ground_truth_summary), SummaryJson(result.detection_summary), attempts);
}

}  // namespace mv::tool::foxglove::pnp
