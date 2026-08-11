#include "tool/foxglove/pnp/pnp_message_encoder.hpp"

#include "geometry/rigid_transform.hpp"

#include <algorithm>
#include <array>
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
      {.key = "source", .value = modules::PnpInputSourceName(estimate.source)},
      {.key = "rmse_px", .value = fmt::format("{:.4f}", estimate.reprojection_rmse_px)}};
  ::foxglove::schemas::LinePrimitive outline;
  outline.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
  outline.thickness = 0.01;
  outline.color = {.g = 1.0, .a = 1.0};
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

std::string DetailedSummaryJson(const modules::PnpSourceSummary& value) {
  return fmt::format(
      "{{\"reprojection_rmse_px\":{},\"mean_corner_error_px\":{},"
      "\"position_error_m\":{},\"depth_error_m\":{},\"rotation_error_deg\":{},"
      "\"position_jitter_m\":{}}}",
      PercentilesJson(value.reprojection_rmse_px), PercentilesJson(value.mean_corner_error_px),
      PercentilesJson(value.position_error_m), PercentilesJson(value.depth_error_m),
      PercentilesJson(value.rotation_error_deg), PercentilesJson(value.position_jitter_m));
}

std::string GroupJson(const std::map<std::string, modules::PnpSourceSummary>& groups) {
  std::string value = "{";
  bool first = true;
  for (const auto& [name, summary] : groups) {
    if (!first)
      value += ',';
    first = false;
    value += fmt::format("\"{}\":{}", name, DetailedSummaryJson(summary));
  }
  value += '}';
  return value;
}

std::string FailureReasonsJson(const std::map<std::string, std::size_t>& reasons) {
  std::string value = "{";
  bool first = true;
  for (const auto& [name, count] : reasons) {
    if (!first)
      value += ',';
    first = false;
    value += fmt::format("\"{}\":{}", name, count);
  }
  value += '}';
  return value;
}

std::string SolveSummaryJson(const modules::PnpSolveSummary& value) {
  return fmt::format(
      "{{\"attempted\":{},\"succeeded\":{},\"candidate_switches\":{},"
      "\"rejection_reasons\":{}}}",
      value.attempted, value.succeeded, value.candidate_switches,
      FailureReasonsJson(value.rejection_reasons));
}

std::string PointJson(const cv::Point2f& point) {
  return fmt::format("[{:.9g},{:.9g}]", point.x, point.y);
}

std::string EndpointJson(const modules::EndpointRefinementDiagnostic& endpoint) {
  std::string candidates = "[";
  for (std::size_t index = 0; index < endpoint.scan_candidates.size(); ++index) {
    if (index != 0)
      candidates += ',';
    candidates += PointJson(endpoint.scan_candidates[index]);
  }
  candidates += ']';
  return fmt::format(
      "{{\"found\":{},\"applied\":{},\"original\":{},\"candidate\":{},\"final\":{},"
      "\"search_start\":{},\"search_end\":{},\"scan_candidates\":{}}}",
      endpoint.found ? "true" : "false", endpoint.applied ? "true" : "false",
      PointJson(endpoint.original), PointJson(endpoint.candidate), PointJson(endpoint.final),
      PointJson(endpoint.search_start), PointJson(endpoint.search_end), candidates);
}

std::string LightbarJson(const modules::LightbarRefinementDiagnostic& lightbar) {
  return fmt::format(
      "{{\"center\":{},\"axis\":{},\"top\":{},\"bottom\":{},\"length_px\":{:.9g},"
      "\"width_px\":{:.9g},\"mean_brightness\":{:.9g},\"axis_valid\":{},"
      "\"success\":{}}}",
      PointJson(lightbar.center), PointJson(lightbar.axis), PointJson(lightbar.top),
      PointJson(lightbar.bottom), lightbar.length_px, lightbar.width_px,
      lightbar.mean_brightness, lightbar.axis_valid ? "true" : "false",
      lightbar.success ? "true" : "false");
}

std::string RefinementJson(const modules::CornerRefinementResult& refinement) {
  return fmt::format(
      "{{\"mode\":\"jlu_pca_gradient\",\"success\":{},\"fallback\":{},\"status\":\"{}\","
      "\"failure_light_index\":{},\"elapsed_ms\":{:.9g},\"lightbars\":[{},{}],"
      "\"endpoints\":[{},{},{},{}],"
      "\"corner_displacements_px\":[{},{},{},{}]}}",
      refinement.success ? "true" : "false", refinement.fallback ? "true" : "false",
      modules::CornerRefinementStatusName(refinement.status), refinement.failure_light_index,
      refinement.elapsed_ms, LightbarJson(refinement.lightbars[0]),
      LightbarJson(refinement.lightbars[1]),
      EndpointJson(refinement.endpoints[0]), EndpointJson(refinement.endpoints[1]),
      EndpointJson(refinement.endpoints[2]), EndpointJson(refinement.endpoints[3]),
      PointJson(refinement.corner_displacements[0]), PointJson(refinement.corner_displacements[1]),
      PointJson(refinement.corner_displacements[2]), PointJson(refinement.corner_displacements[3]));
}

bool HasAppliedRefinement(const modules::ArmorPnpAttempt& attempt) {
  return attempt.refinement && attempt.refinement->success && !attempt.refinement->fallback;
}

}  // namespace

::foxglove::schemas::SceneUpdate EncodeEstimates(const modules::ArmorPnpFrameResult& result,
                                                 const hal::CameraFrame::FrameGeometry& geometry,
                                                 const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::SceneUpdate update;
  const auto world_t_camera =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate || attempt.source == modules::PnpInputSource::GROUND_TRUTH)
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
    ray.color = {.g = 1.0, .a = 0.8};
    ray.points = {{}, Point(estimate.camera_t_armor.translation)};
    ray_entity.lines.push_back(std::move(ray));
    update.entities.push_back(std::move(ray_entity));
  }
  return update;
}

::foxglove::schemas::ImageAnnotations EncodeCorners(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (attempt.source != modules::PnpInputSource::DETECTION || !attempt.refinement)
      continue;
    const auto& refinement = *attempt.refinement;
    ::foxglove::schemas::PointsAnnotation raw;
    raw.timestamp = timestamp;
    raw.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    raw.outline_color = {.g = 1.0, .b = 1.0, .a = 1.0};
    raw.thickness = 1.5;
    for (const auto& point : refinement.original_corners)
      raw.points.push_back({.x = point.x, .y = point.y});
    annotations.points.push_back(std::move(raw));
    if (HasAppliedRefinement(attempt)) {
      ::foxglove::schemas::PointsAnnotation refined;
      refined.timestamp = timestamp;
      refined.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
      refined.outline_color = {.r = 1.0, .b = 1.0, .a = 1.0};
      refined.thickness = 1.5;
      for (const auto& point : refinement.refined_corners)
        refined.points.push_back({.x = point.x, .y = point.y});
      annotations.points.push_back(std::move(refined));
    }
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeReprojection(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate || attempt.source == modules::PnpInputSource::GROUND_TRUTH)
      continue;
    ::foxglove::schemas::PointsAnnotation polygon;
    polygon.timestamp = timestamp;
    polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    polygon.outline_color = {.g = 1.0, .a = 1.0};
    polygon.thickness = 2.0;
    for (const auto& point : attempt.estimate->reprojected_corners) {
      polygon.points.push_back({.x = point.x, .y = point.y});
    }
    annotations.points.push_back(std::move(polygon));
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeErrorVectors(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate || attempt.source != modules::PnpInputSource::DETECTION ||
        !attempt.estimate->mean_corner_error_px || !attempt.refinement)
      continue;
    for (std::size_t index = 0; index < 4; ++index) {
      const cv::Point2f truth(
          attempt.estimate->image_corners[index].x - attempt.estimate->corner_delta_u_px[index],
          attempt.estimate->image_corners[index].y - attempt.estimate->corner_delta_v_px[index]);
      ::foxglove::schemas::PointsAnnotation raw_error;
      raw_error.timestamp = timestamp;
      raw_error.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
      raw_error.outline_color = {.r = 0.6, .g = 0.6, .b = 0.6, .a = 0.9};
      raw_error.thickness = 1.0;
      const auto& raw = attempt.refinement->original_corners[index];
      raw_error.points = {{.x = raw.x, .y = raw.y}, {.x = truth.x, .y = truth.y}};
      annotations.points.push_back(std::move(raw_error));
      if (HasAppliedRefinement(attempt)) {
        ::foxglove::schemas::PointsAnnotation final_error;
        final_error.timestamp = timestamp;
        final_error.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
        final_error.outline_color = {.r = 1.0, .b = 1.0, .a = 0.9};
        final_error.thickness = 1.0;
        const auto& final = attempt.refinement->refined_corners[index];
        final_error.points = {{.x = final.x, .y = final.y}, {.x = truth.x, .y = truth.y}};
        annotations.points.push_back(std::move(final_error));
      }
    }
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeCornerRefinerAxes(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (attempt.source != modules::PnpInputSource::DETECTION || !attempt.refinement)
      continue;
    const auto& refinement = *attempt.refinement;
    for (const auto& lightbar : refinement.lightbars) {
      if (!lightbar.axis_valid || cv::norm(lightbar.axis) < 0.5 || lightbar.length_px <= 0.0)
        continue;
      ::foxglove::schemas::PointsAnnotation axis;
      axis.timestamp = timestamp;
      axis.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
      axis.outline_color = {.r = 0.2, .g = 0.7, .b = 1.0, .a = 0.9};
      axis.thickness = 1.0;
      const auto extent = lightbar.axis * static_cast<float>(0.65 * lightbar.length_px);
      axis.points = {{.x = lightbar.center.x - extent.x, .y = lightbar.center.y - extent.y},
                     {.x = lightbar.center.x + extent.x, .y = lightbar.center.y + extent.y}};
      annotations.points.push_back(std::move(axis));
      ::foxglove::schemas::PointsAnnotation center;
      center.timestamp = timestamp;
      center.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
      center.outline_color = {.r = 0.2, .g = 0.7, .b = 1.0, .a = 1.0};
      center.thickness = 3.0;
      center.points = {{.x = lightbar.center.x, .y = lightbar.center.y}};
      annotations.points.push_back(std::move(center));
    }
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeCornerRefinerCandidates(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (attempt.source != modules::PnpInputSource::DETECTION || !attempt.refinement)
      continue;
    const auto& refinement = *attempt.refinement;
    for (const auto& endpoint : refinement.endpoints) {
      if (cv::norm(endpoint.search_end - endpoint.search_start) > 0.1) {
        ::foxglove::schemas::PointsAnnotation search;
        search.timestamp = timestamp;
        search.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
        search.outline_color = {.r = 1.0, .g = 0.65, .a = 0.8};
        search.thickness = 1.0;
        search.points = {{.x = endpoint.search_start.x, .y = endpoint.search_start.y},
                         {.x = endpoint.search_end.x, .y = endpoint.search_end.y}};
        annotations.points.push_back(std::move(search));
      }
      for (const auto& scan_candidate : endpoint.scan_candidates) {
        ::foxglove::schemas::PointsAnnotation candidate;
        candidate.timestamp = timestamp;
        candidate.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
        candidate.outline_color = {.r = 1.0, .g = 0.85, .a = 0.9};
        candidate.thickness = 2.0;
        candidate.points = {{.x = scan_candidate.x, .y = scan_candidate.y}};
        annotations.points.push_back(std::move(candidate));
      }
      if (endpoint.found) {
        ::foxglove::schemas::PointsAnnotation fused;
        fused.timestamp = timestamp;
        fused.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
        fused.outline_color = endpoint.applied ? ::foxglove::schemas::Color{.g = 1.0, .a = 1.0}
                                               : ::foxglove::schemas::Color{.r = 1.0, .a = 1.0};
        fused.thickness = 4.0;
        fused.points = {{.x = endpoint.candidate.x, .y = endpoint.candidate.y}};
        annotations.points.push_back(std::move(fused));
      }
    }
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
        "\"candidate_rmse_gap_px\":{},\"distance_m\":{},\"viewing_angle_deg\":{},"
        "\"truth_distance_m\":{},\"truth_viewing_angle_deg\":{},\"truth_armor_size\":{},"
        "\"image_width_px\":{},\"image_height_px\":{},"
        "\"corner_errors_px\":[{},{},{},{}],\"corner_delta_u_px\":[{},{},{},{}],"
        "\"corner_delta_v_px\":[{},{},{},{}],\"mean_corner_error_px\":{},"
        "\"position_error_m\":{},\"position_error_camera_m\":{},\"depth_error_m\":{},"
        "\"signed_depth_error_m\":{},\"rotation_error_deg\":{},\"position_jitter_m\":{},"
        "\"refinement\":{}}}",
        modules::PnpInputSourceName(attempt.source), attempt.input_index,
        modules::PnpStatusName(attempt.status),
        estimate && estimate->truth_id ? std::to_string(*estimate->truth_id) : "null",
        estimate ? std::to_string(estimate->candidate_index) : "null",
        estimate ? fmt::format("{:.9g}", estimate->reprojection_rmse_px) : "null",
        estimate ? OptionalNumber(estimate->candidate_rmse_gap_px) : "null",
        estimate ? fmt::format("{:.9g}", estimate->distance_m) : "null",
        estimate ? fmt::format("{:.9g}", estimate->viewing_angle_deg) : "null",
        estimate ? OptionalNumber(estimate->truth_distance_m) : "null",
        estimate ? OptionalNumber(estimate->truth_viewing_angle_deg) : "null",
        estimate && estimate->truth_type
            ? fmt::format("\"{}\"", *estimate->truth_type == hal::CameraFrame::ArmorType::LARGE
                                        ? "large"
                                        : "small")
            : "null",
        estimate ? fmt::format("{:.9g}", estimate->image_width_px) : "null",
        estimate ? fmt::format("{:.9g}", estimate->image_height_px) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_errors_px[0]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_errors_px[1]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_errors_px[2]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_errors_px[3]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_delta_u_px[0]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_delta_u_px[1]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_delta_u_px[2]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_delta_u_px[3]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_delta_v_px[0]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_delta_v_px[1]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_delta_v_px[2]) : "null",
        estimate ? fmt::format("{:.9g}", estimate->corner_delta_v_px[3]) : "null",
        estimate ? OptionalNumber(estimate->mean_corner_error_px) : "null",
        estimate ? OptionalNumber(estimate->position_error_m) : "null",
        estimate && estimate->position_error_camera_m
            ? fmt::format("[{:.9g},{:.9g},{:.9g}]", (*estimate->position_error_camera_m)[0],
                          (*estimate->position_error_camera_m)[1],
                          (*estimate->position_error_camera_m)[2])
            : "null",
        estimate ? OptionalNumber(estimate->depth_error_m) : "null",
        estimate ? OptionalNumber(estimate->signed_depth_error_m) : "null",
        estimate ? OptionalNumber(estimate->rotation_error_deg) : "null",
        estimate ? OptionalNumber(estimate->position_jitter_m) : "null",
        attempt.refinement ? RefinementJson(*attempt.refinement) : "null");
  }
  return fmt::format(
      "{{\"timestamp\":{{\"sec\":{},\"nsec\":{}}},\"sequence\":{},\"summary_sequence\":{},"
      "\"attempted\":{},\"successful\":{},\"summary\":{{\"ground_truth\":{},"
      "\"detection\":{}}},"
      "\"groups\":{{\"distance\":{},\"viewing_angle\":{},\"armor_size\":{}}},"
      "\"solve\":{},"
      "\"refinement\":{{\"attempted\":{},\"succeeded\":{},\"fallback\":{},"
      "\"failure_reasons\":{},\"elapsed_ms\":{},"
      "\"raw_mean_corner_error_px\":{},\"final_mean_corner_error_px\":{}}},"
      "\"attempts\":[{}]}}",
      timestamp.sec, timestamp.nsec, sequence, result.summary_sequence, result.attempts.size(),
      successes, DetailedSummaryJson(result.ground_truth_summary),
      DetailedSummaryJson(result.detection_summary), GroupJson(result.distance_groups),
      GroupJson(result.angle_groups), GroupJson(result.size_groups),
      SolveSummaryJson(result.solve_summary),
      result.refinement_summary.attempted, result.refinement_summary.succeeded,
      result.refinement_summary.fallback,
      FailureReasonsJson(result.refinement_summary.failure_reasons),
      PercentilesJson(result.refinement_summary.elapsed_ms),
      PercentilesJson(result.refinement_summary.raw_mean_corner_error_px),
      PercentilesJson(result.refinement_summary.final_mean_corner_error_px), attempts);
}

}  // namespace mv::tool::foxglove::pnp
