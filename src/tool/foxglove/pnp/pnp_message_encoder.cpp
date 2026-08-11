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
  outline.color = estimate.source == modules::PnpInputSource::DETECTION_RAW
                      ? ::foxglove::schemas::Color{.b = 1.0, .a = 1.0}
                      : ::foxglove::schemas::Color{.g = 1.0, .a = 1.0};
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
    candidates += endpoint.scan_candidate_present[index]
                      ? fmt::format("{{\"point\":{},\"accepted\":{}}}",
                                    PointJson(endpoint.scan_candidates[index]),
                                    endpoint.scan_candidate_valid[index] ? "true" : "false")
                      : "null";
  }
  candidates += ']';
  return fmt::format(
      "{{\"applied\":{},\"fallback\":{},\"candidate_valid\":{},\"reason\":\"{}\","
      "\"reverted_by\":\"{}\",\"gradient_strength\":{:.9g},"
      "\"secondary_gradient_strength\":{:.9g},\"secondary_peak_ratio\":{:.9g},"
      "\"inner_brightness\":{:.9g},"
      "\"bright_side_threshold\":{:.9g},\"valid_scan_lines\":{},"
      "\"profile_peak_spread_px\":{:.9g},"
      "\"original\":{},\"candidate\":{},\"final\":{},\"movement_px\":{:.9g},"
      "\"requested_movement_px\":{:.9g},"
      "\"search_start\":{},\"search_end\":{},\"scan_candidates\":{}}}",
      endpoint.applied ? "true" : "false", endpoint.fallback ? "true" : "false",
      endpoint.candidate_valid ? "true" : "false",
      modules::EndpointRefinementStatusName(endpoint.status),
      modules::EndpointRevertedByName(endpoint.reverted_by), endpoint.gradient_strength,
      endpoint.secondary_gradient_strength, endpoint.secondary_peak_ratio,
      endpoint.inner_brightness, endpoint.bright_side_threshold, endpoint.valid_scan_lines,
      endpoint.profile_peak_spread_px, PointJson(endpoint.original), PointJson(endpoint.candidate),
      PointJson(endpoint.final), endpoint.movement_px, endpoint.requested_movement_px,
      PointJson(endpoint.search_start), PointJson(endpoint.search_end), candidates);
}

std::string StripJson(const modules::RefinedLightStrip& strip) {
  return fmt::format(
      "{{\"center\":{},\"axis\":{},\"estimated_length_px\":{:.9g},"
      "\"estimated_width_px\":{:.9g},\"axis_ratio\":{:.9g},"
      "\"axis_deviation_deg\":{:.9g},\"center_offset_px\":{:.9g},"
      "\"background_brightness\":{:.9g},\"contrast\":{:.9g}}}",
      PointJson(strip.center), PointJson(strip.axis), strip.estimated_length_px,
      strip.estimated_width_px, strip.axis_ratio, strip.axis_deviation_deg, strip.center_offset_px,
      strip.background_brightness, strip.contrast);
}

std::string RefinementJson(const modules::CornerRefinementResult& refinement) {
  return fmt::format(
      "{{\"mode\":\"{}\",\"success\":{},\"fallback\":{},\"status\":\"{}\","
      "\"confidence\":{:.9g},\"elapsed_ms\":{:.9g},\"strips\":[{},{}],"
      "\"endpoints\":[{},{},{},{}],"
      "\"corner_displacements_px\":[{},{},{},{}]}}",
      modules::CornerRefinementModeName(refinement.mode), refinement.success ? "true" : "false",
      refinement.fallback ? "true" : "false",
      modules::CornerRefinementStatusName(refinement.status), refinement.confidence,
      refinement.elapsed_ms, StripJson(refinement.strips[0]), StripJson(refinement.strips[1]),
      EndpointJson(refinement.endpoints[0]), EndpointJson(refinement.endpoints[1]),
      EndpointJson(refinement.endpoints[2]), EndpointJson(refinement.endpoints[3]),
      PointJson(refinement.corner_displacements[0]), PointJson(refinement.corner_displacements[1]),
      PointJson(refinement.corner_displacements[2]), PointJson(refinement.corner_displacements[3]));
}

bool HasAppliedRefinement(const modules::ArmorPnpAttempt& attempt) {
  return attempt.refinement &&
         std::any_of(attempt.refinement->endpoints.begin(), attempt.refinement->endpoints.end(),
                     [](const modules::EndpointRefinementDiagnostic& endpoint) {
                       return endpoint.applied;
                     });
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
    ray.color = estimate.source == modules::PnpInputSource::DETECTION_RAW
                    ? ::foxglove::schemas::Color{.b = 1.0, .a = 0.8}
                    : ::foxglove::schemas::Color{.g = 1.0, .a = 0.8};
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
    if (!attempt.estimate || attempt.source == modules::PnpInputSource::GROUND_TRUTH)
      continue;
    const bool raw = attempt.source == modules::PnpInputSource::DETECTION_RAW;
    if (!raw && !HasAppliedRefinement(attempt))
      continue;
    ::foxglove::schemas::PointsAnnotation input;
    input.timestamp = timestamp;
    input.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    input.outline_color = raw ? ::foxglove::schemas::Color{.g = 1.0, .b = 1.0, .a = 1.0}
                              : ::foxglove::schemas::Color{.r = 1.0, .b = 1.0, .a = 1.0};
    input.thickness = 1.5;
    for (const auto& point : attempt.estimate->image_corners)
      input.points.push_back({.x = point.x, .y = point.y});
    annotations.points.push_back(std::move(input));
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeReprojection(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate || attempt.source == modules::PnpInputSource::GROUND_TRUTH)
      continue;
    const bool raw = attempt.source == modules::PnpInputSource::DETECTION_RAW;
    if (!raw && !HasAppliedRefinement(attempt))
      continue;
    ::foxglove::schemas::PointsAnnotation polygon;
    polygon.timestamp = timestamp;
    polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    polygon.outline_color = raw ? ::foxglove::schemas::Color{.b = 1.0, .a = 1.0}
                                : ::foxglove::schemas::Color{.g = 1.0, .a = 1.0};
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
    if (!attempt.estimate || attempt.source != modules::PnpInputSource::DETECTION_RAW ||
        !attempt.estimate->mean_corner_error_px)
      continue;
    for (std::size_t index = 0; index < 4; ++index) {
      ::foxglove::schemas::PointsAnnotation error;
      error.timestamp = timestamp;
      error.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
      error.outline_color = {.r = 0.6, .g = 0.6, .b = 0.6, .a = 0.9};
      error.thickness = 1.0;
      const auto& point = attempt.estimate->image_corners[index];
      error.points = {{.x = point.x, .y = point.y},
                      {.x = point.x - attempt.estimate->corner_delta_u_px[index],
                       .y = point.y - attempt.estimate->corner_delta_v_px[index]}};
      annotations.points.push_back(std::move(error));
    }
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeCornerRefinerAxes(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (attempt.source != modules::PnpInputSource::DETECTION_REFINED || !attempt.refinement)
      continue;
    const auto& refinement = *attempt.refinement;
    for (const auto& strip : refinement.strips) {
      if (cv::norm(strip.axis) < 0.5 || strip.estimated_length_px <= 0.0)
        continue;
      ::foxglove::schemas::PointsAnnotation axis;
      axis.timestamp = timestamp;
      axis.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
      axis.outline_color = {.r = 0.2, .g = 0.7, .b = 1.0, .a = 0.9};
      axis.thickness = 1.0;
      const auto extent = strip.axis * static_cast<float>(0.65 * strip.estimated_length_px);
      axis.points = {{.x = strip.center.x - extent.x, .y = strip.center.y - extent.y},
                     {.x = strip.center.x + extent.x, .y = strip.center.y + extent.y}};
      annotations.points.push_back(std::move(axis));
    }
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeCornerRefinerCandidates(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (attempt.source != modules::PnpInputSource::DETECTION_REFINED || !attempt.refinement)
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
      for (std::size_t scan = 0; scan < endpoint.scan_candidates.size(); ++scan) {
        if (!endpoint.scan_candidate_present[scan])
          continue;
        ::foxglove::schemas::PointsAnnotation candidate;
        candidate.timestamp = timestamp;
        candidate.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
        candidate.outline_color = endpoint.scan_candidate_valid[scan]
                                      ? ::foxglove::schemas::Color{.g = 1.0, .a = 0.9}
                                      : ::foxglove::schemas::Color{.r = 1.0, .a = 0.9};
        candidate.thickness = 2.0;
        candidate.points = {
            {.x = endpoint.scan_candidates[scan].x, .y = endpoint.scan_candidates[scan].y}};
        annotations.points.push_back(std::move(candidate));
      }
      if (endpoint.requested_movement_px > 0.0) {
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
      "\"detection_raw\":{},\"refined_success_only\":{},"
      "\"refined_with_fallback\":{}}},"
      "\"groups\":{{\"distance\":{{\"detection_raw\":{},\"refined_success_only\":{},"
      "\"refined_with_fallback\":{}}},"
      "\"viewing_angle\":{{\"detection_raw\":{},\"refined_success_only\":{},"
      "\"refined_with_fallback\":{}}},"
      "\"armor_size\":{{\"detection_raw\":{},\"refined_success_only\":{},"
      "\"refined_with_fallback\":{}}}}},"
      "\"solve\":{{\"detection_raw\":{},\"detection_refined\":{}}},"
      "\"refinement\":{{\"attempted\":{},\"succeeded\":{},\"fallback\":{},"
      "\"fully_refined\":{},\"full_fallback\":{},"
      "\"failure_reasons\":{},\"elapsed_ms\":{}}},\"attempts\":[{}]}}",
      timestamp.sec, timestamp.nsec, sequence, result.summary_sequence, result.attempts.size(),
      successes, DetailedSummaryJson(result.ground_truth_summary),
      DetailedSummaryJson(result.detection_raw_summary),
      DetailedSummaryJson(result.detection_refined_success_summary),
      DetailedSummaryJson(result.detection_refined_with_fallback_summary),
      GroupJson(result.raw_distance_groups), GroupJson(result.refined_success_distance_groups),
      GroupJson(result.refined_with_fallback_distance_groups), GroupJson(result.raw_angle_groups),
      GroupJson(result.refined_success_angle_groups),
      GroupJson(result.refined_with_fallback_angle_groups), GroupJson(result.raw_size_groups),
      GroupJson(result.refined_success_size_groups),
      GroupJson(result.refined_with_fallback_size_groups),
      SolveSummaryJson(result.raw_solve_summary), SolveSummaryJson(result.refined_solve_summary),
      result.refinement_summary.attempted, result.refinement_summary.succeeded,
      result.refinement_summary.fallback, result.refinement_summary.fully_refined,
      result.refinement_summary.full_fallback,
      FailureReasonsJson(result.refinement_summary.failure_reasons),
      PercentilesJson(result.refinement_summary.elapsed_ms), attempts);
}

}  // namespace mv::tool::foxglove::pnp
