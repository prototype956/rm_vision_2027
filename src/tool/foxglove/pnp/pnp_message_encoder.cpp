#include "tool/foxglove/pnp/pnp_message_encoder.hpp"

#include "geometry/rigid_transform.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>

namespace mv::tool::foxglove::pnp {
namespace {

::foxglove::schemas::Point3 Point(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

void AddEstimateEntity(::foxglove::schemas::SceneUpdate& update,
                       const simulation_evaluation::EvaluatedArmorPose& estimate,
                       const geometry::RigidTransform& frame_t_armor, std::string_view frame_id,
                       const char* suffix, const ::foxglove::schemas::Timestamp& timestamp) {
  const double WIDTH = estimate.width_m;
  const double HEIGHT = estimate.height_m;
  const std::array<geometry::Vector3, 4> LOCAL{geometry::Vector3(-WIDTH * 0.5, HEIGHT * 0.5, 0),
                                               geometry::Vector3(WIDTH * 0.5, HEIGHT * 0.5, 0),
                                               geometry::Vector3(WIDTH * 0.5, -HEIGHT * 0.5, 0),
                                               geometry::Vector3(-WIDTH * 0.5, -HEIGHT * 0.5, 0)};
  ::foxglove::schemas::SceneEntity entity;
  entity.timestamp = timestamp;
  entity.frame_id = std::string(frame_id);
  // source、input_index 和坐标系后缀组成稳定 ID，使 Foxglove 原位更新而不留下拖影。
  entity.id =
      fmt::format("pnp_{}_{}_{}", static_cast<int>(estimate.source), estimate.input_index, suffix);
  entity.lifetime = {.sec = 0, .nsec = 200'000'000};
  entity.metadata = {
      {.key = "source", .value = simulation_evaluation::PnpEvaluationSourceName(estimate.source)},
      {.key = "rmse_px", .value = fmt::format("{:.4f}", estimate.reprojection_rmse_px)}};
  ::foxglove::schemas::LinePrimitive outline;
  outline.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
  outline.thickness = 0.01;
  outline.color = {.g = 1.0, .a = 1.0};
  for (const auto& corner : LOCAL) {
    outline.points.push_back(Point(geometry::TransformPoint(frame_t_armor, corner)));
  }
  entity.lines.push_back(std::move(outline));
  update.entities.push_back(std::move(entity));
}

std::string OptionalNumber(const std::optional<double>& value) {
  return value ? fmt::format("{:.9g}", *value) : "null";
}

std::string PercentilesJson(const simulation_evaluation::EvaluationPercentiles& value) {
  return fmt::format("{{\"samples\":{},\"p50\":{:.9g},\"p95\":{:.9g}}}", value.samples, value.p50,
                     value.p95);
}

std::string DetailedSummaryJson(const simulation_evaluation::PnpSourceSummary& value) {
  return fmt::format(
      "{{\"reprojection_rmse_px\":{},\"mean_corner_error_px\":{},"
      "\"position_error_m\":{},\"depth_error_m\":{},\"rotation_error_deg\":{},"
      "\"position_jitter_m\":{}}}",
      PercentilesJson(value.reprojection_rmse_px), PercentilesJson(value.mean_corner_error_px),
      PercentilesJson(value.position_error_m), PercentilesJson(value.depth_error_m),
      PercentilesJson(value.rotation_error_deg), PercentilesJson(value.position_jitter_m));
}

std::string GroupJson(
    const std::map<std::string, simulation_evaluation::PnpSourceSummary>& groups) {
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

std::string SolveSummaryJson(const simulation_evaluation::PnpSolveSummary& value) {
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
      PointJson(lightbar.bottom), lightbar.length_px, lightbar.width_px, lightbar.mean_brightness,
      lightbar.axis_valid ? "true" : "false", lightbar.success ? "true" : "false");
}

std::string RefinementJson(const modules::CornerRefinementDiagnostics& refinement) {
  return fmt::format(
      "{{\"mode\":\"jlu_pca_gradient\",\"success\":{},\"fallback\":{},\"status\":\"{}\","
      "\"failure_light_index\":{},\"elapsed_ms\":{:.9g},\"lightbars\":[{},{}],"
      "\"endpoints\":[{},{},{},{}],"
      "\"corner_displacements_px\":[{},{},{},{}]}}",
      refinement.success ? "true" : "false", refinement.fallback ? "true" : "false",
      modules::CornerRefinementStatusName(refinement.status), refinement.failure_light_index,
      refinement.elapsed_ms, LightbarJson(refinement.lightbars[0]),
      LightbarJson(refinement.lightbars[1]), EndpointJson(refinement.endpoints[0]),
      EndpointJson(refinement.endpoints[1]), EndpointJson(refinement.endpoints[2]),
      EndpointJson(refinement.endpoints[3]), PointJson(refinement.corner_displacements[0]),
      PointJson(refinement.corner_displacements[1]), PointJson(refinement.corner_displacements[2]),
      PointJson(refinement.corner_displacements[3]));
}

bool HasAppliedRefinement(const simulation_evaluation::PnpEvaluationAttempt& attempt) {
  return attempt.refinement && attempt.refinement->success && !attempt.refinement->fallback;
}

void AddTimestampCarrier(::foxglove::schemas::ImageAnnotations& annotations,
                         const ::foxglove::schemas::Timestamp& timestamp) {
  // ImageAnnotations 没有顶层时间戳，用不可见点集承载帧时间并清除上一帧角点。
  ::foxglove::schemas::PointsAnnotation carrier;
  carrier.timestamp = timestamp;
  carrier.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
  carrier.thickness = 0.0;
  annotations.points.push_back(std::move(carrier));
}

void AddCornerOutline(::foxglove::schemas::ImageAnnotations& annotations,
                      const std::array<cv::Point2f, 4>& corners,
                      const ::foxglove::schemas::Color& color, double thickness,
                      std::string_view label, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::PointsAnnotation outline;
  outline.timestamp = timestamp;
  outline.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
  outline.outline_color = color;
  outline.thickness = thickness;
  float min_u = corners.front().x;
  float min_v = corners.front().y;
  for (const auto& point : corners) {
    outline.points.push_back({.x = point.x, .y = point.y});
    min_u = std::min(min_u, point.x);
    min_v = std::min(min_v, point.y);
  }
  annotations.points.push_back(std::move(outline));
  if (label.empty())
    return;

  ::foxglove::schemas::TextAnnotation text;
  text.timestamp = timestamp;
  text.position = {.x = std::max(0.0, static_cast<double>(min_u)),
                   .y = std::max(0.0, static_cast<double>(min_v) - 3.0)};
  text.text = std::string(label);
  text.font_size = 12.0;
  text.text_color = color;
  text.background_color = {.a = 0.65};
  annotations.texts.push_back(std::move(text));
}

}  // namespace

::foxglove::schemas::SceneUpdate EncodeEstimates(
    const simulation_evaluation::PnpEvaluationResult& result,
    const frame::FrameKinematics& kinematics, const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::SceneUpdate update;
  const auto WORLD_T_CAMERA =
      geometry::Compose(kinematics.world_t_gimbal, kinematics.gimbal_t_camera_optical);
  // 真值链只用于数值基准，不叠加到绿色正式估计图层，避免与仿真真值频道重复。
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate ||
        attempt.source == simulation_evaluation::PnpEvaluationSource::GROUND_TRUTH)
      continue;
    const auto& estimate = *attempt.estimate;
    AddEstimateEntity(update, estimate, estimate.camera_t_armor, "camera_optical", "camera",
                      timestamp);
    AddEstimateEntity(update, estimate, geometry::Compose(WORLD_T_CAMERA, estimate.camera_t_armor),
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

::foxglove::schemas::ImageAnnotations EncodeRawCorners(
    const simulation_evaluation::PnpEvaluationResult& result,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  AddTimestampCarrier(annotations, timestamp);
  for (const auto& attempt : result.attempts) {
    if (attempt.source != simulation_evaluation::PnpEvaluationSource::DETECTION ||
        !attempt.refinement)
      continue;
    AddCornerOutline(annotations, attempt.refinement->original_corners,
                     {.g = 1.0, .b = 1.0, .a = 1.0}, 1.5, {}, timestamp);
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeFinalCorners(
    const simulation_evaluation::PnpEvaluationResult& result,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  AddTimestampCarrier(annotations, timestamp);
  for (const auto& attempt : result.attempts) {
    if (attempt.source != simulation_evaluation::PnpEvaluationSource::DETECTION ||
        !attempt.refinement)
      continue;
    const auto& refinement = *attempt.refinement;
    if (HasAppliedRefinement(attempt)) {
      AddCornerOutline(annotations, refinement.refined_corners, {.r = 1.0, .b = 1.0, .a = 1.0}, 2.5,
                       "REFINED", timestamp);
      continue;
    }
    const auto LABEL =
        fmt::format("FALLBACK:{}", modules::CornerRefinementStatusName(refinement.status));
    AddCornerOutline(annotations, refinement.refined_corners, {.r = 1.0, .g = 0.85, .a = 1.0}, 2.5,
                     LABEL, timestamp);
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeReprojection(
    const simulation_evaluation::PnpEvaluationResult& result,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate ||
        attempt.source == simulation_evaluation::PnpEvaluationSource::GROUND_TRUTH)
      continue;
    // 重投影与输入角点分属不同频道，便于独立开关并直接观察模型残差。
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
    const simulation_evaluation::PnpEvaluationResult& result,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (!attempt.estimate ||
        attempt.source != simulation_evaluation::PnpEvaluationSource::DETECTION ||
        !attempt.estimate->mean_corner_error_px || !attempt.refinement)
      continue;
    for (std::size_t index = 0; index < 4; ++index) {
      // truth = formal input - 已记录的有符号偏差；无需在消息结果中重复保存真值角点。
      const cv::Point2f TRUTH(static_cast<float>(attempt.estimate->image_corners[index].x -
                                                 attempt.estimate->corner_delta_u_px[index]),
                              static_cast<float>(attempt.estimate->image_corners[index].y -
                                                 attempt.estimate->corner_delta_v_px[index]));
      ::foxglove::schemas::PointsAnnotation raw_error;
      raw_error.timestamp = timestamp;
      raw_error.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
      raw_error.outline_color = {.r = 0.6, .g = 0.6, .b = 0.6, .a = 0.9};
      raw_error.thickness = 1.0;
      const auto& raw = attempt.refinement->original_corners[index];
      raw_error.points = {{.x = raw.x, .y = raw.y}, {.x = TRUTH.x, .y = TRUTH.y}};
      annotations.points.push_back(std::move(raw_error));
      if (HasAppliedRefinement(attempt)) {
        ::foxglove::schemas::PointsAnnotation final_error;
        final_error.timestamp = timestamp;
        final_error.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
        final_error.outline_color = {.r = 1.0, .b = 1.0, .a = 0.9};
        final_error.thickness = 1.0;
        const auto& final = attempt.refinement->refined_corners[index];
        final_error.points = {{.x = final.x, .y = final.y}, {.x = TRUTH.x, .y = TRUTH.y}};
        annotations.points.push_back(std::move(final_error));
      }
    }
  }
  return annotations;
}

::foxglove::schemas::ImageAnnotations EncodeCornerRefinerAxes(
    const simulation_evaluation::PnpEvaluationResult& result,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (attempt.source != simulation_evaluation::PnpEvaluationSource::DETECTION ||
        !attempt.refinement)
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
      const auto EXTENT = lightbar.axis * static_cast<float>(0.65 * lightbar.length_px);
      axis.points = {{.x = lightbar.center.x - EXTENT.x, .y = lightbar.center.y - EXTENT.y},
                     {.x = lightbar.center.x + EXTENT.x, .y = lightbar.center.y + EXTENT.y}};
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
    const simulation_evaluation::PnpEvaluationResult& result,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  for (const auto& attempt : result.attempts) {
    if (attempt.source != simulation_evaluation::PnpEvaluationSource::DETECTION ||
        !attempt.refinement)
      continue;
    const auto& refinement = *attempt.refinement;
    for (const auto& endpoint : refinement.endpoints) {
      if (cv::norm(endpoint.search_end - endpoint.search_start) > 0.1) {
        // 橙色线段表示沿 PCA 主轴实际扫描的区间，黄色点表示各扫描线局部候选。
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
        // 融合候选即使因整块装甲原子回退未采用也保留，并用红色明确区分。
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

std::string EncodeStats(const simulation_evaluation::PnpEvaluationResult& result,
                        std::uint64_t sequence, const ::foxglove::schemas::Timestamp& timestamp) {
  // attempts 是当前帧明细；summary、groups、solve 和 refinement 是最近原子统计快照。
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
        simulation_evaluation::PnpEvaluationSourceName(attempt.source), attempt.input_index,
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
            ? fmt::format("\"{}\"",
                          *estimate->truth_type == geometry::ArmorType::LARGE ? "large" : "small")
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
      SolveSummaryJson(result.solve_summary), result.refinement_summary.attempted,
      result.refinement_summary.succeeded, result.refinement_summary.fallback,
      FailureReasonsJson(result.refinement_summary.failure_reasons),
      PercentilesJson(result.refinement_summary.elapsed_ms),
      PercentilesJson(result.refinement_summary.raw_mean_corner_error_px),
      PercentilesJson(result.refinement_summary.final_mean_corner_error_px), attempts);
}

}  // namespace mv::tool::foxglove::pnp
