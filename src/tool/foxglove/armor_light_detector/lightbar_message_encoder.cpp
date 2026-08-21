#include "tool/foxglove/armor_light_detector/lightbar_message_encoder.hpp"

#include <string_view>
#include <utility>

#include <fmt/format.h>

namespace mv::tool::foxglove::armor_light_detector {
namespace {

using Color = ::foxglove::schemas::Color;

constexpr Color K_RAW_COLOR{.r = 0.75, .g = 0.75, .b = 0.75, .a = 0.8};
constexpr Color K_PREDICTED_COLOR{.r = 0.0, .g = 0.9, .b = 1.0, .a = 0.9};
constexpr Color K_ACCEPTED_COLOR{.r = 0.1, .g = 1.0, .b = 0.1, .a = 1.0};
constexpr Color K_DUPLICATE_COLOR{.r = 1.0, .g = 0.8, .b = 0.0, .a = 1.0};
constexpr Color K_REJECTED_COLOR{.r = 1.0, .g = 0.15, .b = 0.15, .a = 1.0};

void AddLine(::foxglove::schemas::ImageAnnotations& annotations, const cv::Point2f& start,
             const cv::Point2f& end, const Color& color, double thickness,
             const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::PointsAnnotation line;
  line.timestamp = timestamp;
  line.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_STRIP;
  line.outline_color = color;
  line.thickness = thickness;
  line.points = {{.x = start.x, .y = start.y}, {.x = end.x, .y = end.y}};
  annotations.points.push_back(std::move(line));
}

std::string EscapeJson(std::string_view input) {
  std::string result;
  result.reserve(input.size());
  for (const char CHARACTER : input) {
    if (CHARACTER == '\\' || CHARACTER == '"') {
      result.push_back('\\');
    }
    result.push_back(CHARACTER);
  }
  return result;
}

}  // namespace

::foxglove::schemas::ImageAnnotations EncodeAnnotations(
    const modules::LightbarDetectionResult& detection,
    const modules::ArmorPredictionResult& prediction,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;
  ::foxglove::schemas::PointsAnnotation timestamp_carrier;
  timestamp_carrier.timestamp = timestamp;
  timestamp_carrier.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
  timestamp_carrier.thickness = 0.0;
  annotations.points.push_back(std::move(timestamp_carrier));

  std::vector<const modules::LightbarAssociation*> diagnostics(detection.detections.size(),
                                                               nullptr);
  for (std::size_t index = 0;
       index < prediction.lightbar_associations.size() && index < diagnostics.size(); ++index) {
    const auto& association = prediction.lightbar_associations[index];
    diagnostics[index] = &association;
    if (association.candidate_slot >= 0) {
      AddLine(annotations, association.predicted_top, association.predicted_bottom,
              K_PREDICTED_COLOR, 1.5, timestamp);
    }
  }

  for (std::size_t index = 0; index < detection.detections.size(); ++index) {
    const auto& lightbar = detection.detections[index];
    const auto* diagnostic = diagnostics[index];
    Color color = K_RAW_COLOR;
    std::string status = "raw";
    if (diagnostic != nullptr) {
      if (diagnostic->accepted) {
        color = K_ACCEPTED_COLOR;
        status = "accepted";
      } else if (diagnostic->duplicate_full_armor) {
        color = K_DUPLICATE_COLOR;
        status = "duplicate";
      } else {
        color = K_REJECTED_COLOR;
        status = diagnostic->rejection_reason.empty() ? "rejected" : diagnostic->rejection_reason;
      }
    }
    AddLine(annotations, lightbar.top, lightbar.bottom, color, 2.5, timestamp);

    ::foxglove::schemas::TextAnnotation text;
    text.timestamp = timestamp;
    text.position = {.x = lightbar.center.x, .y = lightbar.center.y};
    text.text = diagnostic != nullptr && diagnostic->slot >= 0
                    ? fmt::format("L{} {} {} {}", lightbar.input_index, diagnostic->slot,
                                  diagnostic->left ? "left" : "right", status)
                    : fmt::format("L{} {}", lightbar.input_index, status);
    text.font_size = 11.0;
    text.text_color = color;
    text.background_color = {.r = 0.0, .g = 0.0, .b = 0.0, .a = 0.65};
    annotations.texts.push_back(std::move(text));
  }
  return annotations;
}

std::string EncodeStats(const modules::LightbarDetectionResult& detection,
                        const modules::ArmorPredictionResult& prediction, std::uint64_t sequence,
                        const ::foxglove::schemas::Timestamp& timestamp) {
  const auto& stats = detection.stats;
  return fmt::format(
      "{{\"timestamp\":{{\"sec\":{},\"nsec\":{}}},\"sequence\":{},"
      "\"enabled\":{},\"valid_input\":{},\"binary_threshold\":{},"
      "\"threshold_source\":\"{}\",\"reference_lightbars\":{},\"contours\":{},"
      "\"geometry_candidates\":{},\"color_candidates\":{},\"kept_candidates\":{},"
      "\"elapsed_ms\":{:.3f},\"detected_count\":{},\"deduplicated_count\":{},"
      "\"matched_count\":{},\"accepted_count\":{},\"rejected_count\":{},"
      "\"light_only_pair_count\":{},\"light_only_update\":{},"
      "\"light_only_update_blocked\":{},\"light_only_rejection_reason\":\"{}\","
      "\"light_fusion_used\":{},\"armor_fallback_used\":{},"
      "\"rejection_reason\":\"{}\"}}",
      timestamp.sec, timestamp.nsec, sequence, stats.enabled ? "true" : "false",
      stats.valid_input ? "true" : "false", stats.binary_threshold,
      modules::LightbarThresholdSourceName(stats.threshold_source), stats.reference_lightbars,
      stats.contours, stats.geometry_candidates, stats.color_candidates, stats.kept_candidates,
      stats.elapsed_ms, prediction.detected_lightbar_count, prediction.deduplicated_lightbar_count,
      prediction.matched_lightbar_count, prediction.accepted_lightbar_count,
      prediction.rejected_lightbar_count, prediction.light_only_pair_count,
      prediction.light_only_update ? "true" : "false",
      prediction.light_only_update_blocked ? "true" : "false",
      EscapeJson(prediction.light_only_rejection_reason),
      prediction.light_fusion_used ? "true" : "false",
      prediction.armor_fallback_used ? "true" : "false", EscapeJson(stats.rejection_reason));
}

}  // namespace mv::tool::foxglove::armor_light_detector
