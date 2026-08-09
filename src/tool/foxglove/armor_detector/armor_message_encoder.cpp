#include "tool/foxglove/armor_detector/armor_message_encoder.hpp"

#include <utility>

#include <fmt/format.h>

namespace mv::tool::foxglove::armor_detector {
namespace {

::foxglove::schemas::Color AnnotationColor(modules::ArmorColor color) {
  if (color == modules::ArmorColor::RED) {
    return {.r = 1.0, .g = 0.0, .b = 0.0, .a = 1.0};
  }
  return {.r = 0.0, .g = 0.5, .b = 1.0, .a = 1.0};
}

}  // namespace

::foxglove::schemas::ImageAnnotations EncodeAnnotations(
    std::span<const modules::ArmorDetection> detections,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::ImageAnnotations annotations;

  // 旧版 SDK 没有 ImageAnnotations 顶层时间戳，用不可见点携带清屏帧时间戳。
  ::foxglove::schemas::PointsAnnotation timestamp_carrier;
  timestamp_carrier.timestamp = timestamp;
  timestamp_carrier.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::POINTS;
  timestamp_carrier.thickness = 0.0;
  annotations.points.push_back(std::move(timestamp_carrier));

  for (const auto& detection : detections) {
    const auto COLOR = AnnotationColor(detection.color);
    ::foxglove::schemas::PointsAnnotation polygon;
    polygon.timestamp = timestamp;
    polygon.type = ::foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    polygon.outline_color = COLOR;
    polygon.thickness = 2.0;
    polygon.points.reserve(detection.corners.size());
    for (const auto& corner : detection.corners) {
      polygon.points.push_back({.x = corner.x, .y = corner.y});
    }
    annotations.points.push_back(std::move(polygon));

    ::foxglove::schemas::TextAnnotation text;
    text.timestamp = timestamp;
    text.position =
        ::foxglove::schemas::Point2{.x = detection.bounding_box.x, .y = detection.bounding_box.y};
    text.text = fmt::format("{} {} {:.2f}", modules::ArmorColorName(detection.color),
                            modules::ArmorLabelName(detection.label), detection.objectness);
    text.font_size = 14.0;
    text.text_color = COLOR;
    text.background_color = ::foxglove::schemas::Color{.r = 0.0, .g = 0.0, .b = 0.0, .a = 0.7};
    annotations.texts.push_back(std::move(text));
  }
  return annotations;
}

std::string EncodeDetectorStats(const modules::DetectorStats& stats, std::uint64_t sequence,
                                const ::foxglove::schemas::Timestamp& timestamp) {
  return fmt::format(
      "{{\"timestamp\":{{\"sec\":{},\"nsec\":{}}},\"sequence\":{},"
      "\"preprocess_ms\":{:.3f},\"inference_ms\":{:.3f},"
      "\"postprocess_ms\":{:.3f},\"total_ms\":{:.3f},"
      "\"threshold_candidates\":{},\"kept_detections\":{}}}",
      timestamp.sec, timestamp.nsec, sequence, stats.preprocess_ms, stats.inference_ms,
      stats.postprocess_ms, stats.total_ms, stats.threshold_candidates, stats.kept_detections);
}

}  // namespace mv::tool::foxglove::armor_detector
