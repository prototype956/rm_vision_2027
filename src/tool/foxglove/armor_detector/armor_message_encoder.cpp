#include "tool/foxglove/armor_detector/armor_message_encoder.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <opencv2/imgcodecs.hpp>

namespace mv::tool::foxglove::armor_detector {
namespace {

constexpr std::uint64_t K_NANOSECONDS_PER_SECOND = 1'000'000'000ULL;

double Milliseconds(SteadyClock::duration duration) noexcept {
  return std::chrono::duration<double, std::milli>(duration).count();
}

::foxglove::schemas::Color AnnotationColor(modules::ArmorColor color) {
  if (color == modules::ArmorColor::RED) {
    return {.r = 1.0, .g = 0.0, .b = 0.0, .a = 1.0};
  }
  return {.r = 0.0, .g = 0.5, .b = 1.0, .a = 1.0};
}

::foxglove::schemas::ImageAnnotations MakeAnnotations(
    const std::vector<modules::ArmorDetection>& detections,
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

std::string MakeStatsJson(const FrameItem& item, const ::foxglove::schemas::Timestamp& timestamp,
                          std::optional<double> jpeg_ms, double latency_ms, PipelineCounts counts) {
  const auto JPEG = jpeg_ms.has_value() ? fmt::format("{:.3f}", *jpeg_ms) : "null";
  return fmt::format(
      "{{\"timestamp\":{{\"sec\":{},\"nsec\":{}}},\"sequence\":{},"
      "\"preprocess_ms\":{:.3f},\"inference_ms\":{:.3f},"
      "\"postprocess_ms\":{:.3f},\"total_ms\":{:.3f},"
      "\"threshold_candidates\":{},\"kept_detections\":{},"
      "\"jpeg_encode_ms\":{},\"publish_latency_ms\":{:.3f},"
      "\"rate_limited_frames\":{},\"queue_overwritten_frames\":{}}}",
      timestamp.sec, timestamp.nsec, item.sequence, item.detector_stats.preprocess_ms,
      item.detector_stats.inference_ms, item.detector_stats.postprocess_ms,
      item.detector_stats.total_ms, item.detector_stats.threshold_candidates,
      item.detector_stats.kept_detections, JPEG, latency_ms, counts.rate_limited_frames,
      counts.queue_overwritten_frames);
}

}  // namespace

TopicDemand Merge(TopicDemand left, TopicDemand right) noexcept {
  return {.image = left.image || right.image,
          .annotations = left.annotations || right.annotations,
          .stats = left.stats || right.stats};
}

ArmorMessageEncoder::ArmorMessageEncoder(const ImageConfig& config)
    : config_(config),
      steady_anchor_(SteadyClock::now()),
      system_anchor_(std::chrono::system_clock::now()) {}

PreparedFrame ArmorMessageEncoder::Encode(const FrameItem& item, TopicDemand demand,
                                          PipelineCounts counts) const {
  // 用构造时的双时钟锚点将相机单调时间转换为 epoch，避免运行中系统校时造成跳变。
  const auto SYSTEM_TIME = system_anchor_ + (item.timestamp - steady_anchor_);
  const auto EPOCH_COUNT =
      std::chrono::duration_cast<std::chrono::nanoseconds>(SYSTEM_TIME.time_since_epoch()).count();
  const auto EPOCH_NANOS = EPOCH_COUNT > 0 ? static_cast<std::uint64_t>(EPOCH_COUNT) : 0;
  const ::foxglove::schemas::Timestamp TIMESTAMP{
      .sec = static_cast<std::uint32_t>(EPOCH_NANOS / K_NANOSECONDS_PER_SECOND),
      .nsec = static_cast<std::uint32_t>(EPOCH_NANOS % K_NANOSECONDS_PER_SECOND)};

  PreparedFrame result;
  result.epoch_nanos = EPOCH_NANOS;
  // 图像只有被实时订阅或需要录制时才编码，两个 sink 共享下面生成的同一消息。
  if (demand.image) {
    const auto START = SteadyClock::now();
    std::vector<unsigned char> encoded;
    if (!cv::imencode(".jpg", item.image, encoded,
                      {cv::IMWRITE_JPEG_QUALITY, config_.jpeg_quality})) {
      throw std::runtime_error("cv::imencode returned false");
    }
    result.jpeg_ms = Milliseconds(SteadyClock::now() - START);
    ::foxglove::schemas::CompressedImage message;
    message.timestamp = TIMESTAMP;
    message.frame_id = config_.frame_id;
    message.format = "jpeg";
    message.data.resize(encoded.size());
    std::memcpy(message.data.data(), encoded.data(), encoded.size());
    result.image = std::move(message);
  }
  if (demand.annotations) {
    result.annotations = MakeAnnotations(item.detections, TIMESTAMP);
  }
  result.publish_latency_ms = Milliseconds(SteadyClock::now() - item.timestamp);
  if (demand.stats) {
    result.stats_json =
        MakeStatsJson(item, TIMESTAMP, result.jpeg_ms, result.publish_latency_ms, counts);
  }
  return result;
}

}  // namespace mv::tool::foxglove::armor_detector
