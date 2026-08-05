#include "tool/foxglove/armor_detector/armor_channel_set.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace mv::tool::foxglove::armor_detector {
namespace {

constexpr char K_IMAGE_TOPIC[] = "/vision/camera/image";
constexpr char K_ANNOTATION_TOPIC[] = "/vision/armor/annotations";
constexpr char K_STATS_TOPIC[] = "/vision/armor/stats";

// RawChannel 必须携带稳定 JSON Schema，Foxglove 才能直接在 Plot 面板解析字段。
constexpr char K_STATS_JSON_SCHEMA[] = R"json({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object","properties":{"sec":{"type":"integer"},"nsec":{"type":"integer"}},"required":["sec","nsec"]},
    "sequence":{"type":"integer"},
    "preprocess_ms":{"type":"number"},
    "inference_ms":{"type":"number"},
    "postprocess_ms":{"type":"number"},
    "total_ms":{"type":"number"},
    "threshold_candidates":{"type":"integer"},
    "kept_detections":{"type":"integer"},
    "jpeg_encode_ms":{"type":["number","null"]},
    "publish_latency_ms":{"type":"number"},
    "rate_limited_frames":{"type":"integer"},
    "queue_overwritten_frames":{"type":"integer"}
  },
  "required":["timestamp","sequence","preprocess_ms","inference_ms","postprocess_ms","total_ms","threshold_candidates","kept_detections","jpeg_encode_ms","publish_latency_ms","rate_limited_frames","queue_overwritten_frames"]
})json";

::foxglove::Schema StatsSchema() {
  return {.name = "mv.vision.DetectorStats",
          .encoding = "jsonschema",
          .data = reinterpret_cast<const std::byte*>(K_STATS_JSON_SCHEMA),
          .data_len = sizeof(K_STATS_JSON_SCHEMA) - 1};
}

std::runtime_error SdkError(const std::string& operation, ::foxglove::FoxgloveError error) {
  return std::runtime_error(operation + ": " + ::foxglove::strerror(error));
}

void AddError(ChannelPublishResult& result, ArmorTopic topic,
              ::foxglove::FoxgloveError error) noexcept {
  if (error == ::foxglove::FoxgloveError::Ok) {
    return;
  }
  result.success = false;
  result.errors[result.error_count++] = {.topic = topic, .error = error};
}

}  // namespace

ArmorChannelSet::ArmorChannelSet(const ::foxglove::Context& context) {
  auto image_result = ::foxglove::schemas::CompressedImageChannel::create(K_IMAGE_TOPIC, context);
  if (!image_result.has_value()) {
    throw SdkError("create image channel", image_result.error());
  }
  image_ = std::make_unique<::foxglove::schemas::CompressedImageChannel>(
      std::move(image_result).value());

  auto annotation_result =
      ::foxglove::schemas::ImageAnnotationsChannel::create(K_ANNOTATION_TOPIC, context);
  if (!annotation_result.has_value()) {
    throw SdkError("create annotation channel", annotation_result.error());
  }
  annotations_ = std::make_unique<::foxglove::schemas::ImageAnnotationsChannel>(
      std::move(annotation_result).value());

  auto stats_result = ::foxglove::RawChannel::create(K_STATS_TOPIC, "json", StatsSchema(), context);
  if (!stats_result.has_value()) {
    throw SdkError("create stats channel", stats_result.error());
  }
  stats_ = std::make_unique<::foxglove::RawChannel>(std::move(stats_result).value());
}

ArmorChannelSet::~ArmorChannelSet() {
  Close();
}

ChannelIds ArmorChannelSet::Ids() const noexcept {
  return {.image = image_->id(), .annotations = annotations_->id(), .stats = stats_->id()};
}

ChannelPublishResult ArmorChannelSet::Publish(const PreparedFrame& frame,
                                              TopicDemand demand) noexcept {
  ChannelPublishResult result;
  if (demand.image && frame.image.has_value()) {
    result.attempted = true;
    AddError(result, ArmorTopic::IMAGE, image_->log(*frame.image, frame.epoch_nanos));
  }
  if (demand.annotations && frame.annotations.has_value()) {
    result.attempted = true;
    AddError(result, ArmorTopic::ANNOTATIONS,
             annotations_->log(*frame.annotations, frame.epoch_nanos));
  }
  if (demand.stats && frame.stats_json.has_value()) {
    result.attempted = true;
    const auto* data = reinterpret_cast<const std::byte*>(frame.stats_json->data());
    AddError(result, ArmorTopic::STATS,
             stats_->log(data, frame.stats_json->size(), frame.epoch_nanos));
  }
  return result;
}

void ArmorChannelSet::Close() noexcept {
  if (closed_) {
    return;
  }
  closed_ = true;
  if (image_) {
    image_->close();
  }
  if (annotations_) {
    annotations_->close();
  }
  if (stats_) {
    stats_->close();
  }
}

const char* TopicName(ArmorTopic topic) noexcept {
  switch (topic) {
    case ArmorTopic::IMAGE:
      return "image";
    case ArmorTopic::ANNOTATIONS:
      return "annotations";
    case ArmorTopic::STATS:
      return "stats";
  }
  return "unknown";
}

}  // namespace mv::tool::foxglove::armor_detector
