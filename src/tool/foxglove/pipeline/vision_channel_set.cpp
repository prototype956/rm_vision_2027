#include "tool/foxglove/pipeline/vision_channel_set.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace mv::tool::foxglove::pipeline {
namespace {

constexpr char K_IMAGE_TOPIC[] = "/vision/camera/image";
constexpr char K_ARMOR_ANNOTATIONS_TOPIC[] = "/vision/armor/annotations";
constexpr char K_ARMOR_STATS_TOPIC[] = "/vision/armor/stats";
constexpr char K_DEBUG_STATS_TOPIC[] = "/vision/debug/stats";
constexpr char K_TRANSFORMS_TOPIC[] = "/vision/transforms";
constexpr char K_CALIBRATION_TOPIC[] = "/vision/camera/calibration";
constexpr char K_FRUSTUM_TOPIC[] = "/vision/camera/frustum";
constexpr char K_GROUND_TRUTH_TOPIC[] = "/simulation/ground_truth";
constexpr char K_PROJECTION_ANNOTATIONS_TOPIC[] = "/simulation/ground_truth/annotations";

// RawChannel 必须携带稳定 JSON Schema，Foxglove Plot/Raw Messages 才能解析字段。
constexpr char K_ARMOR_STATS_SCHEMA[] = R"json({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object","properties":{"sec":{"type":"integer"},"nsec":{"type":"integer"}},"required":["sec","nsec"]},
    "sequence":{"type":"integer"},
    "preprocess_ms":{"type":"number"},
    "inference_ms":{"type":"number"},
    "postprocess_ms":{"type":"number"},
    "total_ms":{"type":"number"},
    "threshold_candidates":{"type":"integer"},
    "kept_detections":{"type":"integer"}
  },
  "required":["timestamp","sequence","preprocess_ms","inference_ms","postprocess_ms","total_ms","threshold_candidates","kept_detections"]
})json";

constexpr char K_DEBUG_STATS_SCHEMA[] = R"json({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object","properties":{"sec":{"type":"integer"},"nsec":{"type":"integer"}},"required":["sec","nsec"]},
    "sequence":{"type":"integer"},
    "capture_timestamp_ns":{"type":["integer","null"]},
    "geometry_valid":{"type":"boolean"},
    "source_invalid_frames":{"type":"integer"},
    "jpeg_encode_ms":{"type":["number","null"]},
    "publish_latency_ms":{"type":"number"},
    "rate_limited_frames":{"type":"integer"},
    "queue_overwritten_frames":{"type":"integer"}
  },
  "required":["timestamp","sequence","capture_timestamp_ns","geometry_valid","source_invalid_frames","jpeg_encode_ms","publish_latency_ms","rate_limited_frames","queue_overwritten_frames"]
})json";

::foxglove::Schema JsonSchema(const char* name, const char* data, std::size_t size) {
  return {.name = name,
          .encoding = "jsonschema",
          .data = reinterpret_cast<const std::byte*>(data),
          .data_len = size};
}

std::runtime_error SdkError(const std::string& operation, ::foxglove::FoxgloveError error) {
  return std::runtime_error(operation + ": " + ::foxglove::strerror(error));
}

void AddError(ChannelPublishResult& result, VisionTopic topic,
              ::foxglove::FoxgloveError error) noexcept {
  if (error == ::foxglove::FoxgloveError::Ok) {
    return;
  }
  result.success = false;
  result.errors[result.error_count++] = {.topic = topic, .error = error};
}

template <typename Channel>
std::unique_ptr<Channel> CreateSchemaChannel(const char* topic, const ::foxglove::Context& context,
                                             const char* description) {
  auto result = Channel::create(topic, context);
  if (!result.has_value()) {
    throw SdkError(description, result.error());
  }
  return std::make_unique<Channel>(std::move(result).value());
}

std::unique_ptr<::foxglove::RawChannel> CreateRawChannel(const char* topic, const char* schema_name,
                                                         const char* schema,
                                                         std::size_t schema_size,
                                                         const ::foxglove::Context& context) {
  auto result = ::foxglove::RawChannel::create(
      topic, "json", JsonSchema(schema_name, schema, schema_size), context);
  if (!result.has_value()) {
    throw SdkError(std::string("create ") + topic + " channel", result.error());
  }
  return std::make_unique<::foxglove::RawChannel>(std::move(result).value());
}

}  // namespace

VisionChannelSet::VisionChannelSet(const ::foxglove::Context& context) {
  image_ = CreateSchemaChannel<::foxglove::schemas::CompressedImageChannel>(K_IMAGE_TOPIC, context,
                                                                            "create image channel");
  armor_annotations_ = CreateSchemaChannel<::foxglove::schemas::ImageAnnotationsChannel>(
      K_ARMOR_ANNOTATIONS_TOPIC, context, "create armor annotations channel");
  armor_stats_ = CreateRawChannel(K_ARMOR_STATS_TOPIC, "mv.vision.ArmorDetectorStats",
                                  K_ARMOR_STATS_SCHEMA, sizeof(K_ARMOR_STATS_SCHEMA) - 1, context);
  debug_stats_ = CreateRawChannel(K_DEBUG_STATS_TOPIC, "mv.vision.DebugPipelineStats",
                                  K_DEBUG_STATS_SCHEMA, sizeof(K_DEBUG_STATS_SCHEMA) - 1, context);
  transforms_ = CreateSchemaChannel<::foxglove::schemas::FrameTransformsChannel>(
      K_TRANSFORMS_TOPIC, context, "create transforms channel");
  calibration_ = CreateSchemaChannel<::foxglove::schemas::CameraCalibrationChannel>(
      K_CALIBRATION_TOPIC, context, "create calibration channel");
  frustum_ = CreateSchemaChannel<::foxglove::schemas::SceneUpdateChannel>(K_FRUSTUM_TOPIC, context,
                                                                          "create frustum channel");
  ground_truth_ = CreateSchemaChannel<::foxglove::schemas::SceneUpdateChannel>(
      K_GROUND_TRUTH_TOPIC, context, "create ground truth channel");
  projection_annotations_ = CreateSchemaChannel<::foxglove::schemas::ImageAnnotationsChannel>(
      K_PROJECTION_ANNOTATIONS_TOPIC, context, "create projection annotations channel");
}

VisionChannelSet::~VisionChannelSet() {
  Close();
}

ChannelIds VisionChannelSet::Ids() const noexcept {
  return {.image = image_->id(),
          .armor_annotations = armor_annotations_->id(),
          .armor_stats = armor_stats_->id(),
          .debug_stats = debug_stats_->id(),
          .transforms = transforms_->id(),
          .calibration = calibration_->id(),
          .frustum = frustum_->id(),
          .ground_truth = ground_truth_->id(),
          .projection_annotations = projection_annotations_->id()};
}

ChannelPublishResult VisionChannelSet::Publish(const PreparedFrame& frame,
                                               TopicDemand demand) noexcept {
  ChannelPublishResult result;
  if (demand.image && frame.image.has_value()) {
    result.attempted = true;
    AddError(result, VisionTopic::IMAGE, image_->log(*frame.image, frame.epoch_nanos));
  }
  if (demand.armor_annotations && frame.armor_annotations.has_value()) {
    result.attempted = true;
    AddError(result, VisionTopic::ARMOR_ANNOTATIONS,
             armor_annotations_->log(*frame.armor_annotations, frame.epoch_nanos));
  }
  if (demand.armor_stats && frame.armor_stats_json.has_value()) {
    result.attempted = true;
    const auto* data = reinterpret_cast<const std::byte*>(frame.armor_stats_json->data());
    AddError(result, VisionTopic::ARMOR_STATS,
             armor_stats_->log(data, frame.armor_stats_json->size(), frame.epoch_nanos));
  }
  if (demand.debug_stats && frame.debug_stats_json.has_value()) {
    result.attempted = true;
    const auto* data = reinterpret_cast<const std::byte*>(frame.debug_stats_json->data());
    AddError(result, VisionTopic::DEBUG_STATS,
             debug_stats_->log(data, frame.debug_stats_json->size(), frame.epoch_nanos));
  }
  if (demand.transforms && frame.transforms.has_value()) {
    result.attempted = true;
    AddError(result, VisionTopic::TRANSFORMS,
             transforms_->log(*frame.transforms, frame.epoch_nanos));
  }
  if (demand.calibration && frame.calibration.has_value()) {
    result.attempted = true;
    AddError(result, VisionTopic::CALIBRATION,
             calibration_->log(*frame.calibration, frame.epoch_nanos));
  }
  if (demand.frustum && frame.frustum.has_value()) {
    result.attempted = true;
    AddError(result, VisionTopic::FRUSTUM, frustum_->log(*frame.frustum, frame.epoch_nanos));
  }
  if (demand.ground_truth && frame.ground_truth.has_value()) {
    result.attempted = true;
    AddError(result, VisionTopic::GROUND_TRUTH,
             ground_truth_->log(*frame.ground_truth, frame.epoch_nanos));
  }
  if (demand.projection_annotations && frame.projection_annotations.has_value()) {
    result.attempted = true;
    AddError(result, VisionTopic::PROJECTION_ANNOTATIONS,
             projection_annotations_->log(*frame.projection_annotations, frame.epoch_nanos));
  }
  return result;
}

void VisionChannelSet::Close() noexcept {
  if (closed_) {
    return;
  }
  closed_ = true;
  if (image_)
    image_->close();
  if (armor_annotations_)
    armor_annotations_->close();
  if (armor_stats_)
    armor_stats_->close();
  if (debug_stats_)
    debug_stats_->close();
  if (transforms_)
    transforms_->close();
  if (calibration_)
    calibration_->close();
  if (frustum_)
    frustum_->close();
  if (ground_truth_)
    ground_truth_->close();
  if (projection_annotations_)
    projection_annotations_->close();
}

const char* TopicName(VisionTopic topic) noexcept {
  switch (topic) {
    case VisionTopic::IMAGE:
      return "image";
    case VisionTopic::ARMOR_ANNOTATIONS:
      return "armor_annotations";
    case VisionTopic::ARMOR_STATS:
      return "armor_stats";
    case VisionTopic::DEBUG_STATS:
      return "debug_stats";
    case VisionTopic::TRANSFORMS:
      return "transforms";
    case VisionTopic::CALIBRATION:
      return "calibration";
    case VisionTopic::FRUSTUM:
      return "frustum";
    case VisionTopic::GROUND_TRUTH:
      return "ground_truth";
    case VisionTopic::PROJECTION_ANNOTATIONS:
      return "projection_annotations";
  }
  return "unknown";
}

}  // namespace mv::tool::foxglove::pipeline
