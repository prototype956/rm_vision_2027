#include "tool/foxglove/pipeline/vision_message_encoder.hpp"

#include "tool/foxglove/armor_detector/armor_message_encoder.hpp"
#include "tool/foxglove/image/image_message_encoder.hpp"
#include "tool/foxglove/pnp/pnp_message_encoder.hpp"
#include "tool/foxglove/prediction/prediction_message_encoder.hpp"
#include "tool/foxglove/simulation/simulation_message_encoder.hpp"
#include "tool/foxglove/spatial/spatial_message_encoder.hpp"

#include <fmt/format.h>

namespace mv::tool::foxglove::pipeline {
namespace {

constexpr std::uint64_t K_NANOSECONDS_PER_SECOND = 1'000'000'000ULL;

double Milliseconds(SteadyClock::duration duration) noexcept {
  return std::chrono::duration<double, std::milli>(duration).count();
}

std::string EncodeDebugStats(const VisionDebugFrame& frame,
                             const ::foxglove::schemas::Timestamp& timestamp,
                             std::optional<double> jpeg_ms, double publish_latency_ms,
                             PipelineCounts counts) {
  const auto JPEG = jpeg_ms.has_value() ? fmt::format("{:.3f}", *jpeg_ms) : "null";
  const auto CAPTURE_TIMESTAMP =
      frame.capture_timestamp_ns.has_value() ? std::to_string(*frame.capture_timestamp_ns) : "null";
  return fmt::format(
      "{{\"timestamp\":{{\"sec\":{},\"nsec\":{}}},\"sequence\":{},"
      "\"capture_timestamp_ns\":{},\"geometry_valid\":{},"
      "\"source_invalid_frames\":{},\"jpeg_encode_ms\":{},"
      "\"publish_latency_ms\":{:.3f},\"rate_limited_frames\":{},"
      "\"queue_overwritten_frames\":{}}}",
      timestamp.sec, timestamp.nsec, frame.sequence, CAPTURE_TIMESTAMP,
      frame.geometry.has_value() ? "true" : "false", frame.source_invalid_frames, JPEG,
      publish_latency_ms, counts.rate_limited_frames, counts.queue_overwritten_frames);
}

}  // namespace

bool TopicDemand::Any() const noexcept {
  return image || armor_annotations || armor_stats || debug_stats || transforms || calibration ||
         frustum || ground_truth || projection_annotations || pnp_estimates || pnp_corners ||
         pnp_reprojection || pnp_error_vectors || corner_refiner_axes ||
         corner_refiner_candidates || pnp_stats || prediction_scene || prediction_state ||
         prediction_truth_overlay || prediction_current_annotations ||
         prediction_future_annotations;
}

TopicDemand Merge(TopicDemand left, TopicDemand right) noexcept {
  return {
      .image = left.image || right.image,
      .armor_annotations = left.armor_annotations || right.armor_annotations,
      .armor_stats = left.armor_stats || right.armor_stats,
      .debug_stats = left.debug_stats || right.debug_stats,
      .transforms = left.transforms || right.transforms,
      .calibration = left.calibration || right.calibration,
      .frustum = left.frustum || right.frustum,
      .ground_truth = left.ground_truth || right.ground_truth,
      .projection_annotations = left.projection_annotations || right.projection_annotations,
      .pnp_estimates = left.pnp_estimates || right.pnp_estimates,
      .pnp_corners = left.pnp_corners || right.pnp_corners,
      .pnp_reprojection = left.pnp_reprojection || right.pnp_reprojection,
      .pnp_error_vectors = left.pnp_error_vectors || right.pnp_error_vectors,
      .corner_refiner_axes = left.corner_refiner_axes || right.corner_refiner_axes,
      .corner_refiner_candidates =
          left.corner_refiner_candidates || right.corner_refiner_candidates,
      .pnp_stats = left.pnp_stats || right.pnp_stats,
      .prediction_scene = left.prediction_scene || right.prediction_scene,
      .prediction_state = left.prediction_state || right.prediction_state,
      .prediction_truth_overlay = left.prediction_truth_overlay || right.prediction_truth_overlay,
      .prediction_current_annotations =
          left.prediction_current_annotations || right.prediction_current_annotations,
      .prediction_future_annotations =
          left.prediction_future_annotations || right.prediction_future_annotations};
}

VisionMessageEncoder::VisionMessageEncoder(const ImageConfig& config)
    : config_(config),
      steady_anchor_(SteadyClock::now()),
      system_anchor_(std::chrono::system_clock::now()) {}

PreparedFrame VisionMessageEncoder::Encode(const VisionDebugFrame& frame, TopicDemand demand,
                                           PipelineCounts counts) const {
  // 实机帧没有 epoch 时间时，用固定双时钟锚点换算，避免运行中系统校时造成时间跳变。
  const auto SYSTEM_TIME = system_anchor_ + (frame.receive_steady_time - steady_anchor_);
  const auto FALLBACK_EPOCH_COUNT =
      std::chrono::duration_cast<std::chrono::nanoseconds>(SYSTEM_TIME.time_since_epoch()).count();
  // Talos 等仿真源优先保留原始采集 epoch，使图像、TF 和真值与仿真快照严格同帧。
  const auto EPOCH_NANOS = frame.capture_timestamp_ns.value_or(
      FALLBACK_EPOCH_COUNT > 0 ? static_cast<std::uint64_t>(FALLBACK_EPOCH_COUNT) : 0);
  const ::foxglove::schemas::Timestamp TIMESTAMP{
      .sec = static_cast<std::uint32_t>(EPOCH_NANOS / K_NANOSECONDS_PER_SECOND),
      .nsec = static_cast<std::uint32_t>(EPOCH_NANOS % K_NANOSECONDS_PER_SECOND)};

  PreparedFrame result;
  result.epoch_nanos = EPOCH_NANOS;
  if (demand.image) {
    auto encoded =
        image::EncodeJpeg(frame.image, config_.jpeg_quality, config_.frame_id, TIMESTAMP);
    result.jpeg_ms = encoded.jpeg_ms;
    result.image = std::move(encoded.message);
  }
  if (demand.armor_annotations) {
    result.armor_annotations = armor_detector::EncodeAnnotations(frame.detections, TIMESTAMP);
  }
  if (demand.armor_stats) {
    result.armor_stats_json =
        armor_detector::EncodeDetectorStats(frame.detector_stats, frame.sequence, TIMESTAMP);
  }
  // 空间与仿真领域共用同一份 geometry；后端未提供时不构造任何默认坐标关系。
  if (frame.geometry.has_value()) {
    const auto& geometry = *frame.geometry;
    if (demand.transforms) {
      result.transforms = spatial::EncodeTransforms(geometry, TIMESTAMP);
    }
    if (demand.calibration) {
      result.calibration = spatial::EncodeCalibration(geometry.calibration, TIMESTAMP);
    }
    if (demand.frustum) {
      result.frustum = spatial::EncodeFrustum(geometry.calibration, TIMESTAMP);
    }
    if (demand.ground_truth) {
      result.ground_truth = simulation::EncodeGroundTruth(geometry, TIMESTAMP);
    }
    if (demand.projection_annotations) {
      result.projection_annotations = simulation::EncodeProjectionAnnotations(geometry, TIMESTAMP);
    }
    if (demand.pnp_estimates) {
      result.pnp_estimates = pnp::EncodeEstimates(frame.pnp_result, geometry, TIMESTAMP);
    }
    if (demand.prediction_truth_overlay) {
      result.prediction_truth_overlay =
          prediction::EncodeTruthOverlay(frame.prediction_result, geometry, TIMESTAMP);
    }
  }
  if (demand.prediction_current_annotations) {
    result.prediction_current_annotations =
        frame.geometry
            ? prediction::EncodeAnnotations(frame.prediction_result, *frame.geometry,
                                            prediction::ImagePredictionHorizon::CURRENT, TIMESTAMP)
            : prediction::EncodeEmptyAnnotations(TIMESTAMP);
  }
  if (demand.prediction_future_annotations) {
    result.prediction_future_annotations =
        frame.geometry ? prediction::EncodeAnnotations(
                             frame.prediction_result, *frame.geometry,
                             prediction::ImagePredictionHorizon::FUTURE_100_MS, TIMESTAMP)
                       : prediction::EncodeEmptyAnnotations(TIMESTAMP);
  }
  if (demand.pnp_corners) {
    result.pnp_corners = pnp::EncodeCorners(frame.pnp_result, TIMESTAMP);
  }
  if (demand.pnp_reprojection) {
    result.pnp_reprojection = pnp::EncodeReprojection(frame.pnp_result, TIMESTAMP);
  }
  if (demand.pnp_error_vectors) {
    result.pnp_error_vectors = pnp::EncodeErrorVectors(frame.pnp_result, TIMESTAMP);
  }
  if (demand.corner_refiner_axes) {
    result.corner_refiner_axes = pnp::EncodeCornerRefinerAxes(frame.pnp_result, TIMESTAMP);
  }
  if (demand.corner_refiner_candidates) {
    result.corner_refiner_candidates =
        pnp::EncodeCornerRefinerCandidates(frame.pnp_result, TIMESTAMP);
  }
  if (demand.pnp_stats) {
    result.pnp_stats_json = pnp::EncodeStats(frame.pnp_result, frame.sequence, TIMESTAMP);
  }
  if (demand.prediction_scene) {
    result.prediction_scene = prediction::EncodeScene(frame.prediction_result, TIMESTAMP);
  }
  if (demand.prediction_state) {
    result.prediction_state_json = prediction::EncodeState(frame.prediction_result, TIMESTAMP);
  }

  result.publish_latency_ms = Milliseconds(SteadyClock::now() - frame.receive_steady_time);
  if (demand.debug_stats) {
    result.debug_stats_json =
        EncodeDebugStats(frame, TIMESTAMP, result.jpeg_ms, result.publish_latency_ms, counts);
  }
  return result;
}

}  // namespace mv::tool::foxglove::pipeline
