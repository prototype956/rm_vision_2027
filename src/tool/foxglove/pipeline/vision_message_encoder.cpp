#include "tool/foxglove/pipeline/vision_message_encoder.hpp"

#include "tool/foxglove/armor_detector/armor_message_encoder.hpp"
#include "tool/foxglove/armor_light_detector/lightbar_message_encoder.hpp"
#include "tool/foxglove/image/image_message_encoder.hpp"
#include "tool/foxglove/pnp/pnp_message_encoder.hpp"
#include "tool/foxglove/prediction/prediction_message_encoder.hpp"
#include "tool/foxglove/simulation/combat_message_encoder.hpp"
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
  const auto& packet = frame.packet;
  const auto& stamp = packet.capture.stamp;
  const auto CAPTURE_TIMESTAMP =
      stamp.capture_timestamp_ns.has_value() ? std::to_string(*stamp.capture_timestamp_ns) : "null";
  return fmt::format(
      "{{\"timestamp\":{{\"sec\":{},\"nsec\":{}}},\"sequence\":{},"
      "\"capture_timestamp_ns\":{},\"geometry_valid\":{},"
      "\"source_invalid_frames\":{},\"jpeg_encode_ms\":{},"
      "\"publish_latency_ms\":{:.3f},\"rate_limited_frames\":{},"
      "\"queue_overwritten_frames\":{}}}",
      timestamp.sec, timestamp.nsec, stamp.sequence, CAPTURE_TIMESTAMP,
      mv::frame::MakeSpatialFrameView(packet).has_value() ? "true" : "false",
      stamp.source_invalid_frames, JPEG, publish_latency_ms, counts.rate_limited_frames,
      counts.queue_overwritten_frames);
}

}  // namespace

bool TopicDemand::Any() const noexcept {
  return image || armor_annotations || armor_stats || lightbar_annotations || lightbar_stats ||
         debug_stats || real_transforms || calibration || frustum || ground_truth || projectile_stats ||
         referee_state || combat_evaluation || projection_annotations || pnp_estimates ||
         pnp_raw_corners || pnp_final_corners || pnp_reprojection || pnp_error_vectors ||
         corner_refiner_axes || corner_refiner_candidates || pnp_stats || prediction_scene ||
         impact_scene || prediction_state || prediction_truth_overlay ||
         prediction_current_annotations || impact_annotations || selected_armor_annotations;
}

TopicDemand Merge(TopicDemand left, TopicDemand right) noexcept {
  return {
      .image = left.image || right.image,
      .armor_annotations = left.armor_annotations || right.armor_annotations,
      .armor_stats = left.armor_stats || right.armor_stats,
      .lightbar_annotations = left.lightbar_annotations || right.lightbar_annotations,
      .lightbar_stats = left.lightbar_stats || right.lightbar_stats,
      .debug_stats = left.debug_stats || right.debug_stats,
      .real_transforms = left.real_transforms || right.real_transforms,
      .calibration = left.calibration || right.calibration,
      .frustum = left.frustum || right.frustum,
      .ground_truth = left.ground_truth || right.ground_truth,
      .projectile_stats = left.projectile_stats || right.projectile_stats,
      .referee_state = left.referee_state || right.referee_state,
      .combat_evaluation = left.combat_evaluation || right.combat_evaluation,
      .projection_annotations = left.projection_annotations || right.projection_annotations,
      .pnp_estimates = left.pnp_estimates || right.pnp_estimates,
      .pnp_raw_corners = left.pnp_raw_corners || right.pnp_raw_corners,
      .pnp_final_corners = left.pnp_final_corners || right.pnp_final_corners,
      .pnp_reprojection = left.pnp_reprojection || right.pnp_reprojection,
      .pnp_error_vectors = left.pnp_error_vectors || right.pnp_error_vectors,
      .corner_refiner_axes = left.corner_refiner_axes || right.corner_refiner_axes,
      .corner_refiner_candidates =
          left.corner_refiner_candidates || right.corner_refiner_candidates,
      .pnp_stats = left.pnp_stats || right.pnp_stats,
      .prediction_scene = left.prediction_scene || right.prediction_scene,
      .impact_scene = left.impact_scene || right.impact_scene,
      .prediction_state = left.prediction_state || right.prediction_state,
      .prediction_truth_overlay = left.prediction_truth_overlay || right.prediction_truth_overlay,
      .prediction_current_annotations =
          left.prediction_current_annotations || right.prediction_current_annotations,
      .impact_annotations = left.impact_annotations || right.impact_annotations,
      .selected_armor_annotations =
          left.selected_armor_annotations || right.selected_armor_annotations};
}

VisionMessageEncoder::VisionMessageEncoder(const ImageConfig& config)
    : config_(config),
      steady_anchor_(SteadyClock::now()),
      system_anchor_(std::chrono::system_clock::now()) {}

PreparedFrame VisionMessageEncoder::Encode(
    const VisionDebugFrame& frame, TopicDemand demand, PipelineCounts counts,
    const std::optional<modules::ArmorImpactSnapshot>& impact) const {
  const auto& packet = frame.packet;
  const auto& stamp = packet.capture.stamp;
  const auto SPATIAL_VIEW = mv::frame::MakeSpatialFrameView(packet);
  const auto PNP_DIAGNOSTIC =
      frame.simulation_evaluation
          ? frame.simulation_evaluation->pnp
          : simulation_evaluation::MakePnpDiagnosticResult(frame.output.pnp, frame.diagnostics.pnp,
                                                           frame.diagnostics.refinements);
  const auto* prediction_evaluation =
      frame.simulation_evaluation ? &frame.simulation_evaluation->prediction : nullptr;
  // 实机帧没有 epoch 时间时，用固定双时钟锚点换算，避免运行中系统校时造成时间跳变。
  const auto CAPTURE_TIME = stamp.capture_steady_time.value_or(stamp.receive_steady_time);
  const auto SYSTEM_TIME = system_anchor_ + (CAPTURE_TIME - steady_anchor_);
  const auto FALLBACK_EPOCH_COUNT =
      std::chrono::duration_cast<std::chrono::nanoseconds>(SYSTEM_TIME.time_since_epoch()).count();
  // Talos 等仿真源优先保留原始采集 epoch，使图像、TF 和真值与仿真快照严格同帧。
  const auto EPOCH_NANOS = stamp.capture_timestamp_ns.value_or(
      FALLBACK_EPOCH_COUNT > 0 ? static_cast<std::uint64_t>(FALLBACK_EPOCH_COUNT) : 0);
  const ::foxglove::schemas::Timestamp TIMESTAMP{
      .sec = static_cast<std::uint32_t>(EPOCH_NANOS / K_NANOSECONDS_PER_SECOND),
      .nsec = static_cast<std::uint32_t>(EPOCH_NANOS % K_NANOSECONDS_PER_SECOND)};

  PreparedFrame result;
  result.epoch_nanos = EPOCH_NANOS;
  if (demand.image) {
    auto encoded =
        image::EncodeJpeg(packet.capture.image, config_.jpeg_quality, config_.frame_id, TIMESTAMP);
    result.jpeg_ms = encoded.jpeg_ms;
    result.image = std::move(encoded.message);
  }
  if (demand.armor_annotations) {
    result.armor_annotations =
        armor_detector::EncodeAnnotations(frame.output.detections, TIMESTAMP);
  }
  if (demand.armor_stats) {
    result.armor_stats_json =
        armor_detector::EncodeDetectorStats(frame.diagnostics.detector, stamp.sequence, TIMESTAMP);
  }
  if (demand.lightbar_annotations) {
    result.lightbar_annotations = armor_light_detector::EncodeAnnotations(
        frame.output.lightbars, frame.diagnostics.prediction, TIMESTAMP);
  }
  if (demand.lightbar_stats) {
    result.lightbar_stats_json = armor_light_detector::EncodeStats(
        frame.diagnostics.lightbars, frame.diagnostics.prediction, stamp.sequence, TIMESTAMP);
  }
  // 实机没有控制运行时，因此在视觉链发布同帧 TF。Talos 仍由控制链独占发布，
  // 防止同一个 child 被图像时刻和控制预测时刻交替覆盖。无有效 IMU 时不伪造 TF。
  if (demand.real_transforms && packet.kinematics && !packet.simulation)
    result.real_transforms = spatial::EncodeTransforms(*packet.kinematics, TIMESTAMP);
  if (packet.camera_model && demand.calibration)
    result.calibration = spatial::EncodeCalibration(*packet.camera_model, TIMESTAMP);
  if (packet.camera_model && demand.frustum)
    result.frustum = spatial::EncodeFrustum(*packet.camera_model, TIMESTAMP);
  if (packet.simulation) {
    if (packet.simulation->combat) {
      if (demand.referee_state)
        result.referee_state_json =
            simulation::EncodeCombat(*packet.simulation->combat, stamp.sequence, TIMESTAMP, true);
      if (demand.combat_evaluation)
        result.combat_evaluation_json =
            simulation::EncodeCombat(*packet.simulation->combat, stamp.sequence, TIMESTAMP, false);
    }
    if (demand.ground_truth)
      result.ground_truth = simulation::EncodeGroundTruth(*packet.simulation, TIMESTAMP);
    if (demand.projectile_stats && packet.simulation->projectile_statistics) {
      result.projectile_stats_json = simulation::EncodeProjectileStats(
          *packet.simulation->projectile_statistics, stamp.sequence, TIMESTAMP);
    }
    if (demand.prediction_truth_overlay && prediction_evaluation)
      result.prediction_truth_overlay = prediction::EncodeTruthOverlay(
          frame.output.prediction, *prediction_evaluation, TIMESTAMP);
  }
  if (packet.simulation && packet.camera_model && packet.kinematics &&
      demand.projection_annotations) {
    result.projection_annotations = simulation::EncodeProjectionAnnotations(
        *packet.simulation, *packet.camera_model, *packet.kinematics, TIMESTAMP);
  }
  if (packet.kinematics && demand.pnp_estimates)
    result.pnp_estimates = pnp::EncodeEstimates(PNP_DIAGNOSTIC, *packet.kinematics, TIMESTAMP);
  if (demand.prediction_current_annotations) {
    result.prediction_current_annotations =
        prediction::EncodeCurrentAnnotations(frame.output.prediction, frame.diagnostics.prediction,
                                             SPATIAL_VIEW ? &*SPATIAL_VIEW : nullptr, TIMESTAMP);
  }
  if (demand.impact_annotations) {
    result.impact_annotations =
        SPATIAL_VIEW && impact ? prediction::EncodeImpactAnnotations(
                                     frame.output.prediction, *SPATIAL_VIEW, *impact, TIMESTAMP)
                               : prediction::EncodeEmptyAnnotations(TIMESTAMP);
  }
  if (demand.selected_armor_annotations) {
    result.selected_armor_annotations =
        SPATIAL_VIEW && frame.armor_selection
            ? prediction::EncodeSelectedArmorAnnotations(frame.output.prediction, *SPATIAL_VIEW,
                                                         *frame.armor_selection, TIMESTAMP)
            : prediction::EncodeEmptyAnnotations(TIMESTAMP);
  }
  if (demand.pnp_raw_corners) {
    result.pnp_raw_corners = pnp::EncodeRawCorners(PNP_DIAGNOSTIC, TIMESTAMP);
  }
  if (demand.pnp_final_corners) {
    result.pnp_final_corners = pnp::EncodeFinalCorners(PNP_DIAGNOSTIC, TIMESTAMP);
  }
  if (demand.pnp_reprojection) {
    result.pnp_reprojection = pnp::EncodeReprojection(PNP_DIAGNOSTIC, TIMESTAMP);
  }
  if (demand.pnp_error_vectors) {
    result.pnp_error_vectors = pnp::EncodeErrorVectors(PNP_DIAGNOSTIC, TIMESTAMP);
  }
  if (demand.corner_refiner_axes) {
    result.corner_refiner_axes = pnp::EncodeCornerRefinerAxes(PNP_DIAGNOSTIC, TIMESTAMP);
  }
  if (demand.corner_refiner_candidates) {
    result.corner_refiner_candidates =
        pnp::EncodeCornerRefinerCandidates(PNP_DIAGNOSTIC, TIMESTAMP);
  }
  if (demand.pnp_stats) {
    const char* const PNP_STATE = frame.output.detections.empty()      ? "not_attempted"
                                  : !SPATIAL_VIEW                      ? "unavailable"
                                  : frame.output.pnp.estimates.empty() ? "failed"
                                                                       : "solved";
    result.pnp_stats_json = pnp::EncodeStats(PNP_DIAGNOSTIC, PNP_STATE, stamp.sequence, TIMESTAMP);
  }
  if (demand.prediction_scene) {
    result.prediction_scene = prediction::EncodeScene(frame.output.prediction, TIMESTAMP);
  }
  if (demand.impact_scene) {
    result.impact_scene = prediction::EncodeImpactScene(frame.output.prediction,
                                                        impact ? &*impact : nullptr, TIMESTAMP);
  }
  if (demand.prediction_state) {
    result.prediction_state_json = prediction::EncodeState(
        frame.output.prediction, frame.diagnostics.prediction, prediction_evaluation, TIMESTAMP);
  }

  result.publish_latency_ms = Milliseconds(SteadyClock::now() - stamp.receive_steady_time);
  if (demand.debug_stats) {
    result.debug_stats_json =
        EncodeDebugStats(frame, TIMESTAMP, result.jpeg_ms, result.publish_latency_ms, counts);
  }
  return result;
}

}  // namespace mv::tool::foxglove::pipeline
