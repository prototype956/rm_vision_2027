#include "runtime/vision_runtime.hpp"

#include "core/logger.hpp"
#include "hal/camera/i_camera.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"
#include "runtime/control_runtime.hpp"
#include "runtime/runtime_diagnostics_sink.hpp"
#include "runtime/vision_pipeline.hpp"
#include "tool/debug/armor_detection_overlay.hpp"
#include "tool/debug/debug_window.hpp"
#include "tool/simulation_evaluation/simulation_evaluator.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

namespace mv::runtime {
namespace {

void DrawDetections(cv::Mat& image, const std::vector<modules::ArmorDetection>& detections,
                    const modules::ArmorDetectorDiagnostics& stats) {
  tool::DrawArmorDetections(image, detections);

  const auto SUMMARY = fmt::format("detections={} candidates={} total={:.2f} ms", detections.size(),
                                   stats.threshold_candidates, stats.total_ms);
  cv::putText(image, SUMMARY, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2,
              cv::LINE_AA);
}

void LogPnpHealth(const tool::simulation_evaluation::PnpEvaluationResult& result,
                  std::uint64_t sequence, std::size_t total_truth_armors) {
  if (sequence % 100 != 0)
    return;
  std::string dominant_refinement_failure = "none";
  std::size_t dominant_refinement_failure_count = 0;
  for (const auto& [reason, count] : result.refinement_summary.failure_reasons) {
    if (count > dominant_refinement_failure_count) {
      dominant_refinement_failure = reason;
      dominant_refinement_failure_count = count;
    }
  }
  std::size_t truth_attempted = 0;
  std::size_t truth_succeeded = 0;
  double max_rmse = 0.0;
  double max_position_error = 0.0;
  double max_rotation_error = 0.0;
  for (const auto& attempt : result.attempts) {
    if (attempt.source == tool::simulation_evaluation::PnpEvaluationSource::GROUND_TRUTH) {
      ++truth_attempted;
      if (attempt.estimate) {
        ++truth_succeeded;
        max_rmse = std::max(max_rmse, attempt.estimate->reprojection_rmse_px);
        max_position_error =
            std::max(max_position_error, attempt.estimate->position_error_m.value_or(0.0));
        max_rotation_error =
            std::max(max_rotation_error, attempt.estimate->rotation_error_deg.value_or(0.0));
      }
    }
  }
  MV_LOG_INFO(
      "ArmorPnP",
      "truth baseline seq={} visible_solved={}/{} total={} max_rmse={:.4f}px max_position={:.4f}m "
      "max_rotation={:.3f}deg",
      sequence, truth_succeeded, truth_attempted, total_truth_armors, max_rmse, max_position_error,
      max_rotation_error);
  MV_LOG_INFO("ArmorPnP",
              "single-chain seq={} solved={}/{} refine={}/{} fallback={} "
              "corner_p95(raw/final)={:.3f}/{:.3f}px depth_p95={:.4f}m "
              "top_fallback={}({})",
              sequence, result.solve_summary.succeeded, result.solve_summary.attempted,
              result.refinement_summary.succeeded, result.refinement_summary.attempted,
              result.refinement_summary.fallback,
              result.refinement_summary.raw_mean_corner_error_px.p95,
              result.refinement_summary.final_mean_corner_error_px.p95,
              result.detection_summary.depth_error_m.p95, dominant_refinement_failure,
              dominant_refinement_failure_count);
  const auto MATCHED_DETECTION =
      std::find_if(result.attempts.begin(), result.attempts.end(), [](const auto& attempt) {
        return attempt.source == tool::simulation_evaluation::PnpEvaluationSource::DETECTION &&
               attempt.estimate && attempt.estimate->truth_id;
      });
  if (MATCHED_DETECTION != result.attempts.end()) {
    const auto& value = *MATCHED_DETECTION->estimate;
    MV_LOG_INFO("ArmorPnP",
                "matched final truth={} corner du=[{:.1f},{:.1f},{:.1f},{:.1f}] "
                "dv=[{:.1f},{:.1f},{:.1f},{:.1f}]",
                *value.truth_id, value.corner_delta_u_px[0], value.corner_delta_u_px[1],
                value.corner_delta_u_px[2], value.corner_delta_u_px[3], value.corner_delta_v_px[0],
                value.corner_delta_v_px[1], value.corner_delta_v_px[2], value.corner_delta_v_px[3]);
  }
}

}  // namespace

VisionRuntime::VisionRuntime(hal::ICamera& camera, VisionPipeline& pipeline,
                             ControlRuntime* control, tool::DebugWindow* window,
                             IRuntimeDiagnosticsSink* diagnostics,
                             tool::simulation_evaluation::SimulationEvaluator* evaluator) noexcept
    : camera_(camera),
      pipeline_(pipeline),
      control_(control),
      window_(window),
      diagnostics_(diagnostics),
      evaluator_(evaluator) {}

VisionRunStatus VisionRuntime::Run(const std::function<bool()>& stop_requested) {
  while (!stop_requested()) {
    if (control_ && control_->Failed()) {
      MV_LOG_ERROR("App", "control thread failed; stopping vision pipeline safely");
      return VisionRunStatus::CONTROL_FAILURE;
    }
    frame::FramePacket packet;
    const auto STATUS = camera_.Grab(packet);

    if (STATUS == hal::GrabStatus::OK) {
      try {
        const auto SPATIAL = frame::MakeSpatialFrameView(packet);
        const auto RESULT = pipeline_.Process({.capture = packet.capture, .spatial = SPATIAL});
        if (control_ && packet.camera_model && packet.kinematics)
          control_->Update(RESULT.output.prediction, *packet.kinematics, packet.gimbal_actuator);
        std::optional<tool::simulation_evaluation::SimulationEvaluationResult> evaluation;
        if (evaluator_ && packet.camera_model && packet.kinematics && packet.simulation) {
          try {
            evaluation =
                evaluator_->Evaluate({.stamp = packet.capture.stamp,
                                      .camera_model = *packet.camera_model,
                                      .kinematics = *packet.kinematics,
                                      .simulation = *packet.simulation,
                                      .detections = RESULT.output.detections,
                                      .refinements = RESULT.output.refinements,
                                      .refinement_diagnostics = RESULT.diagnostics.refinements,
                                      .pnp = RESULT.output.pnp,
                                      .pnp_diagnostics = RESULT.diagnostics.pnp,
                                      .prediction = RESULT.output.prediction});
          } catch (const std::exception& error) {
            MV_LOG_WARN("SimulationEvaluation", "frame evaluation skipped: {}", error.what());
          }
        }
        if (evaluation)
          LogPnpHealth(evaluation->pnp, packet.capture.stamp.sequence,
                       packet.simulation->armors.size());
        if (diagnostics_) {
          diagnostics_->PublishVision(packet, RESULT.output, RESULT.diagnostics, evaluation);
        }
        if (window_) {
          cv::Mat debug_image = packet.capture.image.clone();
          DrawDetections(debug_image, RESULT.output.detections, RESULT.diagnostics.detector);
          window_->Show(debug_image);
        }
      } catch (const std::exception& error) {
        MV_LOG_ERROR("App", "armor detection failed: {}", error.what());
        return VisionRunStatus::PIPELINE_FAILURE;
      }
    } else if (STATUS == hal::GrabStatus::DISCONNECTED || STATUS == hal::GrabStatus::FATAL) {
      MV_LOG_ERROR("App", "camera grab failed: {}", hal::GrabStatusName(STATUS));
      return VisionRunStatus::CAMERA_FAILURE;
    }

    if (window_ && window_->Poll().exit_requested)
      return VisionRunStatus::NORMAL;
  }
  MV_LOG_INFO("App", "stop signal received");
  return VisionRunStatus::NORMAL;
}

}  // namespace mv::runtime
