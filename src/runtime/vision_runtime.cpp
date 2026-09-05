#include "runtime/vision_runtime.hpp"

#include "core/logger.hpp"
#include "hal/camera/i_camera.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"
#include "runtime/control_runtime.hpp"
#include "runtime/runtime_diagnostics_sink.hpp"
#include "runtime/vision_pipeline.hpp"
#include "runtime/vision_tuning.hpp"
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
                             tool::simulation_evaluation::SimulationEvaluator* evaluator,
                             RuntimeSupervisor& supervisor,
                             VisionTuningMailbox* tuning_mailbox) noexcept
    : camera_(camera),
      pipeline_(pipeline),
      control_(control),
      window_(window),
      diagnostics_(diagnostics),
      evaluator_(evaluator),
      supervisor_(supervisor),
      tuning_mailbox_(tuning_mailbox) {}

RuntimeRunResult VisionRuntime::Run(const std::function<bool()>& stop_requested) {
  const auto STOP_WITH_TERMINAL = [this](RuntimeTerminationReason fallback) {
    if (control_)
      control_->Stop();
    return supervisor_.TerminalResult().value_or(
        RuntimeRunResult{.reason = fallback, .fault = std::nullopt});
  };
  supervisor_.Recover(RuntimeComponent::CAMERA);
  while (!stop_requested()) {
    bool window_operation_failed = false;
    if (const auto TERMINAL = supervisor_.TerminalResult()) {
      if (control_)
        control_->Stop();
      return *TERMINAL;
    }
    frame::FramePacket packet;
    hal::GrabStatus status{hal::GrabStatus::FATAL};
    try {
      status = camera_.Grab(packet);
    } catch (const std::exception& error) {
      static_cast<void>(supervisor_.Report(RuntimeFaultCode::CAMERA_FATAL, error.what()));
      MV_LOG_ERROR("App", "camera grab failed: fatal");
      return STOP_WITH_TERMINAL(RuntimeTerminationReason::CAMERA_FAILURE);
    } catch (...) {
      static_cast<void>(
          supervisor_.Report(RuntimeFaultCode::CAMERA_FATAL, "unknown camera exception"));
      MV_LOG_ERROR("App", "camera grab failed: fatal");
      return STOP_WITH_TERMINAL(RuntimeTerminationReason::CAMERA_FAILURE);
    }

    if (status == hal::GrabStatus::OK) {
      supervisor_.Recover(RuntimeComponent::CAMERA);
      if (tuning_mailbox_) {
        if (const auto TUNING = tuning_mailbox_->TryTakePending()) {
          pipeline_.ApplyFrontendTuning(TUNING->config);
          tuning_mailbox_->MarkApplied(
              {.request_id = TUNING->request_id, .source_sequence = packet.capture.stamp.sequence});
          MV_LOG_INFO("WebDebug", "frontend tuning revision={} applied at sequence={}",
                      TUNING->target_revision, packet.capture.stamp.sequence);
        }
      }
      VisionFrameResult result;
      try {
        const auto SPATIAL = frame::MakeSpatialFrameView(packet);
        result = pipeline_.Process({.capture = packet.capture, .spatial = SPATIAL});
      } catch (const std::exception& error) {
        static_cast<void>(
            supervisor_.Report(RuntimeFaultCode::VISION_PIPELINE_EXCEPTION, error.what()));
        MV_LOG_ERROR("App", "armor detection failed: {}", error.what());
        return STOP_WITH_TERMINAL(RuntimeTerminationReason::PIPELINE_FAILURE);
      } catch (...) {
        static_cast<void>(supervisor_.Report(RuntimeFaultCode::VISION_PIPELINE_EXCEPTION,
                                             "unknown vision pipeline exception"));
        MV_LOG_ERROR("App", "armor detection failed: unknown exception");
        return STOP_WITH_TERMINAL(RuntimeTerminationReason::PIPELINE_FAILURE);
      }

      if (control_ && packet.camera_model && packet.kinematics) {
        try {
          control_->Update(result.output.prediction, *packet.kinematics, packet.chassis_motion,
                           packet.gimbal_actuator);
        } catch (const std::exception& error) {
          static_cast<void>(
              supervisor_.Report(RuntimeFaultCode::CONTROL_UPDATE_EXCEPTION, error.what()));
          return STOP_WITH_TERMINAL(RuntimeTerminationReason::CONTROL_FAILURE);
        } catch (...) {
          static_cast<void>(supervisor_.Report(RuntimeFaultCode::CONTROL_UPDATE_EXCEPTION,
                                               "unknown control snapshot exception"));
          return STOP_WITH_TERMINAL(RuntimeTerminationReason::CONTROL_FAILURE);
        }
      }

      std::optional<tool::simulation_evaluation::SimulationEvaluationResult> evaluation;
      if (evaluator_ && packet.camera_model && packet.kinematics && packet.simulation) {
        try {
          evaluation =
              evaluator_->Evaluate({.stamp = packet.capture.stamp,
                                    .camera_model = *packet.camera_model,
                                    .kinematics = *packet.kinematics,
                                    .simulation = *packet.simulation,
                                    .detections = result.output.detections,
                                    .refinements = result.output.refinements,
                                    .refinement_diagnostics = result.diagnostics.refinements,
                                    .pnp = result.output.pnp,
                                    .pnp_diagnostics = result.diagnostics.pnp,
                                    .prediction = result.output.prediction});
          supervisor_.Recover(RuntimeComponent::SIMULATION_EVALUATION);
        } catch (const std::exception& error) {
          const auto DECISION =
              supervisor_.Report(RuntimeFaultCode::EVALUATION_EXCEPTION, error.what());
          if (DECISION.disable_component)
            evaluator_ = nullptr;
        } catch (...) {
          const auto DECISION = supervisor_.Report(RuntimeFaultCode::EVALUATION_EXCEPTION,
                                                   "unknown evaluation exception");
          if (DECISION.disable_component)
            evaluator_ = nullptr;
        }
      }
      if (evaluation) {
        try {
          LogPnpHealth(evaluation->pnp, packet.capture.stamp.sequence,
                       packet.simulation->armors.size());
        } catch (const std::exception& error) {
          static_cast<void>(
              supervisor_.Report(RuntimeFaultCode::DIAGNOSTICS_EXCEPTION, error.what()));
        } catch (...) {
          static_cast<void>(supervisor_.Report(RuntimeFaultCode::DIAGNOSTICS_EXCEPTION,
                                               "unknown PnP health logging exception"));
        }
      }

      if (diagnostics_) {
        diagnostics_->PublishVision(packet, result.output, result.diagnostics, evaluation);
        const auto HEALTH = diagnostics_->SnapshotHealth();
        if (HEALTH.available) {
          supervisor_.Recover(RuntimeComponent::DIAGNOSTICS);
        } else {
          static_cast<void>(supervisor_.Report(RuntimeFaultCode::DIAGNOSTICS_UNAVAILABLE,
                                               "all configured diagnostics sinks are unavailable"));
        }
      }

      if (window_) {
        try {
          cv::Mat debug_image = packet.capture.image.clone();
          DrawDetections(debug_image, result.output.detections, result.diagnostics.detector);
          window_->Show(debug_image);
        } catch (const std::exception& error) {
          window_operation_failed = true;
          const auto DECISION =
              supervisor_.Report(RuntimeFaultCode::DEBUG_WINDOW_EXCEPTION, error.what());
          if (DECISION.disable_component) {
            window_->Close();
            window_ = nullptr;
          }
        } catch (...) {
          window_operation_failed = true;
          const auto DECISION = supervisor_.Report(RuntimeFaultCode::DEBUG_WINDOW_EXCEPTION,
                                                   "unknown debug window exception");
          if (DECISION.disable_component) {
            window_->Close();
            window_ = nullptr;
          }
        }
      }
    } else {
      RuntimeFaultCode code{RuntimeFaultCode::CAMERA_TIMEOUT};
      if (status == hal::GrabStatus::INVALID_FRAME) {
        code = RuntimeFaultCode::CAMERA_INVALID_FRAME;
      } else if (status == hal::GrabStatus::DISCONNECTED) {
        code = RuntimeFaultCode::CAMERA_DISCONNECTED;
      } else if (status == hal::GrabStatus::FATAL) {
        code = RuntimeFaultCode::CAMERA_FATAL;
      }
      const auto DECISION = supervisor_.Report(code, hal::GrabStatusName(status));
      if (DECISION.termination) {
        MV_LOG_ERROR("App", "camera grab failed: {}", hal::GrabStatusName(status));
        return STOP_WITH_TERMINAL(RuntimeTerminationReason::CAMERA_FAILURE);
      }
    }

    if (window_) {
      try {
        if (window_->Poll().exit_requested)
          return {};
        if (!window_operation_failed)
          supervisor_.Recover(RuntimeComponent::DEBUG_WINDOW);
      } catch (const std::exception& error) {
        const auto DECISION =
            supervisor_.Report(RuntimeFaultCode::DEBUG_WINDOW_EXCEPTION, error.what());
        if (DECISION.disable_component) {
          window_->Close();
          window_ = nullptr;
        }
      } catch (...) {
        const auto DECISION = supervisor_.Report(RuntimeFaultCode::DEBUG_WINDOW_EXCEPTION,
                                                 "unknown debug window exception");
        if (DECISION.disable_component) {
          window_->Close();
          window_ = nullptr;
        }
      }
    }
  }
  MV_LOG_INFO("App", "stop signal received");
  return {};
}

}  // namespace mv::runtime
