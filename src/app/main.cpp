#include "app/main.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/camera_factory.hpp"
#include "hal/gimbal/i_gimbal_command_sink.hpp"
#include "hal/gimbal/talos/talos_gimbal_command_sink.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_detector/armor_detector_config.hpp"
#include "modules/armor_pnp/armor_pnp.hpp"
#include "modules/armor_predictor/armor_predictor.hpp"
#include "modules/fire_control/fire_control.hpp"
#include "modules/fire_control/fire_control_config.hpp"
#include "modules/fire_control/gimbal_feedback_estimator.hpp"
#include "modules/gimbal_trajectory_planner/gimbal_trajectory_planner_config.hpp"
#include "tool/debug/armor_detection_overlay.hpp"
#include "tool/debug/debug_window.hpp"
#include "tool/foxglove/foxglove_config.hpp"
#include "tool/foxglove/vision_debug_publisher.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <filesystem>
#include <fmt/format.h>
#include <numbers>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace mv::app {
namespace {

constexpr char K_WINDOW_NAME[] = "MiracleVision Camera Preview";
volatile std::sig_atomic_t g_stop_requested = 0;

std::uint64_t SystemNowNs() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}

void HandleStopSignal(int) noexcept {
  g_stop_requested = 1;
}

bool LoadDebugWindowEnabled(const std::filesystem::path& config_path) {
  constexpr char CONTEXT[] = "debug window config";
  const auto ROOT = ConfigLoader::LoadFile(config_path);
  ConfigLoader::RejectUnknownKeys(ROOT, {"schema_version", "enabled"}, CONTEXT);
  if (ConfigLoader::Require<int>(ROOT, "schema_version", CONTEXT) != 1) {
    throw ConfigError("debug window config schema_version must be 1");
  }
  return ConfigLoader::Require<bool>(ROOT, "enabled", CONTEXT);
}

struct CameraSelection {
  std::string backend;                ///< 传给相机工厂的后端名称。
  std::filesystem::path config_path;  ///< 相对于配置根目录解析后的后端配置路径。
};

/** 读取主程序的相机选择，并在创建后端前完成名称和配置键校验。 */
CameraSelection LoadCameraSelection(const std::filesystem::path& config_root) {
  constexpr char CONTEXT[] = "main app config";
  const auto ROOT = ConfigLoader::LoadFile(config_root / "app/main.yaml");
  ConfigLoader::RejectUnknownKeys(ROOT, {"schema_version", "camera"}, CONTEXT);

  const auto CAMERA = ROOT["camera"];
  ConfigLoader::RequireMap(CAMERA, "main app config.camera");
  ConfigLoader::RejectUnknownKeys(CAMERA, {"backend", "configs"}, "main app config.camera");
  const auto BACKEND =
      ConfigLoader::Require<std::string>(CAMERA, "backend", "main app config.camera");

  const auto CONFIGS = CAMERA["configs"];
  ConfigLoader::RequireMap(CONFIGS, "main app config.camera.configs");
  ConfigLoader::RejectUnknownKeys(CONFIGS, {"mindvision", "talos"},
                                  "main app config.camera.configs");
  if (BACKEND != "mindvision" && BACKEND != "talos") {
    throw ConfigError("main app config.camera.backend must be mindvision or talos");
  }
  const auto CONFIG_FILE =
      ConfigLoader::Require<std::string>(CONFIGS, BACKEND, "main app config.camera.configs");
  return CameraSelection{BACKEND, ConfigLoader::ResolvePath(config_root, CONFIG_FILE)};
}

/** Talos 专用的 100 Hz 控制运行时；视觉线程只替换不可变输入快照。 */
class ControlRuntime final {
 public:
  ControlRuntime(modules::FireControlConfig fire_config,
                 modules::GimbalTrajectoryPlannerConfig planner_config,
                 std::unique_ptr<hal::IGimbalCommandSink> sink,
                 tool::foxglove::VisionDebugPublisher* diagnostics)
      : period_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(planner_config.dt_s))),
        planner_dt_s_(planner_config.dt_s),
        fire_control_(std::move(fire_config), planner_config),
        feedback_estimator_(planner_config.max_yaw_velocity_rad_s,
                            planner_config.max_pitch_velocity_rad_s),
        sink_(std::move(sink)),
        diagnostics_(diagnostics) {
    if (!sink_)
      throw std::invalid_argument("control runtime requires a command sink");
  }

  ~ControlRuntime() { Stop(); }
  ControlRuntime(const ControlRuntime&) = delete;
  ControlRuntime& operator=(const ControlRuntime&) = delete;

  void Start() {
    if (running_.exchange(true, std::memory_order_acq_rel))
      return;
    try {
      thread_ = std::thread([this] { Loop(); });
    } catch (...) {
      running_.store(false, std::memory_order_release);
      SendStop();
      throw;
    }
  }

  void Update(const modules::ArmorPredictionResult& prediction,
              const hal::CameraFrame::FrameGeometry& geometry) {
    auto snapshot = std::make_shared<modules::ControlInputSnapshot>();
    snapshot->prediction = prediction;
    // 仿真真值不进入控制快照，只保留同帧云台与枪口外参。
    snapshot->world_t_gimbal = geometry.world_t_gimbal;
    snapshot->gimbal_t_muzzle = geometry.gimbal_t_muzzle;
    snapshot->frame_actuator = geometry.gimbal_actuator;
    std::atomic_store_explicit(&latest_snapshot_,
                               std::shared_ptr<const modules::ControlInputSnapshot>(snapshot),
                               std::memory_order_release);
  }

  void Stop() noexcept {
    running_.store(false, std::memory_order_release);
    if (thread_.joinable())
      thread_.join();
    SendStop();
  }

  [[nodiscard]] bool Failed() const noexcept { return failed_.load(std::memory_order_acquire); }

 private:
  modules::MatchedGimbalCommand MatchCommand(
      const std::optional<std::uint64_t>& capture_timestamp_ns,
      const std::optional<hal::GimbalActuatorTelemetry>& actuator) const noexcept {
    modules::MatchedGimbalCommand match;
    if (!capture_timestamp_ns)
      return match;
    if (actuator && actuator->valid && actuator->consumed_command_timestamp_ns != 0) {
      for (auto iterator = sent_commands_.rbegin(); iterator != sent_commands_.rend(); ++iterator) {
        if (iterator->timestamp_ns != actuator->consumed_command_timestamp_ns)
          continue;
        match.valid = iterator->valid;
        match.approximate = false;
        match.command = *iterator;
        match.age_at_capture_s =
            *capture_timestamp_ns >= iterator->timestamp_ns
                ? static_cast<double>(*capture_timestamp_ns - iterator->timestamp_ns) * 1.0e-9
                : 0.0;
        return match;
      }
    }
    for (auto iterator = sent_commands_.rbegin(); iterator != sent_commands_.rend(); ++iterator) {
      if (iterator->timestamp_ns > *capture_timestamp_ns)
        continue;
      match.valid = iterator->valid;
      match.approximate = true;
      match.command = *iterator;
      match.age_at_capture_s =
          static_cast<double>(*capture_timestamp_ns - iterator->timestamp_ns) * 1.0e-9;
      return match;
    }
    return match;
  }

  void RememberCommand(const hal::GimbalCommand& command) {
    sent_commands_.push_back(command);
    while (!sent_commands_.empty() &&
           command.timestamp_ns > sent_commands_.front().timestamp_ns + 1'000'000'000ULL) {
      sent_commands_.pop_front();
    }
  }

  void ClearPublishedProjection(std::string_view reason) noexcept {
    feedback_estimator_.ClearCommandProjection();
    fire_control_.ResetFireReadiness();
    last_successful_trajectory_.clear();
    control_projection_active_ = false;
    output_projection_cleared_pending_ = true;
    if (!output_projection_clear_reason_.empty())
      output_projection_clear_reason_.push_back(',');
    output_projection_clear_reason_.append(reason);
  }

  void AttachProjectionDiagnostics(modules::FireControlResult& result) {
    result.output_projection_cleared = output_projection_cleared_pending_;
    result.output_projection_clear_reason = std::move(output_projection_clear_reason_);
    output_projection_cleared_pending_ = false;
    output_projection_clear_reason_.clear();
  }

  void Loop() noexcept {
    std::uint64_t observed_sequence = ~std::uint64_t{0};
    std::optional<bool> last_external_control;
    std::optional<bool> last_sink_healthy;
    std::optional<hal::GimbalActuatorMode> last_actuator_mode;
    std::uint64_t control_cycles = 0;
    auto next = std::chrono::steady_clock::now();
    auto previous_cycle = next;
    modules::MatchedGimbalCommand matched_command;
    try {
      while (running_.load(std::memory_order_acquire)) {
        const auto now = std::chrono::steady_clock::now();
        const double control_period_s = std::chrono::duration<double>(now - previous_cycle).count();
        previous_cycle = now;
        const double deadline_lateness_us =
            now > next ? std::chrono::duration<double, std::micro>(now - next).count() : 0.0;
        auto snapshot = std::atomic_load_explicit(&latest_snapshot_, std::memory_order_acquire);
        if (snapshot) {
          const auto system_now_ns = SystemNowNs();
          const auto actuator = sink_->ActuatorTelemetry();
          const std::optional<hal::GimbalActuatorMode> actuator_mode =
              actuator.valid ? std::optional(actuator.mode) : std::nullopt;
          if (actuator_mode != last_actuator_mode) {
            feedback_estimator_.ClearRuntimeActuator();
            ClearPublishedProjection("actuator_mode_changed");
            last_actuator_mode = actuator_mode;
          }

          auto input = *snapshot;
          input.external_control_enabled = sink_->ExternalControlEnabled();
          if (!last_external_control || *last_external_control != input.external_control_enabled) {
            if (input.external_control_enabled) {
              MV_LOG_INFO("Control", "Talos external auto-aim subscription enabled");
            } else {
              MV_LOG_WARN(
                  "Control",
                  "Talos external auto-aim subscription is disabled; press F5 in simulation");
            }
            feedback_estimator_.ClearRuntimeActuator();
            ClearPublishedProjection(input.external_control_enabled ? "external_control_enabled"
                                                                    : "external_control_disabled");
            last_external_control = input.external_control_enabled;
          }

          const bool runtime_was_active = feedback_estimator_.RuntimeActuatorActive();
          feedback_estimator_.ObserveActuatorTelemetry(actuator, now, system_now_ns);
          if (runtime_was_active != feedback_estimator_.RuntimeActuatorActive()) {
            ClearPublishedProjection(feedback_estimator_.RuntimeActuatorActive()
                                         ? "runtime_actuator_enabled"
                                         : "runtime_actuator_invalid");
          }

          bool measurement_fresh = false;
          if (snapshot->prediction.sequence != observed_sequence) {
            feedback_estimator_.ObserveMeasurement(
                snapshot->prediction.sequence, snapshot->prediction.source_receive_steady_time,
                snapshot->world_t_gimbal, snapshot->frame_actuator);
            observed_sequence = snapshot->prediction.sequence;
            measurement_fresh = true;
            matched_command = MatchCommand(snapshot->prediction.source_capture_timestamp_ns,
                                           snapshot->frame_actuator);
          }
          const bool sink_healthy = sink_->IsHealthy();
          if (!last_sink_healthy || *last_sink_healthy != sink_healthy) {
            if (!sink_healthy)
              ClearPublishedProjection("talos_unhealthy");
            last_sink_healthy = sink_healthy;
          }
          const auto feedback = feedback_estimator_.Estimate(now);
          const auto feedback_source = feedback_estimator_.Source();
          auto result = fire_control_.Step(input, feedback, now);
          if (result.tracking_object_reset && control_projection_active_) {
            feedback_estimator_.ClearRuntimeActuator();
            ClearPublishedProjection("tracking_object_changed");
          }
          result.feedback_source = feedback_source;
          result.measured_feedback = feedback_estimator_.LastMeasurement();
          result.measurement_fresh = measurement_fresh;
          result.measurement_age_s =
              result.measured_feedback.valid
                  ? std::max(0.0,
                             std::chrono::duration<double>(now - result.measured_feedback.timestamp)
                                 .count())
                  : std::numeric_limits<double>::infinity();
          result.matched_prior_command = matched_command;
          result.actuator_telemetry = actuator;
          result.frame_actuator_telemetry = snapshot->frame_actuator;
          result.runtime_actuator_age_s = feedback_estimator_.RuntimeActuatorAgeS();
          result.feedback_projection_dt_s = feedback_estimator_.ProjectionDtS();
          result.feedback_runtime_state_timestamp_ns =
              feedback_estimator_.RuntimeStateTimestampNs();
          if (snapshot->frame_actuator && snapshot->frame_actuator->valid &&
              snapshot->frame_actuator->state_timestamp_ns != 0 &&
              snapshot->frame_actuator->state_timestamp_ns <= system_now_ns) {
            result.frame_actuator_age_s =
                static_cast<double>(system_now_ns - snapshot->frame_actuator->state_timestamp_ns) *
                1.0e-9;
          } else {
            result.frame_actuator_age_s = std::numeric_limits<double>::infinity();
          }
          result.feedback_runtime_comparison_valid =
              feedback.valid && actuator.valid &&
              actuator.mode == hal::GimbalActuatorMode::PHYSICAL;
          if (result.feedback_runtime_comparison_valid) {
            result.yaw_feedback_minus_runtime_actuator =
                std::remainder(feedback.yaw - actuator.actual_yaw, 2.0 * std::numbers::pi);
            result.pitch_feedback_minus_runtime_actuator = feedback.pitch - actuator.actual_pitch;
          }
          result.frame_runtime_comparison_valid =
              snapshot->frame_actuator && snapshot->frame_actuator->valid && actuator.valid &&
              snapshot->frame_actuator->mode == hal::GimbalActuatorMode::PHYSICAL &&
              actuator.mode == hal::GimbalActuatorMode::PHYSICAL;
          if (result.frame_runtime_comparison_valid) {
            const auto& frame = *snapshot->frame_actuator;
            result.yaw_frame_minus_runtime_actuator =
                std::remainder(frame.actual_yaw - actuator.actual_yaw, 2.0 * std::numbers::pi);
            result.pitch_frame_minus_runtime_actuator = frame.actual_pitch - actuator.actual_pitch;
            result.yaw_frame_acceleration_minus_runtime =
                frame.yaw_acceleration - actuator.yaw_acceleration;
            result.pitch_frame_acceleration_minus_runtime =
                frame.pitch_acceleration - actuator.pitch_acceleration;
          }
          result.control_period_s = control_period_s;
          result.deadline_lateness_us = deadline_lateness_us;
          result.command_sink_healthy = sink_healthy;
          result.talos_heartbeat_ns = sink_->HeartbeatTimestampNs();

          if (result.reject_reason == modules::FireRejectReason::MPC_FAILED) {
            ++consecutive_mpc_failure_cycles_;
          } else {
            consecutive_mpc_failure_cycles_ = 0;
          }
          result.consecutive_mpc_failure_cycles = consecutive_mpc_failure_cycles_;

          if (result.reject_reason == modules::FireRejectReason::MPC_FAILED &&
              !last_successful_trajectory_.empty() && result.external_control_enabled &&
              result.command_sink_healthy && !result.tracking_object_reset &&
              result.selected_slot == last_successful_plan_slot_) {
            const double fallback_age_s = std::max(
                0.0, std::chrono::duration<double>(now - last_successful_plan_time_).count());
            result.fallback_age_s = fallback_age_s;
            result.fallback_source_slot = last_successful_plan_slot_;
            constexpr double MAX_FALLBACK_AGE_S = 0.100;
            const auto elapsed_steps = static_cast<std::size_t>(
                std::max(1LL, std::llround(fallback_age_s / planner_dt_s_)));
            const auto fallback_index = last_successful_command_index_ + elapsed_steps;
            if (fallback_age_s <= MAX_FALLBACK_AGE_S &&
                fallback_index < last_successful_trajectory_.size()) {
              const auto& point = last_successful_trajectory_[fallback_index];
              result.command = {.valid = true,
                                .fire = false,
                                .timestamp_ns = result.command_timestamp_ns,
                                .yaw = std::remainder(point.yaw, 2.0 * std::numbers::pi),
                                .yaw_velocity = point.yaw_velocity,
                                .yaw_acceleration = point.yaw_acceleration,
                                .pitch = point.pitch,
                                .pitch_velocity = point.pitch_velocity,
                                .pitch_acceleration = point.pitch_acceleration,
                                .target_distance_m = last_successful_target_distance_m_};
              result.command.valid = true;
              result.command.fire = false;
              result.command_source = modules::GimbalCommandSource::TRAJECTORY_FALLBACK;
              result.fallback_active = true;
              result.fallback_trajectory_index = static_cast<int>(fallback_index);
              result.fallback_remaining_points =
                  static_cast<int>(last_successful_trajectory_.size() - fallback_index - 1);
            }
          }

          if (!result.command_sink_healthy || !result.external_control_enabled) {
            result.command.valid = false;
            result.command.fire = false;
            result.command_source = modules::GimbalCommandSource::STOP;
            result.reject_reason = modules::FireRejectReason::TALOS_UNHEALTHY;
            if (!result.external_control_enabled)
              result.reject_reason = modules::FireRejectReason::EXTERNAL_CONTROL_DISABLED;
          }

          if (!result.command.valid) {
            result.command_source = modules::GimbalCommandSource::STOP;
            result.command.fire = false;
            if (control_projection_active_) {
              if (result.reject_reason == modules::FireRejectReason::MPC_FAILED) {
                result.fallback_expired_this_cycle = true;
                ClearPublishedProjection("mpc_fallback_expired");
              } else {
                ClearPublishedProjection(modules::FireRejectReasonName(result.reject_reason));
              }
            }
          }
          const auto send_start = std::chrono::steady_clock::now();
          const bool send_succeeded = sink_->Send(result.command);
          result.sink_send_time_us = std::chrono::duration<double, std::micro>(
                                         std::chrono::steady_clock::now() - send_start)
                                         .count();
          result.command_publish_succeeded = send_succeeded && result.command.valid;
          result.published_valid = result.command_publish_succeeded;
          if (!send_succeeded) {
            result.command_sink_healthy = false;
            result.command.valid = false;
            result.command.fire = false;
            result.reject_reason = modules::FireRejectReason::TALOS_UNHEALTHY;
            result.command_source = modules::GimbalCommandSource::STOP;
            result.published_valid = false;
            ClearPublishedProjection("command_send_failed");
          }
          if (send_succeeded)
            RememberCommand(result.command);
          if (result.command_publish_succeeded) {
            feedback_estimator_.ObservePublishedCommand(result.command, now, false);
            control_projection_active_ = true;
            if (result.command_source == modules::GimbalCommandSource::MPC) {
              last_successful_trajectory_ = result.plan.trajectory;
              last_successful_command_index_ = static_cast<std::size_t>(result.plan.command_index);
              last_successful_target_distance_m_ = result.command.target_distance_m;
              last_successful_plan_time_ = now;
              last_successful_plan_slot_ = result.selected_slot;
            }
          }
          AttachProjectionDiagnostics(result);
          if (++control_cycles % 100 == 0) {
            MV_LOG_INFO("Control",
                        "seq={} tracker={} slot={} command={} fire={} reject={} "
                        "age(pred/fb)={:.1f}/{:.1f}ms mpc={} iter={}/{} solve={:.1f}us",
                        result.source_sequence, modules::TrackerStateName(result.tracker_state),
                        result.selected_slot, result.command.valid, result.command.fire,
                        modules::FireRejectReasonName(result.reject_reason),
                        result.prediction_age_s * 1.0e3, result.feedback_age_s * 1.0e3,
                        result.plan.valid, result.plan.yaw_iterations, result.plan.pitch_iterations,
                        result.plan.solve_time_us);
          }
          if (diagnostics_)
            diagnostics_->PublishControl(result);
        } else {
          SendStop();
        }

        next += period_;
        const auto finished = std::chrono::steady_clock::now();
        if (finished >= next + period_)
          next = finished + period_;
        std::this_thread::sleep_until(next);
      }
    } catch (const std::exception& error) {
      failed_.store(true, std::memory_order_release);
      running_.store(false, std::memory_order_release);
      SendStop();
      MV_LOG_ERROR("Control", "100 Hz control thread stopped after exception: {}", error.what());
    } catch (...) {
      failed_.store(true, std::memory_order_release);
      running_.store(false, std::memory_order_release);
      SendStop();
      MV_LOG_ERROR("Control", "100 Hz control thread stopped after unknown exception");
    }
  }

  void SendStop() noexcept {
    if (!sink_)
      return;
    hal::GimbalCommand stop;
    stop.timestamp_ns =
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count());
    sink_->Send(stop);
    feedback_estimator_.ClearCommandProjection();
    last_successful_trajectory_.clear();
    control_projection_active_ = false;
  }

  const std::chrono::steady_clock::duration period_;
  const double planner_dt_s_;
  modules::FireControl fire_control_;
  modules::GimbalFeedbackEstimator feedback_estimator_;
  std::unique_ptr<hal::IGimbalCommandSink> sink_;
  tool::foxglove::VisionDebugPublisher* diagnostics_{nullptr};
  std::shared_ptr<const modules::ControlInputSnapshot> latest_snapshot_;
  std::deque<hal::GimbalCommand> sent_commands_;
  std::vector<modules::PlannedGimbalPoint> last_successful_trajectory_;
  std::size_t last_successful_command_index_{1};
  double last_successful_target_distance_m_{-1.0};
  std::chrono::steady_clock::time_point last_successful_plan_time_{};
  int last_successful_plan_slot_{-1};
  int consecutive_mpc_failure_cycles_{0};
  bool control_projection_active_{false};
  bool output_projection_cleared_pending_{false};
  std::string output_projection_clear_reason_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> failed_{false};
};

void DrawDetections(cv::Mat& image, const std::vector<modules::ArmorDetection>& detections,
                    const modules::DetectorStats& stats) {
  tool::DrawArmorDetections(image, detections);

  const auto SUMMARY = fmt::format("detections={} candidates={} total={:.2f} ms", detections.size(),
                                   stats.threshold_candidates, stats.total_ms);
  cv::putText(image, SUMMARY, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2,
              cv::LINE_AA);
}

void LogPnpHealth(const modules::ArmorPnpFrameResult& result, std::uint64_t sequence,
                  std::size_t total_truth_armors) {
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
    if (attempt.source != modules::PnpInputSource::GROUND_TRUTH)
      continue;
    ++truth_attempted;
    if (!attempt.estimate)
      continue;
    ++truth_succeeded;
    max_rmse = std::max(max_rmse, attempt.estimate->reprojection_rmse_px);
    max_position_error =
        std::max(max_position_error, attempt.estimate->position_error_m.value_or(0.0));
    max_rotation_error =
        std::max(max_rotation_error, attempt.estimate->rotation_error_deg.value_or(0.0));
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
  const auto MATCHED_DETECTION = std::find_if(
      result.attempts.begin(), result.attempts.end(), [](const modules::ArmorPnpAttempt& attempt) {
        return attempt.source == modules::PnpInputSource::DETECTION && attempt.estimate &&
               attempt.estimate->truth_id;
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

int Run() {
  try {
    const std::filesystem::path CONFIG_ROOT = CONFIG_FILE_PATH;
    const std::filesystem::path PROJECT_ROOT = PROJECT_ROOT_PATH;
    Logger::Instance().InitFromFile(CONFIG_ROOT / "core/logger.yaml");
    std::signal(SIGINT, HandleStopSignal);
    std::signal(SIGTERM, HandleStopSignal);

    modules::YoloArmorDetector detector;
    try {
      const auto DETECTOR_YAML =
          ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_detector.yaml");
      detector.Init(modules::ParseArmorDetectorConfig(DETECTOR_YAML, PROJECT_ROOT));
    } catch (const std::exception& error) {
      MV_LOG_ERROR("App", "armor detector initialization failed: {}", error.what());
      return 2;
    }

    const auto PNP_YAML = ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_pnp.yaml", 2);
    modules::ArmorPnp pnp(modules::ParseArmorPnpConfig(PNP_YAML));
    const auto PREDICTOR_YAML =
        ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_predictor.yaml", 2);
    modules::ArmorPredictor predictor(modules::ParseArmorPredictorConfig(PREDICTOR_YAML));
    const auto REFINER_YAML =
        ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_corner_refiner.yaml");
    modules::ArmorCornerRefiner corner_refiner(
        modules::ParseArmorCornerRefinerConfig(REFINER_YAML));

    const auto CAMERA_SELECTION = LoadCameraSelection(CONFIG_ROOT);
    const auto CAMERA_CONFIG = ConfigLoader::LoadFile(CAMERA_SELECTION.config_path);
    auto camera = hal::CreateCamera(CAMERA_SELECTION.backend);
    MV_LOG_INFO("Config", "camera backend={} config={}", CAMERA_SELECTION.backend,
                CAMERA_SELECTION.config_path.string());
    if (!camera->Open(CAMERA_CONFIG)) {
      MV_LOG_ERROR("App", "{} camera open failed", CAMERA_SELECTION.backend);
      return 3;
    }

    const bool PREVIEW_ENABLED = LoadDebugWindowEnabled(CONFIG_ROOT / "tool/debug_window.yaml");
    std::unique_ptr<tool::DebugWindow> window;
    if (PREVIEW_ENABLED) {
      window = std::make_unique<tool::DebugWindow>(K_WINDOW_NAME);
    }

    std::unique_ptr<tool::foxglove::VisionDebugPublisher> foxglove_publisher;
    try {
      const auto FOXGLOVE_PATH = CONFIG_ROOT / "tool/foxglove.yaml";
      const auto FOXGLOVE_YAML = ConfigLoader::LoadFile(FOXGLOVE_PATH);
      auto foxglove_config = tool::foxglove::ParseConfig(FOXGLOVE_YAML, FOXGLOVE_PATH);
      if (foxglove_config.enabled) {
        foxglove_publisher =
            std::make_unique<tool::foxglove::VisionDebugPublisher>(std::move(foxglove_config));
        if (!foxglove_publisher->IsRunning()) {
          MV_LOG_WARN("App", "Foxglove configured but no live or recording sink started");
        }
      }
    } catch (const std::exception& error) {
      MV_LOG_ERROR("App", "Foxglove disabled after initialization failure: {}", error.what());
      foxglove_publisher.reset();
    }

    std::unique_ptr<ControlRuntime> control_runtime;
    if (CAMERA_SELECTION.backend == "talos") {
      const auto FIRE_YAML = ConfigLoader::LoadFile(CONFIG_ROOT / "modules/fire_control.yaml");
      const auto PLANNER_YAML =
          ConfigLoader::LoadFile(CONFIG_ROOT / "modules/gimbal_trajectory_planner.yaml");
      auto command_sink = std::make_unique<hal::TalosGimbalCommandSink>();
      if (!command_sink->Open(CAMERA_CONFIG)) {
        MV_LOG_ERROR("App", "Talos command sink initialization failed");
        return 6;
      }
      control_runtime = std::make_unique<ControlRuntime>(
          modules::ParseFireControlConfig(FIRE_YAML),
          modules::ParseGimbalTrajectoryPlannerConfig(PLANNER_YAML), std::move(command_sink),
          foxglove_publisher.get());
      control_runtime->Start();
      MV_LOG_INFO("Control", "Talos 100 Hz trajectory planning and fire control started");
    }

    while (g_stop_requested == 0) {
      if (control_runtime && control_runtime->Failed()) {
        MV_LOG_ERROR("App", "control thread failed; stopping vision pipeline safely");
        return 7;
      }
      hal::CameraFrame frame;
      const auto STATUS = camera->Grab(frame);

      if (STATUS == hal::GrabStatus::OK) {
        try {
          const auto DETECTIONS = detector.Detect(frame.image);
          const auto STATS = detector.LastStats();
          cv::Mat gray_image;
          cv::cvtColor(frame.image, gray_image, cv::COLOR_BGR2GRAY);
          std::vector<modules::CornerRefinementResult> refinements;
          refinements.reserve(DETECTIONS.size());
          for (const auto& detection : DETECTIONS) {
            refinements.push_back(corner_refiner.Refine(gray_image, detection.corners));
          }
          const auto PNP_RESULT = pnp.ProcessFrame(frame, DETECTIONS, refinements);
          const auto PREDICTION_RESULT = predictor.ProcessFrame(frame, PNP_RESULT);
          if (control_runtime && frame.geometry) {
            control_runtime->Update(PREDICTION_RESULT, *frame.geometry);
          }
          LogPnpHealth(PNP_RESULT, frame.sequence,
                       frame.geometry ? frame.geometry->armors.size() : 0);
          if (foxglove_publisher) {
            foxglove_publisher->Publish(frame, DETECTIONS, STATS, PNP_RESULT, PREDICTION_RESULT);
          }
          if (window) {
            cv::Mat debug_image = frame.image.clone();
            DrawDetections(debug_image, DETECTIONS, STATS);
            window->Show(debug_image);
          }
        } catch (const std::exception& error) {
          MV_LOG_ERROR("App", "armor detection failed: {}", error.what());
          return 5;
        }
      } else if (STATUS == hal::GrabStatus::DISCONNECTED || STATUS == hal::GrabStatus::FATAL) {
        MV_LOG_ERROR("App", "camera grab failed: {}", hal::GrabStatusName(STATUS));
        return 4;
      }

      if (window && window->Poll().exit_requested) {
        return 0;
      }
    }
    MV_LOG_INFO("App", "stop signal received");
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[App] FATAL: %s\n", error.what());
    return 1;
  }
}

}  // namespace mv::app

int main() {
  return mv::app::Run();
}
