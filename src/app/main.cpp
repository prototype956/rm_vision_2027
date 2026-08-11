#include "app/main.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/camera_factory.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_detector/armor_detector_config.hpp"
#include "modules/armor_pnp/armor_pnp.hpp"
#include "tool/debug/armor_detection_overlay.hpp"
#include "tool/debug/debug_window.hpp"
#include "tool/foxglove/foxglove_config.hpp"
#include "tool/foxglove/vision_debug_publisher.hpp"

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <filesystem>
#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

namespace mv::app {
namespace {

constexpr char K_WINDOW_NAME[] = "MiracleVision Camera Preview";
volatile std::sig_atomic_t g_stop_requested = 0;

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
              "A/B seq={} raw_solved={}/{} refined_solved={}/{} refine={}/{} fallback={} "
              "corner_p95(raw/refined)={:.3f}/{:.3f}px depth_p95={:.4f}/{:.4f}m "
              "top_fallback={}({})",
              sequence, result.raw_solve_summary.succeeded, result.raw_solve_summary.attempted,
              result.refined_solve_summary.succeeded, result.refined_solve_summary.attempted,
              result.refinement_summary.succeeded, result.refinement_summary.attempted,
              result.refinement_summary.fallback,
              result.detection_raw_summary.mean_corner_error_px.p95,
              result.detection_refined_with_fallback_summary.mean_corner_error_px.p95,
              result.detection_raw_summary.depth_error_m.p95,
              result.detection_refined_with_fallback_summary.depth_error_m.p95,
              dominant_refinement_failure, dominant_refinement_failure_count);
  const auto MATCHED_RAW = std::find_if(
      result.attempts.begin(), result.attempts.end(), [](const modules::ArmorPnpAttempt& attempt) {
        return attempt.source == modules::PnpInputSource::DETECTION_RAW && attempt.estimate &&
               attempt.estimate->truth_id;
      });
  if (MATCHED_RAW != result.attempts.end()) {
    const auto& value = *MATCHED_RAW->estimate;
    MV_LOG_INFO("ArmorPnP",
                "matched raw truth={} corner du=[{:.1f},{:.1f},{:.1f},{:.1f}] "
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
    const auto REFINER_YAML =
        ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_corner_refiner.yaml", 4);
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

    while (g_stop_requested == 0) {
      hal::CameraFrame frame;
      const auto STATUS = camera->Grab(frame);

      if (STATUS == hal::GrabStatus::OK) {
        try {
          const auto DETECTIONS = detector.Detect(frame.image);
          const auto STATS = detector.LastStats();
          std::vector<modules::CornerRefinementResult> refinements;
          refinements.reserve(DETECTIONS.size());
          for (const auto& detection : DETECTIONS) {
            refinements.push_back(
                corner_refiner.Refine(frame.image, detection.corners, detection.color,
                                      modules::ArmorTypeForLabel(detection.label)));
          }
          const auto PNP_RESULT = pnp.ProcessFrame(frame, DETECTIONS, refinements);
          LogPnpHealth(PNP_RESULT, frame.sequence,
                       frame.geometry ? frame.geometry->armors.size() : 0);
          if (foxglove_publisher) {
            foxglove_publisher->Publish(frame, DETECTIONS, STATS, PNP_RESULT);
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
