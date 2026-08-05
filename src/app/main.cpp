#include "app/main.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/mindvision/mindvision_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_detector/armor_detector_config.hpp"
#include "tool/debug/armor_detection_overlay.hpp"
#include "tool/debug/debug_window.hpp"
#include "tool/foxglove/armor_debug_publisher.hpp"
#include "tool/foxglove/foxglove_config.hpp"

#include <csignal>
#include <cstdio>
#include <exception>
#include <memory>
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

void DrawDetections(cv::Mat& image, const std::vector<modules::ArmorDetection>& detections,
                    const modules::DetectorStats& stats) {
  tool::DrawArmorDetections(image, detections);

  const auto SUMMARY = fmt::format("detections={} candidates={} total={:.2f} ms", detections.size(),
                                   stats.threshold_candidates, stats.total_ms);
  cv::putText(image, SUMMARY, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2,
              cv::LINE_AA);
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

    const auto CAMERA_CONFIG = ConfigLoader::LoadFile(CONFIG_ROOT / "hal/camera/mindvision.yaml");
    hal::MindVisionCamera camera;
    if (!camera.Open(CAMERA_CONFIG)) {
      MV_LOG_ERROR("App", "camera open failed");
      return 3;
    }

    const bool PREVIEW_ENABLED = LoadDebugWindowEnabled(CONFIG_ROOT / "tool/debug_window.yaml");
    std::unique_ptr<tool::DebugWindow> window;
    if (PREVIEW_ENABLED) {
      window = std::make_unique<tool::DebugWindow>(K_WINDOW_NAME);
    }

    std::unique_ptr<tool::foxglove::ArmorDebugPublisher> foxglove_publisher;
    try {
      const auto FOXGLOVE_PATH = CONFIG_ROOT / "tool/foxglove.yaml";
      const auto FOXGLOVE_YAML = ConfigLoader::LoadFile(FOXGLOVE_PATH);
      auto foxglove_config = tool::foxglove::ParseConfig(FOXGLOVE_YAML, FOXGLOVE_PATH);
      if (foxglove_config.enabled) {
        foxglove_publisher =
            std::make_unique<tool::foxglove::ArmorDebugPublisher>(std::move(foxglove_config));
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
      const auto STATUS = camera.Grab(frame);

      if (STATUS == hal::GrabStatus::OK) {
        try {
          const auto DETECTIONS = detector.Detect(frame.image);
          const auto STATS = detector.LastStats();
          if (foxglove_publisher) {
            foxglove_publisher->Publish(frame, DETECTIONS, STATS);
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
