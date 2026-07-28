#include "app/main.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/mindvision/mindvision_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_detector/armor_detector_config.hpp"
#include "tool/debug/armor_detection_overlay.hpp"
#include "tool/debug/debug_window.hpp"

#include <cstdio>
#include <exception>
#include <vector>

#include <filesystem>
#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

namespace mv::app {
namespace {

constexpr char K_WINDOW_NAME[] = "MiracleVision Camera Preview";

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

    tool::DebugWindow window(K_WINDOW_NAME);
    while (true) {
      hal::CameraFrame frame;
      const auto STATUS = camera.Grab(frame);

      if (STATUS == hal::GrabStatus::OK) {
        try {
          const auto DETECTIONS = detector.Detect(frame.image);
          cv::Mat debug_image = frame.image.clone();
          DrawDetections(debug_image, DETECTIONS, detector.LastStats());
          window.Show(debug_image);
        } catch (const std::exception& error) {
          MV_LOG_ERROR("App", "armor detection failed: {}", error.what());
          return 5;
        }
      } else if (STATUS == hal::GrabStatus::DISCONNECTED || STATUS == hal::GrabStatus::FATAL) {
        MV_LOG_ERROR("App", "camera grab failed: {}", hal::GrabStatusName(STATUS));
        return 4;
      }

      if (window.Poll().exit_requested) {
        return 0;
      }
    }
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[App] FATAL: %s\n", error.what());
    return 1;
  }
}

}  // namespace mv::app

int main() {
  return mv::app::Run();
}
