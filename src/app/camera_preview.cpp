#include "app/camera_preview.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/i_camera.hpp"
#include "hal/camera/mindvision_camera.hpp"

#include <cstdio>
#include <exception>
#include <memory>

#include <filesystem>
#include <opencv2/highgui.hpp>

namespace mv::app {
namespace {

constexpr char kWindowName[] = "MiracleVision Camera Preview";
constexpr int kExpectedWidth = 1280;
constexpr int kExpectedHeight = 720;
constexpr int kMaximumConsecutiveBadFrames = 5;

bool PreviewExitRequested() {
  const int key = cv::waitKey(1);
  if (key == 27 || key == 'q' || key == 'Q') {
    return true;
  }
  return cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE) < 1.0;
}

bool HasExpectedFormat(const hal::CameraFrame& frame) {
  return !frame.image.empty() && frame.image.cols == kExpectedWidth &&
         frame.image.rows == kExpectedHeight && frame.image.type() == CV_8UC3;
}

}  // namespace

int RunCameraPreview() {
  try {
    const std::filesystem::path logger_path =
        std::filesystem::path(CONFIG_FILE_PATH) / "core/logger.yaml";
    const std::filesystem::path camera_path =
        std::filesystem::path(CONFIG_FILE_PATH) / "hal/camera/mindvision.yaml";

    Logger::Instance().InitFromFile(logger_path);
    const auto camera_config = ConfigLoader::LoadFile(camera_path);
    MV_LOG_INFO("Config", "camera config: {}",
                std::filesystem::absolute(camera_path).lexically_normal().string());

    std::unique_ptr<hal::ICamera> camera = std::make_unique<hal::MindVisionCamera>();
    if (!camera->Open(camera_config)) {
      MV_LOG_ERROR("CameraPreview",
                   "camera open failed; no fallback or automatic recovery will run");
      return 2;
    }

    const auto info = camera->Info();
    if (info.output_width != kExpectedWidth || info.output_height != kExpectedHeight ||
        info.pixel_format != hal::PixelFormat::BGR8) {
      MV_LOG_ERROR("CameraPreview", "unexpected camera output {}x{}; expected 1280x720 BGR8",
                   info.output_width, info.output_height);
      camera->Close();
      return 3;
    }

    MV_LOG_INFO("CameraPreview",
                "opened '{}' sensor={}x{} output={}x{} ROI=({}, {}) exposure={}us timeout={}ms",
                info.device_name, info.sensor_width, info.sensor_height, info.output_width,
                info.output_height, info.roi_offset_x, info.roi_offset_y, info.exposure_us,
                info.grab_timeout_ms);
    MV_LOG_INFO("CameraPreview", "press Q or Esc, or close the window, to exit");

    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    int consecutive_bad_frames = 0;
    int result = 0;

    while (true) {
      hal::CameraFrame frame;
      const auto status = camera->Grab(frame);

      if (status == hal::GrabStatus::OK && HasExpectedFormat(frame)) {
        consecutive_bad_frames = 0;
        cv::imshow(kWindowName, frame.image);
      } else if (status == hal::GrabStatus::TIMEOUT) {
        MV_LOG_WARN("CameraPreview", "camera grab timeout");
      } else if (status == hal::GrabStatus::DISCONNECTED || status == hal::GrabStatus::FATAL) {
        MV_LOG_ERROR("CameraPreview", "camera grab failed: {}", hal::GrabStatusName(status));
        result = 4;
        break;
      } else {
        ++consecutive_bad_frames;
        MV_LOG_WARN("CameraPreview", "invalid frame ({}/{}), status={}", consecutive_bad_frames,
                    kMaximumConsecutiveBadFrames, hal::GrabStatusName(status));
        if (consecutive_bad_frames > kMaximumConsecutiveBadFrames) {
          MV_LOG_ERROR("CameraPreview", "too many consecutive invalid frames");
          result = 5;
          break;
        }
      }

      if (PreviewExitRequested()) {
        break;
      }
    }

    camera->Close();
    cv::destroyAllWindows();
    return result;
  } catch (const std::exception& error) {
    cv::destroyAllWindows();
    std::fprintf(stderr, "[CameraPreview] FATAL: %s\n", error.what());
    return 1;
  }
}

}  // namespace mv::app
