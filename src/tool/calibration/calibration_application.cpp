#include "tool/calibration/calibration_application.hpp"

#include "core/logger.hpp"
#include "hal/camera/mindvision/mindvision_camera.hpp"
#include "tool/debug/debug_window.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace mv::tool::calibration {
namespace {

constexpr char K_WINDOW_NAME[] = "MiracleVision Camera Calibration";
constexpr int K_SPACE_KEY = 32;
volatile std::sig_atomic_t g_stop_requested = 0;

void HandleStopSignal(int) noexcept {
  g_stop_requested = 1;
}

std::string SessionTimestamp() {
  const auto NOW = std::chrono::system_clock::now();
  const std::time_t TIME = std::chrono::system_clock::to_time_t(NOW);
  std::tm local_time{};
  localtime_r(&TIME, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::filesystem::path CreateSessionDirectory(const std::filesystem::path& output_root) {
  std::filesystem::create_directories(output_root);
  const std::string BASE_NAME = SessionTimestamp();
  // 同一秒内重复启动时追加编号，避免覆盖已有样本和验收记录。
  for (int suffix = 0; suffix < 1000; ++suffix) {
    const std::string NAME = suffix == 0 ? BASE_NAME : fmt::format("{}_{:02d}", BASE_NAME, suffix);
    auto session_dir = output_root / NAME;
    std::error_code error;
    if (std::filesystem::create_directory(session_dir, error)) {
      std::filesystem::create_directories(session_dir / "images");
      return session_dir;
    }
    if (error && error != std::errc::file_exists) {
      throw std::filesystem::filesystem_error("cannot create calibration session", session_dir,
                                              error);
    }
  }
  throw std::runtime_error("cannot allocate a unique calibration session directory");
}

void DrawText(cv::Mat& image, const std::vector<std::string>& lines,
              const cv::Scalar& color = {0, 255, 0}) {
  int y = 28;
  for (const auto& line : lines) {
    cv::putText(image, line, {14, y}, cv::FONT_HERSHEY_SIMPLEX, 0.58, {0, 0, 0}, 3, cv::LINE_AA);
    cv::putText(image, line, {14, y}, cv::FONT_HERSHEY_SIMPLEX, 0.58, color, 1, cv::LINE_AA);
    y += 25;
  }
}

bool IsKey(int key, char lower, char upper) {
  return key == lower || key == upper;
}

void RemoveStaleIntrinsics(const std::filesystem::path& path) {
  std::error_code error;
  std::filesystem::remove(path, error);
  if (error) {
    throw std::filesystem::filesystem_error("cannot remove stale intrinsics", path, error);
  }
}

}  // namespace

CalibrationApplication::CalibrationApplication(CalibrationSettings settings,
                                               const YAML::Node& camera_config)
    : settings_(std::move(settings)), camera_config_(camera_config) {}

int CalibrationApplication::Run() {
  std::signal(SIGINT, HandleStopSignal);
  std::signal(SIGTERM, HandleStopSignal);

  hal::MindVisionCamera camera;
  if (!camera.Open(camera_config_)) {
    MV_LOG_ERROR("Calibration", "MindVision camera open failed");
    return 2;
  }
  const auto CAMERA_INFO = camera.Info();
  if (CAMERA_INFO.output_width != 1280 || CAMERA_INFO.output_height != 720 ||
      CAMERA_INFO.pixel_format != hal::PixelFormat::BGR8) {
    MV_LOG_ERROR("Calibration", "expected 1280x720 BGR8, camera reports {}x{} format={}",
                 CAMERA_INFO.output_width, CAMERA_INFO.output_height,
                 static_cast<int>(CAMERA_INFO.pixel_format));
    camera.Close();
    return 3;
  }

  const auto SESSION_DIR = CreateSessionDirectory(settings_.output_dir);
  const auto SESSION_PATH = SESSION_DIR / "session.yaml";
  const auto INTRINSICS_PATH = SESSION_DIR / "intrinsics.yaml";
  CameraCalibrator calibrator(settings_, {CAMERA_INFO.output_width, CAMERA_INFO.output_height});
  std::optional<CalibrationResult> last_result;
  std::string last_message = "waiting for a complete chessboard";
  WriteSession(SESSION_PATH, settings_, CAMERA_INFO, calibrator.Samples(), last_result,
               "collecting");
  MV_LOG_INFO("Calibration", "session directory: {}", SESSION_DIR.string());

  tool::DebugWindow window(K_WINDOW_NAME, tool::WindowMode::NORMAL);
  while (g_stop_requested == 0) {
    hal::CameraFrame frame;
    const auto GRAB_STATUS = camera.Grab(frame);
    if (GRAB_STATUS == hal::GrabStatus::TIMEOUT || GRAB_STATUS == hal::GrabStatus::INVALID_FRAME) {
      continue;
    }
    if (GRAB_STATUS != hal::GrabStatus::OK) {
      MV_LOG_ERROR("Calibration", "camera grab failed: {}", hal::GrabStatusName(GRAB_STATUS));
      WriteSession(SESSION_PATH, settings_, CAMERA_INFO, calibrator.Samples(), last_result,
                   "aborted");
      camera.Close();
      return 4;
    }

    const FrameObservation OBSERVATION = calibrator.Observe(frame.image);
    const bool LIKELY_DUPLICATE = calibrator.IsLikelyDuplicate(OBSERVATION);
    cv::Mat preview = frame.image.clone();
    if (OBSERVATION.found) {
      cv::drawChessboardCorners(preview, {settings_.board_columns, settings_.board_rows},
                                OBSERVATION.corners, true);
    }
    const cv::Scalar STATUS_COLOR = OBSERVATION.found && OBSERVATION.sharp_enough
                                        ? cv::Scalar(0, 255, 0)
                                        : cv::Scalar(0, 180, 255);
    std::vector<std::string> hud = {
        fmt::format("samples: {} / {}", calibrator.ActiveSampleCount(), settings_.min_samples),
        fmt::format("board: {}  sharpness: {:.1f} / {:.1f}",
                    OBSERVATION.found ? "found" : "not found", OBSERVATION.sharpness,
                    settings_.min_sharpness),
        LIKELY_DUPLICATE ? "warning: current pose is similar to an active sample" : last_message,
    };
    if (last_result) {
      hud.push_back(fmt::format("RMS: {:.3f}  max view: {:.3f}  grid: {}/9", last_result->rms_px,
                                last_result->max_view_rms_px,
                                last_result->coverage.occupied_grid_cells));
    }
    DrawText(preview, hud, STATUS_COLOR);
    window.Show(preview);
    const auto EVENT = window.Poll(1);
    if (EVENT.exit_requested)
      break;

    if (EVENT.key == K_SPACE_KEY) {
      if (!OBSERVATION.found) {
        last_message = "capture rejected: chessboard not found";
        continue;
      }
      if (!OBSERVATION.sharp_enough) {
        last_message = "capture rejected: image is not sharp enough";
        continue;
      }
      const std::size_t FILE_INDEX = calibrator.Samples().size() + 1;
      const auto RELATIVE_IMAGE_PATH =
          std::filesystem::path("images") / fmt::format("sample_{:04d}.png", FILE_INDEX);
      const auto ABSOLUTE_IMAGE_PATH = SESSION_DIR / RELATIVE_IMAGE_PATH;
      if (!cv::imwrite(ABSOLUTE_IMAGE_PATH.string(), frame.image,
                       {cv::IMWRITE_PNG_COMPRESSION, 3})) {
        last_message = "capture failed: cannot save PNG";
        MV_LOG_ERROR("Calibration", "cannot save image: {}", ABSOLUTE_IMAGE_PATH.string());
        continue;
      }
      if (!calibrator.AddSample(OBSERVATION, RELATIVE_IMAGE_PATH)) {
        last_message = "capture rejected by sample validation";
        continue;
      }
      // 样本集合变化后旧求解结果立即失效，防止遗留内参被误认为当前会话已通过。
      last_result.reset();
      RemoveStaleIntrinsics(INTRINSICS_PATH);
      WriteSession(SESSION_PATH, settings_, CAMERA_INFO, calibrator.Samples(), last_result,
                   "collecting");
      last_message = fmt::format("accepted sample {}", calibrator.Samples().back().id);
      MV_LOG_INFO("Calibration", "{}", last_message);
      continue;
    }

    if (IsKey(EVENT.key, 'u', 'U')) {
      const auto UNDONE_ID = calibrator.UndoLastSample();
      if (UNDONE_ID) {
        // 排除样本与新增样本具有相同的结果失效语义，但保留原图供事后审计。
        last_result.reset();
        RemoveStaleIntrinsics(INTRINSICS_PATH);
        WriteSession(SESSION_PATH, settings_, CAMERA_INFO, calibrator.Samples(), last_result,
                     "collecting");
        last_message = fmt::format("excluded sample {}", *UNDONE_ID);
        MV_LOG_INFO("Calibration", "{}", last_message);
      } else {
        last_message = "no active sample to exclude";
      }
      continue;
    }

    if (IsKey(EVENT.key, 'c', 'C')) {
      last_result = calibrator.Solve();
      if (last_result->accepted) {
        WriteIntrinsics(INTRINSICS_PATH, settings_, CAMERA_INFO, *last_result);
        WriteSession(SESSION_PATH, settings_, CAMERA_INFO, calibrator.Samples(), last_result,
                     "passed");
        last_message = fmt::format("PASS: intrinsics written, RMS {:.3f} px", last_result->rms_px);
        MV_LOG_INFO("Calibration", "{} ({})", last_message, INTRINSICS_PATH.string());
      } else {
        RemoveStaleIntrinsics(INTRINSICS_PATH);
        WriteSession(SESSION_PATH, settings_, CAMERA_INFO, calibrator.Samples(), last_result,
                     "failed");
        last_message =
            fmt::format("quality check failed: {} condition(s)", last_result->failures.size());
        MV_LOG_WARN("Calibration", "{}", last_message);
        for (const auto& failure : last_result->failures) {
          MV_LOG_WARN("Calibration", "  {}", failure);
        }
        if (last_result->solved) {
          MV_LOG_WARN("Calibration", "worst sample={} RMS={:.3f}px", last_result->worst_sample_id,
                      last_result->max_view_rms_px);
        }
      }
    }
  }

  const std::string FINAL_STATUS =
      last_result && last_result->accepted ? "passed" : (last_result ? "failed" : "collecting");
  WriteSession(SESSION_PATH, settings_, CAMERA_INFO, calibrator.Samples(), last_result,
               FINAL_STATUS);
  camera.Close();
  MV_LOG_INFO("Calibration", "session closed: {}", SESSION_DIR.string());
  return 0;
}

}  // namespace mv::tool::calibration
