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
  std::filesystem::create_directories(SESSION_DIR / "images");
  const auto SESSION_PATH = SESSION_DIR / "session.yaml";
  std::vector<std::filesystem::path> images;
  std::size_t next_image_id = 1;
  std::string last_message = "Space/S: save image   U: exclude last   Q/Esc: finish";
  WriteSession(SESSION_PATH, settings_, CAMERA_INFO, {}, std::nullopt, "collecting");
  MV_LOG_INFO("Calibration", "capture directory: {}", SESSION_DIR.string());

  tool::DebugWindow window(K_WINDOW_NAME, tool::WindowMode::NORMAL);
  while (g_stop_requested == 0) {
    frame::FramePacket packet;
    const auto GRAB_STATUS = camera.Grab(packet);
    if (GRAB_STATUS == hal::GrabStatus::TIMEOUT || GRAB_STATUS == hal::GrabStatus::INVALID_FRAME) {
      continue;
    }
    if (GRAB_STATUS != hal::GrabStatus::OK) {
      MV_LOG_ERROR("Calibration", "camera grab failed: {}", hal::GrabStatusName(GRAB_STATUS));
      WriteSession(SESSION_PATH, settings_, CAMERA_INFO, {}, std::nullopt, "aborted");
      camera.Close();
      return 4;
    }

    // 采集阶段不运行角点检测或清晰度计算，避免检测耗时阻塞预览和按键。
    cv::Mat preview = packet.capture.image.clone();
    DrawText(preview, {fmt::format("saved images: {}", images.size()), last_message,
                       "Space/S: save   U: exclude last   Q/Esc: finish"});
    window.Show(preview);
    const auto EVENT = window.Poll(1);
    if (EVENT.exit_requested)
      break;

    if (EVENT.key == K_SPACE_KEY || IsKey(EVENT.key, 's', 'S')) {
      const auto IMAGE_PATH = SESSION_DIR / "images" /
                              fmt::format("sample_{:04d}.png", next_image_id);
      if (!cv::imwrite(IMAGE_PATH.string(), packet.capture.image,
                       {cv::IMWRITE_PNG_COMPRESSION, 3})) {
        throw std::runtime_error("cannot save image: " + IMAGE_PATH.string());
      }
      ++next_image_id;
      images.push_back(IMAGE_PATH);
      last_message = "saved " + IMAGE_PATH.filename().string();
      MV_LOG_INFO("Calibration", "{}", last_message);
    } else if (IsKey(EVENT.key, 'u', 'U') && !images.empty()) {
      // 保留排除的原图，但移出离线求解扫描的 images 目录。
      std::filesystem::create_directories(SESSION_DIR / "excluded");
      std::filesystem::rename(images.back(), SESSION_DIR / "excluded" / images.back().filename());
      last_message = "excluded " + images.back().filename().string();
      images.pop_back();
    }
  }
  WriteSession(SESSION_PATH, settings_, CAMERA_INFO, {}, std::nullopt, "captured");
  camera.Close();
  MV_LOG_INFO("Calibration", "session closed: {}", SESSION_DIR.string());
  return 0;
}

int CalibrationApplication::SolveOffline(const std::filesystem::path& session_dir) {
  g_stop_requested = 0;
  std::signal(SIGINT, HandleStopSignal);
  std::signal(SIGTERM, HandleStopSignal);
  // 内参必须绑定采集时的设备、ROI 和棋盘尺寸，不能使用当前相机配置替代。
  const auto SESSION = YAML::LoadFile((session_dir / "session.yaml").string());
  const auto CAMERA = SESSION["camera"];
  hal::CameraInfo info;
  info.device_name = CAMERA["device_name"].as<std::string>();
  info.sensor_width = CAMERA["sensor_width"].as<int>();
  info.sensor_height = CAMERA["sensor_height"].as<int>();
  info.output_width = CAMERA["output_width"].as<int>();
  info.output_height = CAMERA["output_height"].as<int>();
  info.roi_offset_x = CAMERA["roi_offset_x"].as<int>();
  info.roi_offset_y = CAMERA["roi_offset_y"].as<int>();
  info.exposure_us = CAMERA["exposure_us"].as<int>();
  info.pixel_format = hal::PixelFormat::BGR8;
  auto settings = settings_;
  settings.board_columns = SESSION["board"]["columns"].as<int>();
  settings.board_rows = SESSION["board"]["rows"].as<int>();
  settings.square_size_mm = SESSION["board"]["square_size_mm"].as<double>();
  if (info.output_width != 1280 || info.output_height != 720 || settings.board_columns < 2 ||
      settings.board_rows < 2 || !(settings.square_size_mm > 0.0)) {
    throw std::runtime_error("invalid captured image size or board metadata");
  }
  std::vector<std::filesystem::path> images;
  for (const auto& entry : std::filesystem::directory_iterator(session_dir / "images")) {
    if (entry.is_regular_file() && entry.path().extension() == ".png") {
      images.push_back(entry.path());
    }
  }
  std::sort(images.begin(), images.end());
  if (images.empty()) {
    throw std::runtime_error("no PNG images in capture session");
  }
  // 每次解算独立存档；失败重试不会覆盖此前结果，也不会修改采集原图。
  const auto RESULT_DIR = CreateSessionDirectory(session_dir / "solutions");
  CameraCalibrator calibrator(settings, {info.output_width, info.output_height});
  tool::DebugWindow review_window("Calibration review - press any key for next image",
                                   tool::WindowMode::NORMAL);
  for (std::size_t index = 0; index < images.size(); ++index) {
    MV_LOG_INFO("Calibration", "detecting {}/{}: {}", index + 1, images.size(),
                images[index].filename().string());
    const auto IMAGE = cv::imread(images[index].string(), cv::IMREAD_COLOR);
    if (IMAGE.empty() || IMAGE.cols != info.output_width || IMAGE.rows != info.output_height) {
      throw std::runtime_error("unreadable image or size mismatch: " + images[index].string());
    }
    const auto OBSERVATION = calibrator.Observe(IMAGE);
    cv::Mat preview = IMAGE.clone();
    cv::drawChessboardCorners(preview, {settings.board_columns, settings.board_rows},
                              OBSERVATION.corners, OBSERVATION.found);
    DrawText(preview,
             {fmt::format("{}/{}: {}", index + 1, images.size(), images[index].filename().string()),
              OBSERVATION.found ? "Board found" : "Board NOT found - will be skipped",
              "Any key: next image   Close window / Ctrl+C: cancel"},
             OBSERVATION.found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 180, 255));
    review_window.Show(preview);
    // 短轮询保持窗口关闭和 Ctrl+C 可响应；包括 Q/Esc 在内的任意键均用于下一张。
    bool confirmed = false;
    while (g_stop_requested == 0) {
      const auto EVENT = review_window.Poll(30);
      if (EVENT.key >= 0) {
        confirmed = true;
        break;
      }
      if (EVENT.exit_requested) {
        break;
      }
    }
    if (!confirmed || g_stop_requested != 0) {
      WriteSession(RESULT_DIR / "session.yaml", settings, info, calibrator.Samples(),
                   std::nullopt, "aborted");
      MV_LOG_INFO("Calibration", "review cancelled; no calibration performed: {}",
                  RESULT_DIR.string());
      return 6;
    }
    if (!calibrator.AddSample(OBSERVATION, std::filesystem::relative(images[index], RESULT_DIR))) {
      MV_LOG_WARN("Calibration", "skipped {}: board_found={}, sharpness={:.1f}",
                  images[index].filename().string(), OBSERVATION.found, OBSERVATION.sharpness);
    }
  }
  review_window.Close();
  MV_LOG_INFO("Calibration", "review complete; solving with {} accepted samples",
              calibrator.ActiveSampleCount());
  const auto RESULT = calibrator.Solve();
  if (RESULT.solved) {
    // 终端报告与文件采用同一份解算结果，包含未通过质量验收的已求解结果。
    YAML::Emitter output;
    output.SetDoublePrecision(15);
    output << YAML::BeginMap << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow
           << YAML::BeginSeq;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        output << RESULT.camera_matrix.at<double>(row, column);
      }
    }
    output << YAML::EndSeq << YAML::Key << "distortion_coefficients" << YAML::Value
           << YAML::Flow << YAML::BeginSeq;
    for (int index = 0; index < 5; ++index) {
      output << RESULT.distortion_coefficients.at<double>(0, index);
    }
    output << YAML::EndSeq << YAML::Key << "rms_px" << YAML::Value << RESULT.rms_px
           << YAML::Key << "max_view_rms_px" << YAML::Value << RESULT.max_view_rms_px
           << YAML::Key << "worst_sample_id" << YAML::Value << RESULT.worst_sample_id
           << YAML::Key << "accepted" << YAML::Value << RESULT.accepted << YAML::EndMap;
    fmt::print("\n{}\n", output.c_str());
    for (std::size_t index = 0; index < RESULT.per_view_rms_px.size(); ++index) {
      MV_LOG_INFO("Calibration", "sample {}: RMS={:.6f}px", RESULT.active_sample_ids[index],
                  RESULT.per_view_rms_px[index]);
    }
  }
  WriteSession(RESULT_DIR / "session.yaml", settings, info, calibrator.Samples(), RESULT,
               RESULT.accepted ? "passed" : "failed");
  if (!RESULT.accepted) {
    for (const auto& failure : RESULT.failures) {
      MV_LOG_WARN("Calibration", "{}", failure);
    }
    if (RESULT.solved) {
      MV_LOG_WARN("Calibration", "worst sample={} RMS={:.3f}px", RESULT.worst_sample_id,
                  RESULT.max_view_rms_px);
    }
    MV_LOG_WARN("Calibration", "failed report: {}", RESULT_DIR.string());
    return 5;
  }
  WriteIntrinsics(RESULT_DIR / "intrinsics.yaml", settings, info, RESULT);
  MV_LOG_INFO("Calibration", "PASS: RMS={:.3f}px, intrinsics: {}", RESULT.rms_px,
              (RESULT_DIR / "intrinsics.yaml").string());
  return 0;
}

}  // namespace mv::tool::calibration
