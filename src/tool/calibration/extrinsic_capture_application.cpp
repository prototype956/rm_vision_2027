#include "tool/calibration/extrinsic_capture_application.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "geometry/rigid_transform.hpp"
#include "hal/camera/talos/talos_camera.hpp"
#include "tool/debug/debug_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <fmt/format.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::tool::calibration {
namespace {

constexpr char K_CONFIG_CONTEXT[] = "extrinsic calibration config";
constexpr char K_WINDOW_NAME[] = "MiracleVision Simulation Extrinsic Calibration";
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
  for (int suffix = 0; suffix < 1000; ++suffix) {
    const std::string NAME = suffix == 0 ? BASE_NAME : fmt::format("{}_{:02d}", BASE_NAME, suffix);
    auto session_dir = output_root / NAME;
    std::error_code error;
    if (std::filesystem::create_directory(session_dir, error))
      return session_dir;
    if (error && error != std::errc::file_exists) {
      throw std::filesystem::filesystem_error("cannot create calibration session", session_dir,
                                              error);
    }
  }
  throw std::runtime_error("cannot allocate a unique calibration session directory");
}

void WriteEmitterAtomically(const std::filesystem::path& path, const YAML::Emitter& emitter) {
  if (!emitter.good())
    throw std::runtime_error("cannot serialize session: " + emitter.GetLastError());
  std::filesystem::create_directories(path.parent_path());
  const auto TEMP_PATH = path.string() + ".tmp";
  {
    std::ofstream output(TEMP_PATH, std::ios::trunc);
    if (!output) {
      throw std::runtime_error("cannot open output file: " + TEMP_PATH);
    }
    output << emitter.c_str() << '\n';
    output.close();
    if (!output) {
      throw std::runtime_error("cannot write output file: " + TEMP_PATH);
    }
  }
  std::error_code error;
  std::filesystem::rename(TEMP_PATH, path, error);
  // Linux 同目录 rename 原子覆盖，失败时不能删除上一份有效清单。
  if (error) {
    std::filesystem::remove(TEMP_PATH);
    throw std::runtime_error("cannot replace output file '" + path.string() +
                             "': " + error.message());
  }
}

bool IsKey(int key, char lower, char upper) {
  return key == lower || key == upper;
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

void EmitVector3(YAML::Emitter& output, const geometry::Vector3& value) {
  output << YAML::Flow << YAML::BeginSeq << value.x() << value.y() << value.z() << YAML::EndSeq;
}

void EmitQuaternion(YAML::Emitter& output, const geometry::Quaternion& value) {
  output << YAML::Flow << YAML::BeginSeq << value.x() << value.y() << value.z() << value.w()
         << YAML::EndSeq;
}

void EmitTransform(YAML::Emitter& output, const geometry::RigidTransform& value) {
  output << YAML::BeginMap;
  output << YAML::Key << "translation" << YAML::Value;
  EmitVector3(output, value.translation);
  output << YAML::Key << "rotation_xyzw" << YAML::Value;
  EmitQuaternion(output, value.rotation);
  output << YAML::EndMap;
}

bool ValidTransform(const geometry::RigidTransform& value) {
  const auto& translation = value.translation;
  const auto& rotation = value.rotation;
  return translation.allFinite() && rotation.coeffs().allFinite() &&
         std::abs(rotation.norm() - 1.0) <= 1.0e-6;
}

bool ValidCameraModel(const frame::CameraModel& value, int expected_width, int expected_height) {
  return value.width == static_cast<std::uint32_t>(expected_width) &&
         value.height == static_cast<std::uint32_t>(expected_height) && std::isfinite(value.fx) &&
         std::isfinite(value.fy) && std::isfinite(value.cx) && std::isfinite(value.cy) &&
         value.fx > 0.0 && value.fy > 0.0 &&
         std::all_of(value.distortion.begin(), value.distortion.end(),
                     [](double coefficient) { return std::isfinite(coefficient); });
}

}  // namespace

ExtrinsicCaptureSettings ParseExtrinsicCaptureSettings(const YAML::Node& root,
                                                       const std::filesystem::path& project_root) {
  ConfigLoader::RejectUnknownKeys(
      root, {"schema_version", "backend", "expected", "board", "solve", "output_dir"},
      K_CONFIG_CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", K_CONFIG_CONTEXT) != 1) {
    throw ConfigError("extrinsic calibration config schema_version must be 1");
  }
  if (ConfigLoader::Require<std::string>(root, "backend", K_CONFIG_CONTEXT) != "talos") {
    throw ConfigError("extrinsic calibration config.backend must be talos");
  }

  const auto EXPECTED = root["expected"];
  ConfigLoader::RequireMap(EXPECTED, "extrinsic calibration config.expected");
  ConfigLoader::RejectUnknownKeys(EXPECTED, {"width", "height", "pixel_format"},
                                  "extrinsic calibration config.expected");
  const auto PIXEL_FORMAT =
      ConfigLoader::Require<std::string>(EXPECTED, "pixel_format", K_CONFIG_CONTEXT);

  const auto BOARD = root["board"];
  ConfigLoader::RequireMap(BOARD, "extrinsic calibration config.board");
  ConfigLoader::RejectUnknownKeys(BOARD, {"pattern_cols", "pattern_rows", "square_size_mm"},
                                  "extrinsic calibration config.board");

  ExtrinsicCaptureSettings settings;
  settings.expected_width = ConfigLoader::Require<int>(EXPECTED, "width", K_CONFIG_CONTEXT);
  settings.expected_height = ConfigLoader::Require<int>(EXPECTED, "height", K_CONFIG_CONTEXT);
  settings.board_columns =
      ConfigLoader::Require<int>(BOARD, "pattern_cols", "extrinsic calibration config.board");
  settings.board_rows =
      ConfigLoader::Require<int>(BOARD, "pattern_rows", "extrinsic calibration config.board");
  settings.square_size_mm =
      ConfigLoader::Require<double>(BOARD, "square_size_mm", "extrinsic calibration config.board");
  const auto SOLVE = root["solve"];
  ConfigLoader::RequireMap(SOLVE, "extrinsic calibration config.solve");
  ConfigLoader::RejectUnknownKeys(SOLVE, {"min_samples"}, "extrinsic calibration config.solve");
  settings.solve_min_samples =
      ConfigLoader::Require<int>(SOLVE, "min_samples", "extrinsic calibration config.solve");
  const auto OUTPUT_DIR = ConfigLoader::Require<std::string>(root, "output_dir", K_CONFIG_CONTEXT);
  settings.output_dir = ConfigLoader::ResolvePath(project_root, OUTPUT_DIR);

  if (settings.expected_width <= 0 || settings.expected_height <= 0 || PIXEL_FORMAT != "bgr8" ||
      settings.board_columns < 2 || settings.board_rows < 2 ||
      !std::isfinite(settings.square_size_mm) || settings.square_size_mm <= 0.0 ||
      settings.solve_min_samples < 3 || OUTPUT_DIR.empty()) {
    throw ConfigError("extrinsic calibration config contains an invalid setting");
  }
  return settings;
}

ExtrinsicCaptureApplication::ExtrinsicCaptureApplication(ExtrinsicCaptureSettings settings,
                                                         const YAML::Node& talos_config)
    : settings_(std::move(settings)), talos_config_(talos_config) {}

void ExtrinsicCaptureApplication::WriteSession(const std::string& status,
                                               const std::string& error) const {
  YAML::Emitter output;
  output.SetDoublePrecision(15);
  output << YAML::BeginMap;
  output << YAML::Key << "schema_version" << YAML::Value << 1;
  output << YAML::Key << "status" << YAML::Value << status;
  if (!error.empty())
    output << YAML::Key << "error" << YAML::Value << error;
  output << YAML::Key << "translation_unit" << YAML::Value << "m";
  output << YAML::Key << "transform_convention" << YAML::Value << "parent_t_child";
  output << YAML::Key << "source" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "backend" << YAML::Value << "talos";
  output << YAML::Key << "device_name" << YAML::Value << device_name_;
  output << YAML::EndMap;
  output << YAML::Key << "board" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "pattern_cols" << YAML::Value << settings_.board_columns;
  output << YAML::Key << "pattern_rows" << YAML::Value << settings_.board_rows;
  output << YAML::Key << "square_size_mm" << YAML::Value << settings_.square_size_mm;
  output << YAML::EndMap;
  output << YAML::Key << "samples" << YAML::Value << YAML::BeginSeq;
  for (const auto& sample : samples_) {
    output << YAML::BeginMap;
    output << YAML::Key << "id" << YAML::Value << sample.id;
    output << YAML::Key << "active" << YAML::Value << sample.active;
    output << YAML::Key << "image" << YAML::Value << sample.image_path.generic_string();
    output << YAML::Key << "frame_sequence" << YAML::Value << sample.frame_sequence;
    output << YAML::Key << "simulation_round_id" << YAML::Value << sample.simulation_round_id;
    output << YAML::Key << "capture_timestamp_ns" << YAML::Value << sample.capture_timestamp_ns;
    output << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "width" << YAML::Value << sample.camera.width;
    output << YAML::Key << "height" << YAML::Value << sample.camera.height;
    output << YAML::Key << "fx" << YAML::Value << sample.camera.fx;
    output << YAML::Key << "fy" << YAML::Value << sample.camera.fy;
    output << YAML::Key << "cx" << YAML::Value << sample.camera.cx;
    output << YAML::Key << "cy" << YAML::Value << sample.camera.cy;
    output << YAML::Key << "distortion" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const double VALUE : sample.camera.distortion)
      output << VALUE;
    output << YAML::EndSeq << YAML::EndMap;
    output << YAML::Key << "world_t_gimbal" << YAML::Value;
    EmitTransform(output, sample.world_t_gimbal);
    output << YAML::Key << "gimbal_t_camera_optical" << YAML::Value;
    EmitTransform(output, sample.gimbal_t_camera_optical);
    output << YAML::EndMap;
  }
  output << YAML::EndSeq << YAML::EndMap;
  WriteEmitterAtomically(session_path_, output);
}

int ExtrinsicCaptureApplication::Run() {
  g_stop_requested = 0;
  std::signal(SIGINT, HandleStopSignal);
  std::signal(SIGTERM, HandleStopSignal);

  hal::TalosCamera camera;
  if (!camera.Open(talos_config_)) {
    MV_LOG_ERROR("ExtrinsicCalibration", "Talos camera open failed");
    return 2;
  }
  const auto CAMERA_INFO = camera.Info();
  device_name_ = CAMERA_INFO.device_name;
  if (CAMERA_INFO.output_width != settings_.expected_width ||
      CAMERA_INFO.output_height != settings_.expected_height ||
      CAMERA_INFO.pixel_format != hal::PixelFormat::BGR8) {
    MV_LOG_ERROR("ExtrinsicCalibration", "expected {}x{} BGR8, camera reports {}x{} format={}",
                 settings_.expected_width, settings_.expected_height, CAMERA_INFO.output_width,
                 CAMERA_INFO.output_height, static_cast<int>(CAMERA_INFO.pixel_format));
    camera.Close();
    return 3;
  }

  session_dir_ = CreateSessionDirectory(settings_.output_dir);
  std::filesystem::create_directories(session_dir_ / "images");
  std::filesystem::create_directories(session_dir_ / "excluded");
  session_path_ = session_dir_ / "session.yaml";
  WriteSession("collecting");
  MV_LOG_INFO("ExtrinsicCalibration", "capture directory: {}", session_dir_.string());

  try {
    tool::DebugWindow window(K_WINDOW_NAME, tool::WindowMode::NORMAL);
    std::string last_message = "Space/S: save image   U: exclude last   Q/Esc: finish";
    while (g_stop_requested == 0) {
      frame::FramePacket packet;
      const auto GRAB_STATUS = camera.Grab(packet);
      if (GRAB_STATUS == hal::GrabStatus::TIMEOUT) {
        // 断帧期间也处理退出事件；保存键不作用于旧帧。
        const auto EVENT = window.Poll(1);
        if (EVENT.exit_requested) {
          if (!IsKey(EVENT.key, 'q', 'Q') && EVENT.key != 27)
            throw std::runtime_error("capture window closed");
          break;
        }
        continue;
      }
      if (GRAB_STATUS != hal::GrabStatus::OK) {
        MV_LOG_ERROR("ExtrinsicCalibration", "camera grab failed: {}",
                     hal::GrabStatusName(GRAB_STATUS));
        throw std::runtime_error(std::string("camera grab failed: ") +
                                 hal::GrabStatusName(GRAB_STATUS));
      }

      if (!packet.camera_model || !packet.kinematics ||
          !ValidCameraModel(*packet.camera_model, settings_.expected_width,
                            settings_.expected_height) ||
          packet.capture.image.empty() || packet.capture.image.cols != settings_.expected_width ||
          packet.capture.image.rows != settings_.expected_height ||
          packet.capture.image.type() != CV_8UC3 ||
          !packet.capture.stamp.capture_timestamp_ns.has_value() ||
          packet.capture.stamp.capture_timestamp_ns.value() == 0 ||
          !ValidTransform(packet.kinematics->world_t_gimbal) ||
          !ValidTransform(packet.kinematics->gimbal_t_camera_optical)) {
        throw std::runtime_error("frame has invalid image or incomplete spatial metadata");
      }
      if (!samples_.empty() &&
          (packet.capture.stamp.sequence <= samples_.back().frame_sequence ||
           *packet.capture.stamp.capture_timestamp_ns <= samples_.back().capture_timestamp_ns ||
           packet.capture.stamp.simulation_round_id != samples_.back().simulation_round_id)) {
        throw std::runtime_error("frame sequence/time regressed or simulation round changed");
      }

      cv::Mat preview = packet.capture.image.clone();
      DrawText(preview,
               {fmt::format("saved images: {}",
                            std::count_if(samples_.begin(), samples_.end(),
                                          [](const Sample& sample) { return sample.active; })),
                last_message, fmt::format("frame: {}", packet.capture.stamp.sequence),
                "Space/S: save   U: exclude   Q/Esc: finish"});
      window.Show(preview);
      const auto EVENT = window.Poll(1);
      if (EVENT.exit_requested) {
        if (!IsKey(EVENT.key, 'q', 'Q') && EVENT.key != 27)
          throw std::runtime_error("capture window closed");
        break;
      }

      if (EVENT.key == K_SPACE_KEY || IsKey(EVENT.key, 's', 'S')) {
        const auto IMAGE_RELATIVE_PATH =
            std::filesystem::path("images") / fmt::format("sample_{:04d}.png", next_sample_id_);
        const auto IMAGE_PATH = session_dir_ / IMAGE_RELATIVE_PATH;
        Sample sample;
        sample.id = next_sample_id_;
        sample.image_path = IMAGE_RELATIVE_PATH;
        sample.frame_sequence = packet.capture.stamp.sequence;
        sample.capture_timestamp_ns = *packet.capture.stamp.capture_timestamp_ns;
        sample.simulation_round_id = packet.capture.stamp.simulation_round_id;
        sample.camera = *packet.camera_model;
        sample.world_t_gimbal = packet.kinematics->world_t_gimbal;
        sample.gimbal_t_camera_optical = packet.kinematics->gimbal_t_camera_optical;
        samples_.push_back(std::move(sample));
        // 清单为提交点：PNG 成功落盘后才列为有效样本，失败则回滚本次记录。
        try {
          if (!cv::imwrite(IMAGE_PATH.string(), packet.capture.image,
                           {cv::IMWRITE_PNG_COMPRESSION, 3})) {
            throw std::runtime_error("cannot save image: " + IMAGE_PATH.string());
          }
          WriteSession("collecting");
        } catch (...) {
          samples_.pop_back();
          std::error_code cleanup_error;
          std::filesystem::remove(IMAGE_PATH, cleanup_error);
          throw;
        }
        ++next_sample_id_;
        last_message = "saved " + IMAGE_RELATIVE_PATH.filename().string();
        MV_LOG_INFO("ExtrinsicCalibration", "{} (frame={}, timestamp={})", last_message,
                    packet.capture.stamp.sequence, *packet.capture.stamp.capture_timestamp_ns);
      } else if (IsKey(EVENT.key, 'u', 'U')) {
        const auto SAMPLE = std::find_if(samples_.rbegin(), samples_.rend(),
                                         [](const Sample& value) { return value.active; });
        if (SAMPLE != samples_.rend()) {
          const auto SOURCE = session_dir_ / SAMPLE->image_path;
          const auto DESTINATION = session_dir_ / "excluded" / SOURCE.filename();
          std::error_code error;
          std::filesystem::rename(SOURCE, DESTINATION, error);
          if (error) {
            MV_LOG_WARN("ExtrinsicCalibration", "cannot exclude {}: {}", SOURCE.string(),
                        error.message());
          } else {
            SAMPLE->active = false;
            SAMPLE->image_path = std::filesystem::path("excluded") / SOURCE.filename();
            try {
              WriteSession("collecting");
            } catch (...) {
              std::filesystem::rename(DESTINATION, SOURCE);
              SAMPLE->active = true;
              SAMPLE->image_path = std::filesystem::path("images") / SOURCE.filename();
              throw;
            }
            last_message = "excluded " + SOURCE.filename().string();
            MV_LOG_INFO("ExtrinsicCalibration", "{}", last_message);
          }
        }
      }
    }

    WriteSession(g_stop_requested == 0 ? "captured" : "aborted",
                 g_stop_requested == 0 ? "" : "capture interrupted by signal");
    camera.Close();
    MV_LOG_INFO("ExtrinsicCalibration", "session closed: {}", session_dir_.string());
    return g_stop_requested == 0 ? 0 : 6;
  } catch (const std::exception& error) {
    camera.Close();
    try {
      WriteSession("aborted", error.what());
    } catch (const std::exception& write_error) {
      MV_LOG_ERROR("ExtrinsicCalibration", "cannot persist aborted session: {}",
                   write_error.what());
    }
    MV_LOG_ERROR("ExtrinsicCalibration", "capture aborted: {}", error.what());
    return 4;
  }
}

}  // namespace mv::tool::calibration
