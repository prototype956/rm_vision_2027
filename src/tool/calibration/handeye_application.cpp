#include "tool/calibration/handeye_application.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/mindvision/mindvision_camera.hpp"
#include "runtime/real_geometry_config.hpp"
#include "tool/calibration/calibration_io.hpp"
#include "tool/calibration/handeye_calibration.hpp"
#include "tool/debug/debug_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <optional>
#include <set>
#include <utility>

#include <fmt/format.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::tool::calibration {
namespace {
volatile std::sig_atomic_t stop_requested = 0;
void Stop(int) { stop_requested = 1; }
void InstallSignals() {
  stop_requested = 0;
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
}

std::size_t ActiveCount(const YAML::Node& samples) {
  std::size_t count = 0;
  for (const auto& sample : samples) if (sample["active"].as<bool>()) ++count;
  return count;
}

YAML::Node CameraMetadata(const hal::CameraInfo& info) {
  YAML::Node camera;
  camera["device_name"] = info.device_name;
  camera["sensor_width"] = info.sensor_width;
  camera["sensor_height"] = info.sensor_height;
  camera["output_width"] = info.output_width;
  camera["output_height"] = info.output_height;
  camera["roi_offset_x"] = info.roi_offset_x;
  camera["roi_offset_y"] = info.roi_offset_y;
  camera["exposure_us"] = info.exposure_us;
  camera["pixel_format"] = "BGR8";
  return camera;
}

geometry::Quaternion ReadQuaternion(const YAML::Node& sample) {
  const auto Q = sample["world_q_imu_wxyz"].as<std::vector<double>>();
  if (Q.size() != 4) throw ConfigError("sample has no wxyz IMU quaternion");
  geometry::Quaternion q(Q[0], Q[1], Q[2], Q[3]);
  if (!q.coeffs().allFinite() || q.norm() < 0.5 || q.norm() > 1.5) {
    throw ConfigError("invalid saved IMU quaternion");
  }
  return q.normalized();
}

void DrawBoard(cv::Mat& image, const std::vector<cv::Point2f>& corners,
               const CalibrationSettings& settings) {
  const auto EXPECTED = static_cast<std::size_t>(settings.board_columns * settings.board_rows);
  if (corners.size() != EXPECTED) return;
  cv::drawChessboardCorners(image, {settings.board_columns, settings.board_rows}, corners, true);
  cv::circle(image, corners.front(), 9, {0, 0, 255}, 2);
  cv::putText(image, "0", corners.front(), cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2);
  cv::arrowedLine(image, corners.front(), corners[settings.board_columns - 1], {0, 0, 255}, 2);
  const auto Y_ENDPOINT = corners[(settings.board_rows - 1) * settings.board_columns];
  cv::arrowedLine(image, corners.front(), Y_ENDPOINT, {0, 255, 0}, 2);
  cv::putText(image, "X", corners[settings.board_columns - 1], cv::FONT_HERSHEY_SIMPLEX,
              0.7, {0, 0, 255}, 2);
  cv::putText(image, "Y", Y_ENDPOINT,
              cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
}
}  // namespace

int HandeyeApplication::Capture(const std::filesystem::path& config_root,
                                const CalibrationSettings& settings,
                                const YAML::Node& camera_config) {
  InstallSignals();
  const auto DEPLOYMENT = runtime::LoadRealGeometryConfig(config_root);
  if (!DEPLOYMENT.imu.enabled || DEPLOYMENT.imu.device.empty()) {
    throw ConfigError("extrinsics capture requires enabled IMU and configured serial device");
  }
  const auto INTRINSICS_YAML = ConfigLoader::LoadFile(DEPLOYMENT.intrinsics_path);
  const auto INTRINSICS = ParseHandeyeIntrinsics(INTRINSICS_YAML);
  hal::MindVisionCamera camera;
  if (!camera.Open(camera_config)) throw std::runtime_error("MindVision camera open failed");
  const auto INFO = camera.Info();
  if (INFO.output_width != INTRINSICS.size.width || INFO.output_height != INTRINSICS.size.height ||
      INFO.roi_offset_x != INTRINSICS.roi_x || INFO.roi_offset_y != INTRINSICS.roi_y ||
      INFO.pixel_format != hal::PixelFormat::BGR8) {
    throw ConfigError("camera resolution/ROI/pixel format does not match frozen intrinsics");
  }
  auto serial_imu_config = DEPLOYMENT.imu;
  serial_imu_config.require_calibration_sample = true;
  hal::SerialImu imu(std::move(serial_imu_config));
  const auto DIRECTORY = CreateSessionDirectory(settings.output_dir / "extrinsics");
  std::filesystem::create_directories(DIRECTORY / "images");
  YAML::Node session;
  session["schema_version"] = 1;
  session["kind"] = "single_yaw_handeye";
  session["status"] = "collecting";
  session["model"] = "fixed_large_yaw_small_yaw_pitch_intersection";
  session["profile"] = DEPLOYMENT.profile;
  session["camera"] = CameraMetadata(INFO);
  session["camera_config"] = YAML::Clone(camera_config);
  auto imu_config_yaml = session["imu_config"];
  imu_config_yaml["schema_version"] = 1;
  imu_config_yaml["enabled"] = DEPLOYMENT.imu.enabled;
  imu_config_yaml["device"] = DEPLOYMENT.imu.device;
  imu_config_yaml["baud"] = DEPLOYMENT.imu.baud;
  imu_config_yaml["startup_sync_ms"] = DEPLOYMENT.imu.startup_sync_ms;
  imu_config_yaml["sync_ms"] = DEPLOYMENT.imu.sync_ms;
  imu_config_yaml["timeout_ms"] = DEPLOYMENT.imu.timeout_ms;
  imu_config_yaml["cache_ms"] = DEPLOYMENT.imu.cache_ms;
  imu_config_yaml["max_gap_ms"] = DEPLOYMENT.imu.max_gap_ms;
  imu_config_yaml["wait_ms"] = DEPLOYMENT.imu.wait_ms;
  session["intrinsics"] = YAML::Clone(INTRINSICS_YAML);
  session["extrinsics"] = YAML::Clone(DEPLOYMENT.extrinsics_snapshot);
  session["board"]["columns"] = settings.board_columns;
  session["board"]["rows"] = settings.board_rows;
  session["board"]["square_size_mm"] = settings.square_size_mm;
  session["minimum_samples"] = settings.min_samples;
  session["samples"] = YAML::Node(YAML::NodeType::Sequence);
  const auto MANIFEST = DIRECTORY / "session.yaml";
  WriteCalibrationYaml(MANIFEST, session);
  fmt::print("外参采集目录: {}\n固定大 yaw、底盘和标定板，只转小 yaw/pitch，停稳后保存。\n",
             DIRECTORY.string());
  DebugWindow window("Single yaw handeye capture", WindowMode::NORMAL);
  std::optional<std::uint64_t> generation;
  std::optional<std::chrono::steady_clock::time_point> last_saved;
  std::string message = "S/Space: save pair  U: exclude last  Q/Esc: finish";
  bool aborted = false;
  try {
    while (stop_requested == 0) {
      frame::FramePacket packet;
      const auto GRAB = camera.Grab(packet);
      const auto HEALTH = imu.Health();
      if (generation && *generation != HEALTH.generation) {
        message = "IMU session changed: start a new capture session";
        aborted = true; break;
      }
      if (GRAB == hal::GrabStatus::TIMEOUT || GRAB == hal::GrabStatus::INVALID_FRAME) continue;
      if (GRAB != hal::GrabStatus::OK) {
        message = std::string("camera grab failed: ") + hal::GrabStatusName(GRAB);
        aborted = true; break;
      }
      cv::Mat preview = packet.capture.image.clone();
      DrawText(preview, {fmt::format("profile={} saved={} sync={} {:.0f}Hz", DEPLOYMENT.profile,
                                    ActiveCount(session["samples"]), HEALTH.synchronized,
                                    HEALTH.rate_hz), message,
                        "Fix large yaw/base/board; move small yaw + pitch; STOP before save"});
      window.Show(preview);
      const auto EVENT = window.Poll(1);
      if (EVENT.exit_requested) break;
      if (IsKey(EVENT.key, 'u', 'U')) {
        for (std::size_t i = session["samples"].size(); i > 0; --i) {
          auto sample = session["samples"][i - 1];
          if (!sample["active"].as<bool>()) continue;
          sample["active"] = false;
          WriteCalibrationYaml(MANIFEST, session);
          message = "excluded pair " + sample["id"].as<std::string>();
          break;
        }
      }
      if (EVENT.key != 32 && !IsKey(EVENT.key, 's', 'S')) continue;
      const auto TIME = packet.capture.stamp.capture_steady_time;
      const auto BEFORE = imu.Health();
      if (!TIME || !BEFORE.synchronized || (last_saved && *TIME <= *last_saved)) {
        message = "save rejected: image timestamp/IMU sync not ready";
        continue;
      }
      const auto Q = imu.At(*TIME);
      const auto AFTER = imu.Health();
      if (generation && *generation != AFTER.generation) {
        message = "IMU session changed: start a new capture session";
        aborted = true; break;
      }
      if (!Q || !AFTER.synchronized || BEFORE.generation != AFTER.generation) {
        message = "save rejected: " + AFTER.reason;
        continue;
      }
      if (packet.capture.image.size() != INTRINSICS.size) {
        throw std::runtime_error("frame size changed during capture");
      }
      const std::size_t ID = session["samples"].size() + 1;
      const auto RELATIVE_IMAGE = std::filesystem::path("images") /
                                  fmt::format("sample_{:04d}.png", ID);
      SaveCalibrationImage(DIRECTORY / RELATIVE_IMAGE, packet.capture.image);
      YAML::Node sample;
      sample["id"] = ID;
      sample["active"] = true;
      sample["image"] = RELATIVE_IMAGE.generic_string();
      sample["capture_steady_us"] = std::chrono::duration_cast<std::chrono::microseconds>(
          TIME->time_since_epoch()).count();
      sample["world_q_imu_wxyz"] = std::vector<double>{Q->w(), Q->x(), Q->y(), Q->z()};
      sample["imu_generation"] = AFTER.generation;
      sample["synchronized"] = AFTER.synchronized;
      sample["host_minus_mcu_us"] = AFTER.offset_us;
      sample["rtt_us"] = AFTER.rtt_us;
      sample["sample_age_ms"] = AFTER.sample_age_ms;
      sample["crc_errors"] = AFTER.crc_errors;
      sample["dropped_packets"] = AFTER.dropped_packets;
      session["samples"].push_back(sample);
      session["imu_generation"] = AFTER.generation;
      WriteCalibrationYaml(MANIFEST, session);
      generation = AFTER.generation;
      last_saved = TIME;
      message = fmt::format("saved pair {} (stop before next save)", ID);
      MV_LOG_INFO("Handeye", "{}", message);
    }
  } catch (const std::exception& error) {
    session["status"] = "aborted";
    session["failure"] = error.what();
    WriteCalibrationYaml(MANIFEST, session);
    throw;
  }
  session["status"] = aborted ? "aborted" : "captured";
  if (aborted) session["failure"] = message;
  WriteCalibrationYaml(MANIFEST, session);
  fmt::print("{}: {}\n", aborted ? "采集中止" : "采集结束", DIRECTORY.string());
  if (aborted) fmt::print("{}\n", message);
  return aborted ? 4 : 0;
}

int HandeyeApplication::SolveOffline(const std::filesystem::path& session_dir) {
  InstallSignals();
  const auto SESSION = ConfigLoader::LoadFile(session_dir / "session.yaml", 1);
  if (!SESSION["kind"] || SESSION["kind"].as<std::string>() != "single_yaw_handeye") {
    throw ConfigError("not an extrinsics session: intrinsic-only images have no paired IMU poses");
  }
  if (SESSION["model"].as<std::string>() != "fixed_large_yaw_small_yaw_pitch_intersection") {
    throw ConfigError("unsupported handeye kinematic model");
  }
  const auto INTRINSICS = ParseHandeyeIntrinsics(SESSION["intrinsics"]);
  const auto EXTRINSICS = runtime::ParseGimbalExtrinsics(SESSION["extrinsics"]);
  const auto CAMERA = SESSION["camera"];
  if (CAMERA["output_width"].as<int>() != INTRINSICS.size.width ||
      CAMERA["output_height"].as<int>() != INTRINSICS.size.height ||
      CAMERA["roi_offset_x"].as<int>() != INTRINSICS.roi_x ||
      CAMERA["roi_offset_y"].as<int>() != INTRINSICS.roi_y ||
      CAMERA["pixel_format"].as<std::string>() != "BGR8") {
    throw ConfigError("saved camera metadata does not match intrinsics");
  }
  CalibrationSettings settings;
  settings.board_columns = SESSION["board"]["columns"].as<int>();
  settings.board_rows = SESSION["board"]["rows"].as<int>();
  settings.square_size_mm = SESSION["board"]["square_size_mm"].as<double>();
  settings.min_samples = SESSION["minimum_samples"].as<int>();
  HandeyeCalibrator calibrator(settings, INTRINSICS, EXTRINSICS);
  CameraCalibrator detector(settings, INTRINSICS.size);
  const auto SAMPLES = SESSION["samples"];
  if (!SAMPLES.IsSequence()) throw ConfigError("missing image-pose sample manifest");
  // 验证完整配对索引后再打开审核窗口，不以扫描 images/ 的方式猜测关联。
  std::set<std::size_t> ids;
  std::set<std::string> images;
  std::optional<std::int64_t> last_time;
  for (const auto& sample : SAMPLES) {
    const auto ID = sample["id"].as<std::size_t>();
    const auto IMAGE = sample["image"].as<std::string>();
    const std::filesystem::path RELATIVE(IMAGE);
    if (ID == 0 || !ids.insert(ID).second || !images.insert(IMAGE).second ||
        RELATIVE.parent_path() != "images" || RELATIVE.filename() == "." ||
        RELATIVE.filename() == ".." || RELATIVE.extension() != ".png") {
      throw ConfigError("invalid or duplicate sample id/image path");
    }
    (void)sample["active"].as<bool>();
    (void)ReadQuaternion(sample);
    const auto TIME = sample["capture_steady_us"].as<std::int64_t>();
    if (!sample["synchronized"].as<bool>() || TIME < 0 || (last_time && TIME <= *last_time) ||
        sample["imu_generation"].as<std::uint64_t>() !=
            SESSION["imu_generation"].as<std::uint64_t>()) {
      throw ConfigError("sample timestamps or IMU generations are inconsistent");
    }
    last_time = TIME;
  }
  const auto RESULT_DIR = CreateSessionDirectory(session_dir / "solutions");
  std::filesystem::create_directories(RESULT_DIR / "review");
  YAML::Node review;
  review["schema_version"] = 1;
  review["source_session"] = std::filesystem::absolute(session_dir).string();
  review["source_snapshot"] = YAML::Clone(SESSION);
  review["status"] = "reviewing";
  review["samples"] = YAML::Node(YAML::NodeType::Sequence);
  const auto REVIEW_PATH = RESULT_DIR / "review.yaml";
  WriteCalibrationYaml(REVIEW_PATH, review);
  fmt::print("离线审核结果目录: {}\n保持 0 号角点对应同一物理角，F 翻转，S/空格接纳，X 排除。\n",
             RESULT_DIR.string());
  DebugWindow window("Handeye review: S accept / X exclude / F reverse / Q cancel",
                     WindowMode::NORMAL);
  try {
    for (const auto& sample : SAMPLES) {
      if (!sample["active"].as<bool>()) continue;
      const auto ID = sample["id"].as<std::size_t>();
      const auto IMAGE_PATH = session_dir / sample["image"].as<std::string>();
      const auto IMAGE = cv::imread(IMAGE_PATH.string());
      if (IMAGE.empty() || IMAGE.size() != INTRINSICS.size) {
        throw std::runtime_error("unreadable image or resolution mismatch for sample " +
                                 std::to_string(ID));
      }
      auto observation = detector.Observe(IMAGE);
      bool reversed = false, done = false, accepted = false;
      std::string message =
          observation.found ? "Check origin 0 and X/Y directions" : "Board not found";
      YAML::Node item;
      item["id"] = ID;
      item["image"] = sample["image"].as<std::string>();
      while (!done) {
        cv::Mat preview = IMAGE.clone();
        DrawBoard(preview, observation.corners, settings);
        DrawText(preview,
                 {fmt::format("sample {} accepted {}", ID, calibrator.Observations().size()),
                  message, "S/Space accept | X exclude | F reverse | Q/Esc cancel"});
        window.Show(preview);
        const auto EVENT = window.Poll(30);
        if (stop_requested != 0 || EVENT.exit_requested) {
          review["status"] = "aborted";
          WriteCalibrationYaml(REVIEW_PATH, review);
          return 6;
        }
        if (IsKey(EVENT.key, 'f', 'F') && observation.found) {
          std::reverse(observation.corners.begin(), observation.corners.end());
          reversed = !reversed;
        }
        if (IsKey(EVENT.key, 'x', 'X')) {
          item["reason"] = observation.found ? "user_excluded" : "board_not_found";
          done = true;
        }
        if (EVENT.key == 32 || IsKey(EVENT.key, 's', 'S')) {
          std::string failure;
          if (calibrator.Add(ID, observation.corners, ReadQuaternion(sample), failure)) {
            accepted = true;
            done = true;
            message = fmt::format("PnP RMS {:.3f}px (not precision acceptance)",
                                 calibrator.Observations().back().reprojection_rms_px);
          } else {
            item["pnp_failure"] = failure;
            message = "PnP rejected; X exclude: " + failure;
          }
        }
        if (done) {
          DrawText(preview, {accepted ? "Accepted: " + message : "Excluded"});
          SaveCalibrationImage(
              RESULT_DIR / "review" / fmt::format("sample_{:04d}.png", ID), preview);
        }
      }
      item["accepted"] = accepted;
      item["reversed"] = reversed;
      item["corners"] = YAML::Node(YAML::NodeType::Sequence);
      for (const auto& point : observation.corners) {
        item["corners"].push_back(std::vector<float>{point.x, point.y});
      }
      review["samples"].push_back(item);
      WriteCalibrationYaml(REVIEW_PATH, review);
    }
    window.Close();
    if (stop_requested != 0) {
      review["status"] = "aborted";
      WriteCalibrationYaml(REVIEW_PATH, review);
      return 6;
    }
    const auto RESULT = calibrator.Solve();
    WriteCalibrationYaml(RESULT_DIR / "report.yaml", RESULT.report);
    review["status"] = RESULT.solved ? "solved_requires_manual_validation" : "failed";
    WriteCalibrationYaml(REVIEW_PATH, review);
    if (!RESULT.solved) {
      fmt::print("外参求解失败: {}，详见 {}\n", RESULT.report["failure"].as<std::string>(),
                 (RESULT_DIR / "report.yaml").string());
      return 5;
    }
    auto candidate = YAML::Clone(SESSION["extrinsics"]);
    candidate["gimbal_r_camera"] = HandeyeMatrixValues(RESULT.gimbal_r_camera);
    candidate["gimbal_t_camera_m"] = HandeyeMatrixValues(RESULT.gimbal_t_camera);
    WriteCalibrationYaml(RESULT_DIR / "extrinsics_candidate.yaml", candidate);
    fmt::print("数值求解成功，待用户实机确认；未修改部署配置。\n"
               "重投影 RMS: {:.3f}px，棋盘位置 RMS: {:.4f}m，姿态 RMS: {:.3f}deg\n结果: {}\n",
               RESULT.report["reprojection_rms_px"].as<double>(),
               RESULT.report["position_rms_m"].as<double>(),
               RESULT.report["rotation_rms_deg"].as<double>(), RESULT_DIR.string());
    return 0;
  } catch (const std::exception& error) {
    review["status"] = "aborted";
    review["failure"] = error.what();
    WriteCalibrationYaml(REVIEW_PATH, review);
    throw;
  }
}
}  // namespace mv::tool::calibration
