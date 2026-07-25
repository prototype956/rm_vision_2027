/**
 * @file capture_calib_data.cpp
 * @brief 标定数据采集工具（图像 + 姿态）
 *
 * 【职责边界】
 *   负责按序号保存图像 `i.jpg` 与姿态 `i.txt`（w x y z），
 *   并提供图像与姿态的时间同步采样。
 *
 * 【姿态来源】
 *   1) `pose-source=serial`：实时串口上行帧（默认，推荐）；
 *   2) `pose-source=file`：离线姿态文件（每行 w x y z）；
 *   3) `pose-source=identity`：单位四元数（仅流程联调）。
 *
 * 【交互约定】
 *   - 按 `s` 保存当前帧与对应姿态；
 *   - 按 `q` 退出采集；
 *   - 保存文件采用同一序号，便于后续 rwhandeye 直接按序读取。
 */

#include "hal/serial/rm_protocol.hpp"
#include "hal/serial/uart_serial.hpp"
#include "tool/calibration/calibration_io.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <filesystem>
#include <fmt/core.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

namespace {

enum class PoseSource {
  SERIAL,
  FILE,
  IDENTITY,
};

struct PoseStamped {
  std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::chrono::steady_clock::time_point timestamp;
  uint8_t seq{0};
};

const std::string K_KEYS =
    "{help h usage ? |                     | Print help.}"
    "{source-type    | camera              | camera or video.}"
    "{camera-id      | 0                   | Camera index for source-type=camera.}"
    "{video-path     |                     | Video path for source-type=video.}"
    "{output-folder o| data/calib_capture  | Output folder.}"
    "{pose-source    | serial              | serial / file / identity.}"
    "{quat-file q    |                     | Pose file for pose-source=file, one line: w x y z.}"
    "{serial-config  | src/config/vision.yaml | YAML containing serial node.}"
    "{serial-node    | serial              | Serial node key in YAML.}"
    "{serial-max-age-ms | 30              | Max allowed pose age for frame sync.}"
    "{pattern-type p | chessboard          | chessboard or circles.}"
    "{cols           | 10                  | Pattern columns.}"
    "{rows           | 7                   | Pattern rows.}"
    "{spacing-mm     | 40.0                | Pattern center spacing in mm.}"
    "{show-pattern   | true                | Draw pattern detection result.}";

PoseSource ParsePoseSource(const std::string& text) {
  if (text == "file") {
    return PoseSource::FILE;
  }
  if (text == "identity") {
    return PoseSource::IDENTITY;
  }
  return PoseSource::SERIAL;
}

bool LoadQuaternionFile(const std::string& quat_file_path,
                        std::vector<std::array<double, 4>>* quaternions) {
  if (!quaternions || quat_file_path.empty()) {
    return false;
  }

  std::ifstream fin(quat_file_path);
  if (!fin.is_open()) {
    fmt::print(stderr, "[calib] cannot open quat-file: {}\n", quat_file_path);
    return false;
  }

  std::string line;
  while (std::getline(fin, line)) {
    if (line.empty()) {
      continue;
    }
    std::istringstream iss(line);
    double quat_w = 1.0;
    double quat_x = 0.0;
    double quat_y = 0.0;
    double quat_z = 0.0;
    if (!(iss >> quat_w >> quat_x >> quat_y >> quat_z)) {
      // 姿态文件允许夹杂注释或非法行，解析失败则跳过。
      continue;
    }
    quaternions->push_back({quat_w, quat_x, quat_y, quat_z});
  }

  return !quaternions->empty();
}

int16_t ReadLeI16(const uint8_t* ptr) {
  return static_cast<int16_t>(static_cast<uint16_t>(ptr[0]) |
                              static_cast<uint16_t>(static_cast<uint16_t>(ptr[1]) << 8U));
}

bool TryParseUpFrame(std::deque<uint8_t>* buffer, mv::protocol::UpFrame* frame) {
  if (buffer == nullptr || frame == nullptr) {
    return false;
  }

  while (buffer->size() >= mv::protocol::UP_FRAME_LEN) {
    while (!buffer->empty() && buffer->front() != mv::protocol::FRAME_HEAD_0) {
      buffer->pop_front();
    }
    if (buffer->size() < mv::protocol::UP_FRAME_LEN) {
      return false;
    }

    if ((*buffer)[1] != mv::protocol::UP_HEAD_1 || (*buffer)[2] != mv::protocol::UP_PAYLOAD_LEN ||
        (*buffer)[mv::protocol::UP_FRAME_LEN - 1] != mv::protocol::FRAME_TAIL) {
      buffer->pop_front();
      continue;
    }

    std::array<uint8_t, mv::protocol::UP_FRAME_LEN> raw{};
    std::copy_n(buffer->begin(), raw.size(), raw.begin());

    auto crc_expected =
        static_cast<uint16_t>(static_cast<uint16_t>(raw[25]) |
                              static_cast<uint16_t>(static_cast<uint16_t>(raw[26]) << 8U));
    auto crc_actual = mv::protocol::Crc16Ccitt(raw.data(), mv::protocol::UP_CRC_LEN);
    if (crc_expected != crc_actual) {
      buffer->pop_front();
      continue;
    }

    frame->seq = raw[3];
    frame->color = raw[4];
    frame->mode = static_cast<mv::protocol::UpMode>(raw[5]);
    frame->robot_id = static_cast<mv::protocol::RobotId>(raw[6]);
    frame->bullet_speed = ReadLeI16(&raw[7]);
    frame->q_w = ReadLeI16(&raw[9]);
    frame->q_x = ReadLeI16(&raw[11]);
    frame->q_y = ReadLeI16(&raw[13]);
    frame->q_z = ReadLeI16(&raw[15]);
    frame->yaw = ReadLeI16(&raw[17]);
    frame->pitch = ReadLeI16(&raw[19]);
    frame->yaw_vel = ReadLeI16(&raw[21]);
    frame->pitch_vel = ReadLeI16(&raw[23]);

    buffer->erase(buffer->begin(), buffer->begin() + static_cast<std::ptrdiff_t>(raw.size()));
    return true;
  }

  return false;
}

std::array<double, 4> NormalizeQuaternion(std::array<double, 4> quat_wxyz) {
  auto norm = std::sqrt(quat_wxyz[0] * quat_wxyz[0] + quat_wxyz[1] * quat_wxyz[1] +
                        quat_wxyz[2] * quat_wxyz[2] + quat_wxyz[3] * quat_wxyz[3]);
  if (norm < 1e-9) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  for (double& value : quat_wxyz) {
    value /= norm;
  }
  return quat_wxyz;
}

bool PullSerialPoses(mv::hal::UartSerial* serial, std::deque<uint8_t>* raw_bytes,
                     std::deque<PoseStamped>* pose_history) {
  if (serial == nullptr || raw_bytes == nullptr || pose_history == nullptr || !serial->IsOpen()) {
    return false;
  }

  bool has_update = false;
  std::array<uint8_t, 256> chunk{};
  while (true) {
    size_t received = 0;
    if (!serial->Recv(chunk.data(), chunk.size(), received) || received == 0) {
      break;
    }
    raw_bytes->insert(raw_bytes->end(), chunk.begin(),
                      chunk.begin() + static_cast<std::ptrdiff_t>(received));

    mv::protocol::UpFrame frame{};
    while (TryParseUpFrame(raw_bytes, &frame)) {
      std::array<double, 4> quaternion_wxyz{
          static_cast<double>(frame.q_w) / 10000.0,
          static_cast<double>(frame.q_x) / 10000.0,
          static_cast<double>(frame.q_y) / 10000.0,
          static_cast<double>(frame.q_z) / 10000.0,
      };
      PoseStamped sample{};
      sample.quaternion_wxyz = NormalizeQuaternion(quaternion_wxyz);
      sample.timestamp = std::chrono::steady_clock::now();
      sample.seq = frame.seq;
      pose_history->push_back(sample);
      has_update = true;
    }
  }

  auto now = std::chrono::steady_clock::now();
  while (!pose_history->empty()) {
    auto age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - pose_history->front().timestamp)
            .count();
    if (age_ms <= 2000) {
      break;
    }
    pose_history->pop_front();
  }

  return has_update;
}

bool GetSynchronizedPose(const std::deque<PoseStamped>& pose_history,
                         std::chrono::steady_clock::time_point frame_ts, int max_age_ms,
                         std::array<double, 4>* synchronized_quat, int64_t* out_age_ms) {
  if (synchronized_quat == nullptr || pose_history.empty()) {
    return false;
  }

  int64_t best_abs_age = std::numeric_limits<int64_t>::max();
  size_t best_index = 0;
  for (size_t index = 0; index < pose_history.size(); ++index) {
    auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      frame_ts - pose_history[index].timestamp)
                      .count();
    auto abs_age = std::llabs(age_ms);
    if (abs_age < best_abs_age) {
      best_abs_age = abs_age;
      best_index = index;
      if (best_abs_age == 0) {
        break;
      }
    }
  }

  if (best_abs_age > max_age_ms) {
    return false;
  }

  *synchronized_quat = pose_history[best_index].quaternion_wxyz;
  if (out_age_ms != nullptr) {
    *out_age_ms = best_abs_age;
  }
  return true;
}

}  // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity,readability-function-size)
int main(int argc, char* argv[]) {
  cv::CommandLineParser cli(argc, argv, K_KEYS);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto source_type = cli.get<std::string>("source-type");
  auto camera_id = cli.get<int>("camera-id");
  auto video_path = cli.get<std::string>("video-path");
  auto output_folder = cli.get<std::string>("output-folder");
  auto pose_source = ParsePoseSource(cli.get<std::string>("pose-source"));
  auto quat_file_path = cli.get<std::string>("quat-file");
  auto serial_config_path = cli.get<std::string>("serial-config");
  auto serial_node_key = cli.get<std::string>("serial-node");
  auto serial_max_age_ms = cli.get<int>("serial-max-age-ms");
  auto show_pattern = cli.get<bool>("show-pattern");

  mv::tool::calibration::PatternConfig pattern_cfg;
  pattern_cfg.type = mv::tool::calibration::ParsePatternType(cli.get<std::string>("pattern-type"));
  pattern_cfg.cols = cli.get<int>("cols");
  pattern_cfg.rows = cli.get<int>("rows");
  pattern_cfg.spacing_mm = cli.get<double>("spacing-mm");

  std::vector<std::array<double, 4>> quaternions;
  size_t quaternion_index = 0;
  mv::hal::UartSerial serial;
  std::deque<uint8_t> serial_raw_bytes;
  std::deque<PoseStamped> pose_history;

  if (pose_source == PoseSource::FILE) {
    auto use_quaternion_file = LoadQuaternionFile(quat_file_path, &quaternions);
    if (!use_quaternion_file) {
      fmt::print(stderr, "[calib] pose-source=file but quat-file invalid\n");
      return 1;
    }
    fmt::print("[calib] loaded {} quaternions from {}\n", quaternions.size(), quat_file_path);
  } else if (pose_source == PoseSource::SERIAL) {
    YAML::Node serial_root;
    if (!mv::tool::calibration::LoadVisionYaml(serial_config_path, &serial_root)) {
      return 1;
    }
    auto serial_node = serial_root[serial_node_key];
    if (!serial_node) {
      fmt::print(stderr, "[calib] serial node '{}' not found in {}\n", serial_node_key,
                 serial_config_path);
      return 1;
    }
    if (!serial.Open(serial_node)) {
      fmt::print(stderr, "[calib] failed to open serial pose source\n");
      return 1;
    }
    fmt::print("[calib] serial pose source opened from {}:{}\n", serial_config_path,
               serial_node_key);
  } else {
    fmt::print("[calib] pose-source=identity\n");
  }

  std::filesystem::create_directories(output_folder);

  cv::VideoCapture cap;
  if (source_type == "video") {
    cap.open(video_path);
  } else {
    cap.open(camera_id);
  }

  if (!cap.isOpened()) {
    fmt::print(stderr, "[calib] cannot open source (type={}, camera_id={}, video={})\n",
               source_type, camera_id, video_path);
    return 1;
  }

  fmt::print("[calib] capture started, save='s', quit='q'\n");
  int index = 0;
  while (true) {
    if (pose_source == PoseSource::SERIAL) {
      PullSerialPoses(&serial, &serial_raw_bytes, &pose_history);
    }

    cv::Mat frame;
    cap >> frame;
    auto frame_ts = std::chrono::steady_clock::now();
    if (frame.empty()) {
      break;
    }

    cv::Mat view = frame;
    if (show_pattern) {
      std::vector<cv::Point2f> corners;
      cv::Mat debug;
      auto detect_success =
          mv::tool::calibration::DetectPatternCorners(frame, pattern_cfg, &corners, &debug);
      if (detect_success) {
        view = debug;
      }
    }

    cv::imshow("mv-calib-capture", view);
    auto key = cv::waitKey(1);
    if (key == 'q') {
      break;
    }
    if (key != 's') {
      continue;
    }

    ++index;
    auto image_path = fmt::format("{}/{}.jpg", output_folder, index);
    auto quat_path = fmt::format("{}/{}.txt", output_folder, index);

    cv::imwrite(image_path, frame);

    std::array<double, 4> current_quaternion{1.0, 0.0, 0.0, 0.0};
    int64_t sync_age_ms = -1;
    if (pose_source == PoseSource::FILE) {
      if (quaternion_index >= quaternions.size()) {
        fmt::print(stderr, "[calib] quat-file exhausted at sample {}\n", index);
        return 1;
      }
      current_quaternion = quaternions[quaternion_index++];
    } else if (pose_source == PoseSource::SERIAL) {
      if (!GetSynchronizedPose(pose_history, frame_ts, serial_max_age_ms, &current_quaternion,
                               &sync_age_ms)) {
        fmt::print(stderr,
                   "[calib] no synchronized serial pose (max-age={}ms), sample {} skipped\n",
                   serial_max_age_ms, index);
        --index;
        continue;
      }
    }

    std::ofstream qout(quat_path);
    qout << fmt::format("{} {} {} {}", current_quaternion[0], current_quaternion[1],
                        current_quaternion[2], current_quaternion[3]);
    qout.close();

    if (pose_source == PoseSource::SERIAL) {
      fmt::print("[calib] saved {} + {} (serial sync age={}ms)\n", image_path, quat_path,
                 sync_age_ms);
    } else {
      fmt::print("[calib] saved {} + {}\n", image_path, quat_path);
    }
  }

  serial.Close();

  return 0;
}
