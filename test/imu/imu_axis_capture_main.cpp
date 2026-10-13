#include "hal/serial/controller_link.hpp"
#include "imu_capture_support.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

#include <filesystem>
#include <poll.h>
#include <unistd.h>

namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stopped = 0;
void Stop(int) {
  stopped = 1;
}

/** @brief 按回车切换阶段；pause 用于复位，静止与运动阶段均由操作者明确标记。 */
std::string Phase(const std::string& command) {
  if (command == "s")
    return "still";
  if (command == "l")
    return "yaw_left";
  if (command == "r")
    return "yaw_right";
  if (command == "u")
    return "pitch_up";
  if (command == "d")
    return "pitch_down";
  if (command == "p")
    return "pause";
  return {};
}

/** @brief 收发线程逐帧记录，不重采样；mutex 同时保护文件、标签和状态显示快照。 */
struct Capture {
  std::mutex mutex;
  std::ofstream output;
  mv::test::imu::ProtocolDiagnostics diagnostics;
  bool failed{false};
  std::string phase{"pause"};
  unsigned segment{0};
  mv::hal::serial::ControllerLinkState health;
  std::uint64_t rows{0}, accepted{0}, last_mcu{0}, generation{0};
  std::uint32_t last_sample{0}, last_packet{0};
  bool have_sample{false};
  std::array<double, 3> gyro{};
  Clock::time_point last_valid{};

  bool UpdateImpl(const mv::hal::serial::ControllerLinkState& state,
                  const std::optional<mv::hal::serial::ControllerFrame>& frame) {
    std::lock_guard lock(mutex);
    health = state;
    if (generation != state.generation) {
      generation = state.generation;
      have_sample = false;
      // 断线/会话重建后需要操作者重新标记，避免一次实验跨越两个会话。
      phase = "pause";
      ++segment;
    }
    if (!frame || (frame->type != AV_ATTITUDE && frame->type != AV_CALIBRATION_SAMPLE))
      return true;
    const auto PAYLOAD = mv::hal::serial::DecodeAttitudePayload(*frame);
    if (!PAYLOAD) {
      diagnostics.Record(state, *frame, PAYLOAD, "invalid_payload_length");
      return true;
    }
    const auto MCU = PAYLOAD->mcu_us;
    const auto SAMPLE = PAYLOAD->sample_sequence;
    const auto FLAGS = PAYLOAD->status;
    const auto& VALUES = PAYLOAD->values;
    const char* failure = mv::hal::serial::CalibrationStructureFailure(*frame);
    if (!failure)
      failure = mv::hal::serial::AttitudeValueFailure(*PAYLOAD);
    const bool ROLLBACK =
        have_sample && (MCU < last_mcu || SAMPLE < last_sample || frame->sequence < last_packet);
    const bool FRESH =
        !have_sample || (MCU > last_mcu && SAMPLE > last_sample && frame->sequence > last_packet);
    const bool VALID = failure == nullptr;
    const bool ACCEPT = VALID && FRESH && !ROLLBACK && state.synchronized;
    diagnostics.Record(state, *frame, PAYLOAD,
                       ROLLBACK              ? "rollback"
                       : !FRESH              ? "duplicate"
                       : failure             ? failure
                       : !state.synchronized ? "unsynchronized"
                                             : "accepted");
    output << phase << ',' << segment << ',' << state.generation << ',' << frame->receive_time_us
           << ',' << MCU << ',' << frame->sequence << ',' << SAMPLE << ',' << FLAGS << ','
           << state.synchronized << ',' << ACCEPT;
    for (const double VALUE : VALUES)
      output << ',' << VALUE;
    output << ',' << state.offset_us << ',' << state.rtt_us << ',' << state.crc_errors << '\n';
    ++rows;
    if (VALID && FRESH && !ROLLBACK) {
      last_mcu = MCU;
      last_sample = SAMPLE;
      last_packet = frame->sequence;
      have_sample = true;
    }
    if (ACCEPT) {
      ++accepted;
      gyro = {VALUES[4], VALUES[5], VALUES[6]};
      last_valid = Clock::now();
    }
    if (!output)
      throw std::runtime_error("samples.csv write failed");
    return !ROLLBACK;
  }

  /** @brief 文件错误不逃逸到串口线程，主线程负责封口并报告失败。 */
  bool Update(const mv::hal::serial::ControllerLinkState& state,
              const std::optional<mv::hal::serial::ControllerFrame>& frame) noexcept {
    try {
      return UpdateImpl(state, frame);
    } catch (...) {
      std::lock_guard lock(mutex);
      failed = true;
      return false;
    }
  }
};

int Run(const std::string& device, const std::filesystem::path& folder) {
  auto config = mv::runtime::LoadRealGeometryConfig(mv::test::imu::ConfigRoot());
  if (!device.empty())
    config.imu.device = device;
  if (!config.imu.enabled || config.imu.device.empty())
    throw std::runtime_error("selected profile IMU is disabled or has no device");
  auto session = mv::test::imu::CaptureMetadata(config, "imu_axis_capture");
  session["status"] = "capturing";
  // 原子创建新目录，拒绝覆盖先前实验。目录可以由命令行指定，默认位于 artifacts。
  if (!folder.parent_path().empty())
    std::filesystem::create_directories(folder.parent_path());
  if (!std::filesystem::create_directory(folder))
    throw std::runtime_error("output directory already exists: " + folder.string());
  mv::test::imu::SaveMetadata(folder / "session.yaml", session);
  Capture capture;
  capture.diagnostics.Open(folder);
  capture.output.open(folder / "samples.csv");
  if (!capture.output)
    throw std::runtime_error("cannot open samples.csv");
  capture.output << std::setprecision(17)
                 << "phase,segment,generation,host_receive_us,mcu_us,packet_sequence,"
                    "sample_sequence,flags,synchronized,accepted,qw,qx,qy,qz,wx,wy,wz,"
                    "host_minus_mcu_us,rtt_us,crc_errors\n";
  std::ofstream metadata(folder / "README.txt");
  metadata << "device=" << config.imu.device << "\nprotocol=attitude_v1\nbaud=" << config.imu.baud
           << "\n"
           << "gyro: IMU body rad/s; quaternion: IMU body -> inertial, wxyz\n"
           << "gimbal: X forward, Y left, Z up\n"
           << "yaw_left/right: level pitch, viewed from above\n"
           << "pitch_up/down: raise/lower muzzle, yaw held fixed\n"
           << "accepted: synchronized, valid, strictly increasing sample and packet\n"
           << "phase marks host receipt time, not MCU sample time; discard transition margins\n";
  metadata.close();
  if (!metadata)
    throw std::runtime_error("cannot write README.txt");

  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  const auto START = Clock::now();
  auto next_status = START;
  bool failed = false;
  std::cout << "profile=" << config.profile << " device=" << config.imu.device
            << " baud=" << config.imu.baud << "\n输出: " << folder
            << "\n请先停止占用同一串口的视觉主程序。\n"
            << "等待 ready=yes 后输入字母并回车：\n"
            << "s 静止 | l 左转 | r 右转 | u 抬头 | d 低头 | p 暂停/复位 | q 保存退出\n"
            << "yaw 实验保持 pitch 水平；pitch 实验固定 yaw。每次单向转动，复位前切 p。\n"
            << "最多采集 600 秒。工具只发送握手/同步消息。\n";
  {
    mv::hal::serial::ControllerLink link(
        {.enabled = true,
         .device = config.imu.device,
         .baud = config.imu.baud,
         .startup_sync_ms = config.imu.startup_sync_ms,
         .sync_ms = config.imu.sync_ms,
         .timeout_ms = config.imu.timeout_ms},
        [&capture](const auto& state, const auto& frame) { return capture.Update(state, frame); });
    while (stopped == 0 && Clock::now() - START < std::chrono::seconds(600)) {
      pollfd input{STDIN_FILENO, POLLIN, 0};
      const int READY = poll(&input, 1, 100);
      if (READY > 0 && (input.revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) != 0) {
        std::string command;
        if (!std::getline(std::cin, command) || command == "q")
          break;
        const auto PHASE = Phase(command);
        if (PHASE.empty()) {
          std::cout << "未知命令，请输入 s/l/r/u/d/p/q 并回车。\n";
        } else {
          std::lock_guard lock(capture.mutex);
          const bool FRESH = capture.health.synchronized &&
                             Clock::now() - capture.last_valid < std::chrono::milliseconds(200);
          if (PHASE != "pause" && !FRESH) {
            std::cout << "姿态未就绪，保持 pause；请等待 ready=yes。\n";
          } else {
            capture.phase = PHASE;
            ++capture.segment;
            std::cout << "阶段: " << PHASE << "，段号: " << capture.segment << '\n';
          }
        }
      }
      if (Clock::now() >= next_status) {
        std::lock_guard lock(capture.mutex);
        capture.output.flush();
        if (!capture.output || capture.failed) {
          failed = true;
          break;
        }
        const bool FRESH = capture.health.synchronized &&
                           Clock::now() - capture.last_valid < std::chrono::milliseconds(200);
        std::cout << "ready=" << (FRESH ? "yes" : "no") << " phase=" << capture.phase
                  << " rows=" << capture.rows << " accepted=" << capture.accepted
                  << " Hz=" << capture.health.rate_hz << " gyro=[" << capture.gyro[0] << ','
                  << capture.gyro[1] << ',' << capture.gyro[2] << "] link=" << capture.health.reason
                  << std::endl;
        next_status = Clock::now() + std::chrono::seconds(1);
      }
    }
  }  // 先停止串口线程，再关闭文件，确保最后一帧落盘。
  capture.output.close();
  session["status"] = failed || capture.failed ? "write_or_callback_error" : "finished";
  session["rows"] = capture.rows;
  session["accepted"] = capture.accepted;
  session["final_link_reason"] = capture.health.reason;
  session["final_crc_errors"] = capture.health.crc_errors;
  session["protocol_diagnostics"] = capture.diagnostics.Summary();
  mv::test::imu::SaveMetadata(folder / "session.yaml", session);
  capture.diagnostics.Close();
  if (failed || capture.failed || !capture.output)
    throw std::runtime_error("samples.csv write failed");
  std::cout << "已保存 " << capture.rows << " 帧，accepted=" << capture.accepted
            << "；文件: " << folder / "samples.csv" << '\n';
  return capture.accepted == 0 ? 2 : 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "用法: mv-imu-axis-capture [DEVICE [NEW_OUTPUT_DIRECTORY]]\n"
              << "例: mv-imu-axis-capture /dev/ttyUSB0\n";
    return 0;
  }
  if (argc > 3) {
    std::cerr << "用法: mv-imu-axis-capture [DEVICE [NEW_OUTPUT_DIRECTORY]]\n";
    return 1;
  }
  try {
    const auto ID = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
    const std::filesystem::path ROOT = PROJECT_ROOT_PATH;
    const auto FOLDER = argc == 3 ? mv::ConfigLoader::ResolvePath(ROOT, argv[2])
                                  : ROOT / "artifacts/imu_axis" / std::to_string(ID);
    return Run(argc >= 2 ? argv[1] : "", FOLDER);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
