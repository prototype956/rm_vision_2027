#pragma once

#include "core/config.hpp"
#include "hal/serial/attitude_payload.hpp"
#include "runtime/real_geometry_config.hpp"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <string>

#include <filesystem>

namespace mv::test::imu {

/** @brief 采集工具的配置根目录；独立脚本和 CMake 都提供项目根目录。 */
inline std::filesystem::path ConfigRoot() {
  const char* override = std::getenv("RM_VISION_CONFIG_ROOT");
  const std::filesystem::path ROOT = PROJECT_ROOT_PATH;
  return override && override[0] != '\0' ? ConfigLoader::ResolvePath(ROOT, override)
                                         : ROOT / "src/config";
}

/** @brief 冻结有效部署配置，不把本次设备覆盖写回运行配置。 */
inline YAML::Node CaptureMetadata(const runtime::RealGeometryConfig& config,
                                  const std::string& kind) {
  YAML::Node result;
  result["schema_version"] = 1;
  result["kind"] = kind;
  result["profile"] = config.profile;
  result["config_root"] = ConfigRoot().string();
  result["extrinsics"] = YAML::Clone(config.extrinsics_snapshot);
  result["imu_config"] = ConfigLoader::LoadFile(config.imu_config_path);
  result["imu_config"]["device"] = config.imu.device;
  result["device"] = config.imu.device;
  result["baud"] = config.imu.baud;
  result["startup_sync_ms"] = config.imu.startup_sync_ms;
  result["sync_ms"] = config.imu.sync_ms;
  result["timeout_ms"] = config.imu.timeout_ms;
  return result;
}

/** @brief 保存会话清单；失败保留原清单，避免覆盖为半份 YAML。 */
inline void SaveMetadata(const std::filesystem::path& path, const YAML::Node& value) {
  auto temporary = path;
  temporary += ".tmp";
  std::ofstream out(temporary);
  out << value << '\n';
  out.close();
  if (!out)
    throw std::runtime_error("write failed: " + temporary.string());
  std::filesystem::rename(temporary, path);
}

/**
 * @brief 与旧 samples.csv 分离的协议诊断；调用者持采集互斥锁或已结束串口线程。
 *
 * 记录所有 type 5/6 信封，包括无法解码的长度错误；CRC 错帧已在链路层拒收。
 * type 6 机构状态只计数，不用于轴向/漂移样本筛选。
 */
class ProtocolDiagnostics {
 public:
  void Open(const std::filesystem::path& folder) {
    out_.open(folder / "protocol_diagnostics.csv");
    out_ << std::setprecision(17)
         << "generation,host_receive_us,mcu_us,packet_sequence,sample_sequence,message_type,"
            "payload_version,gyro_calibration_status,mechanism_flags,pitch_flags,reason\n";
    if (!out_)
      throw std::runtime_error("cannot open protocol_diagnostics.csv");
  }

  void Record(const hal::serial::ControllerLinkState& state,
              const hal::serial::ControllerFrame& frame,
              const std::optional<hal::serial::AttitudePayload>& payload, const char* reason) {
    ++counts_["message_types"][std::to_string(frame.type)];
    out_ << state.generation << ',' << frame.receive_time_us << ',';
    if (payload)
      out_ << payload->mcu_us;
    out_ << ',' << frame.sequence << ',';
    if (payload)
      out_ << payload->sample_sequence;
    out_ << ',' << static_cast<unsigned>(frame.type) << ',';
    if (payload && payload->calibration) {
      const auto& P = *payload;
      out_ << static_cast<unsigned>(P.version) << ','
           << static_cast<unsigned>(P.gyro_calibration_status) << ','
           << static_cast<unsigned>(P.mechanism_flags) << ','
           << static_cast<unsigned>(P.pitch_flags);
      ++counts_["payload_versions"][std::to_string(P.version)];
      ++counts_["gyro_calibration_status"][std::to_string(P.gyro_calibration_status)];
      ++counts_["mechanism_flags"][std::to_string(P.mechanism_flags)];
      ++counts_["pitch_flags"][std::to_string(P.pitch_flags)];
    } else {
      out_ << ",,,";
    }
    out_ << ',' << reason << '\n';
    if (std::string(reason) != "accepted")
      ++counts_["rejections"][reason];
    if (!out_)
      throw std::runtime_error("protocol diagnostics write failed");
  }

  [[nodiscard]] YAML::Node Summary() const {
    YAML::Node result(YAML::NodeType::Map);
    for (const auto& [category, counts] : counts_)
      for (const auto& [value, count] : counts)
        result[category][value] = count;
    return result;
  }

  void Close() {
    out_.close();
    if (!out_)
      throw std::runtime_error("protocol diagnostics close failed");
  }

 private:
  std::ofstream out_;
  std::map<std::string, std::map<std::string, std::uint64_t>> counts_;
};

}  // namespace mv::test::imu
