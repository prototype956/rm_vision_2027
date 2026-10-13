#pragma once

#include "geometry/rigid_transform.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <yaml-cpp/yaml.h>

namespace mv::hal {
/** @brief USART1 姿态链路配置；角度单位为弧度，时间戳为 MCU 单调时钟微秒数。 */
struct SerialImuConfig {
  bool enabled{true};
  std::string device;
  int baud{460800};
  int startup_sync_ms{100};
  int sync_ms{1000};
  int timeout_ms{3000};
  int cache_ms{2000};
  int max_gap_ms{20};
  int wait_ms{10};
  bool require_calibration_sample{false};  ///< 手眼采集时拒绝普通 ATTITUDE 帧。
};
SerialImuConfig ParseSerialImuConfig(const YAML::Node& root);

/** @brief 线程安全的链路健康快照；有效性由状态字段标识，未知时延和样本年龄以 -1 表示。 */
struct ImuHealth {
  bool connected{false};
  bool synchronized{false};
  std::uint64_t generation{0};
  std::uint64_t crc_errors{0};
  std::uint64_t dropped_packets{0};
  double rate_hz{0};
  double offset_us{0};
  double rtt_us{-1};
  double sample_age_ms{-1};
  double receive_age_ms{-1};
  double attitude_receive_age_ms{-1};
  double sample_progress_age_ms{-1};  ///< Age since a new valid sample was received.
  std::uint32_t packet_sequence{0};
  std::uint32_t sample_sequence{0};
  std::uint64_t duplicate_samples{0};
  geometry::Vector3 gyro_rad_s{geometry::Vector3::Zero()};  ///< 原始 IMU 解算机体系，未应用安装旋转。
  std::string reason{"not_started"};
};

/** @brief 最近一帧通过校验的原始 IMU 状态，时间已映射到上位机时钟。 */
struct ImuObservation {
  geometry::Quaternion world_q_imu{geometry::Quaternion::Identity()};
  geometry::Vector3 gyro_rad_s{geometry::Vector3::Zero()};
  std::chrono::steady_clock::time_point steady_time{};
  std::uint64_t system_timestamp_ns{0};
  std::uint32_t sample_sequence{0};
};

/** @brief 消费串口链路的姿态消息，维护 IMU 缓存并按图像时刻插值，不进行外推。 */
class SerialImu final {
 public:
  explicit SerialImu(SerialImuConfig config);
  ~SerialImu();
  SerialImu(const SerialImu&) = delete;
  SerialImu& operator=(const SerialImu&) = delete;
  [[nodiscard]] std::optional<geometry::Quaternion> At(
      std::chrono::steady_clock::time_point time);
  [[nodiscard]] ImuHealth Health() const;
  [[nodiscard]] std::optional<ImuObservation> Latest() const;

  /** @brief latest-wins 发布双轴速率命令；串口仍由 ControllerLink 单线程独占。 */
  bool QueueGimbalCommand(float yaw_rate_rad_s, float pitch_rate_rad_s,
                          bool enabled, bool fire, bool friction = false) noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace mv::hal
