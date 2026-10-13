#pragma once

#include "hal/serial/attitude_wire.h"
#include "hal/serial/controller_link.hpp"

#include <array>
#include <cmath>

#include <optional>

namespace mv::hal::serial {

/** @brief 未归一化的线协议姿态：wxyz（IMU -> world），角速度为 IMU 系 rad/s。 */
struct AttitudePayload {
  std::uint64_t mcu_us{0};
  std::uint32_t sample_sequence{0};
  std::uint16_t status{0};
  std::array<double, 7> values{};
  bool calibration{false};
  std::uint8_t version{0}, gyro_calibration_status{0}, mechanism_flags{0}, pitch_flags{0};
};

/** @brief 只解码已校验信封中长度正确的 type 5/6；保留非法数值供诊断记录。 */
inline std::optional<AttitudePayload> DecodeAttitudePayload(const ControllerFrame& frame) {
  const bool CALIBRATION = frame.type == AV_CALIBRATION_SAMPLE;
  if ((!CALIBRATION && frame.type != AV_ATTITUDE) ||
      frame.payload.size() != (CALIBRATION ? AV_CALIBRATION_PAYLOAD : AV_ATTITUDE_PAYLOAD))
    return std::nullopt;
  const auto* p = frame.payload.data();
  AttitudePayload result;
  result.mcu_us = AvRead(p, 8);
  result.sample_sequence = static_cast<std::uint32_t>(AvRead(p + 8, 4));
  result.status = static_cast<std::uint16_t>(AvRead(p + 12, 2));
  for (unsigned i = 0; i < result.values.size(); ++i)
    result.values[i] = AvFloat(p + 14 + 4 * i);
  result.calibration = CALIBRATION;
  if (CALIBRATION) {
    result.version = p[AV_CAL_PAYLOAD_VERSION_OFF];
    result.gyro_calibration_status = p[AV_CAL_GYRO_CAL_OFF];
    result.mechanism_flags = p[AV_CAL_FLAGS_OFF];
    result.pitch_flags = p[AV_CAL_PITCH_FLAGS_OFF];
  }
  return result;
}

/** @brief type 6 结构检查，不将机构就绪或陀螺标定状态作为 IMU 诊断门槛。 */
inline const char* CalibrationStructureFailure(const ControllerFrame& frame) {
  if (frame.type != AV_CALIBRATION_SAMPLE)
    return nullptr;
  if (frame.payload.size() != AV_CALIBRATION_PAYLOAD)
    return "invalid_payload_length";
  const auto* p = frame.payload.data();
  if (p[AV_CAL_PAYLOAD_VERSION_OFF] != AV_CAL_PAYLOAD_VERSION)
    return "unsupported_calibration_payload";
  const auto IS_DIRECTION = [](std::uint8_t v) { return v == 0U || v == 1U || v == 0xffU; };
  if (AvRead(p + AV_CAL_RESERVED_OFF, 2) != 0U || !IS_DIRECTION(p[AV_CAL_SMALL_DIRECTION_OFF]) ||
      !IS_DIRECTION(p[AV_CAL_PITCH_DIRECTION_OFF]))
    return "invalid_calibration_payload";
  constexpr unsigned OFFSETS[] = {50U, 54U, 58U, 62U, 72U, 76U, 80U, 84U, 88U};
  for (const unsigned OFFSET : OFFSETS)
    if (!std::isfinite(AvFloat(p + OFFSET)))
      return "invalid_calibration_payload";
  return nullptr;
}

/** @brief 运行时/手眼使用的严格机构门槛；诊断采集仅记录这些状态。 */
inline const char* CalibrationMechanismFailure(const AttitudePayload& payload) {
  if (payload.gyro_calibration_status != 2U)
    return "gyro_calibration_not_valid";
  constexpr std::array<const char*, 8> REASONS = {
      "small_yaw_offline",          "small_yaw_feedback_stale",
      "small_yaw_angle_invalid",    nullptr,
      "small_yaw_baseline_invalid", "large_yaw_not_disabled",
      "pitch_extension_missing",    "calibration_safety_not_active"};
  for (unsigned i = 0; i < REASONS.size(); ++i)
    if (REASONS[i] && (payload.mechanism_flags & (1U << i)) == 0U)
      return REASONS[i];
  constexpr std::array<const char*, 5> PITCH_REASONS = {
      "pitch_offline", "pitch_position_invalid", "pitch_rate_invalid", "pitch_baseline_invalid",
      "pitch_limit_not_initialized"};
  for (unsigned i = 0; i < PITCH_REASONS.size(); ++i)
    if ((payload.pitch_flags & (1U << i)) == 0U)
      return PITCH_REASONS[i];
  return nullptr;
}

/** @brief 原始姿态有效性；type 5 要求 READY，type 6 同时要求 READY/FINITE。 */
inline const char* AttitudeValueFailure(const AttitudePayload& payload) {
  const unsigned REQUIRED = payload.calibration ? 3U : 1U;
  if ((payload.status & REQUIRED) != REQUIRED)
    return "invalid_flag";
  double norm_squared = 0;
  for (unsigned i = 0; i < payload.values.size(); ++i) {
    if (!std::isfinite(payload.values[i]))
      return "nonfinite";
    if (i < 4)
      norm_squared += payload.values[i] * payload.values[i];
  }
  if (norm_squared < 0.25 || norm_squared > 2.25)
    return "quaternion_norm";
  return nullptr;
}

}  // namespace mv::hal::serial
