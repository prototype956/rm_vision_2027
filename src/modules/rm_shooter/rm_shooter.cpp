/**
 * @file rm_shooter.cpp
 * @brief RoboMaster 协议串口发射器实现（正式协议 v1.0）
 *
 * 【下行帧格式】总 15 字节，详见 src/hal/serial/rm_protocol.hpp
 *   [0]  0xAA  帧头1
 *   [1]  0x0F  帧头２（下行标识）
 *   [2]  0x0A  payload 长度（10 字节）
 *   [3]  seq   u8   序列号 0~255 循环
 *   [4]  det   u8   0=未识别, 1=已识别
 *   [5]  fire  u8   脉冲: 1=本帧开火, 0=不开火
 *   [6-7] yaw  i16 LE  预测射击角 yaw   × 10000 [rad]
 *   [8-9] pitch i16 LE  预测射击角 pitch × 10000 [rad]
 *   [10-11] dist u16 LE  目标距离 × 100 [m]
 *   [12-13] crc16 u16 LE  CRC16-CCITT（覆盖 [0]~[11]）
 *   [14] 0x0D  帧尾
 */
#include "rm_shooter.hpp"

#include "core/logger.hpp"
#include "factory/factory.hpp"
#include "hal/serial/i_serial.hpp"
#include "hal/serial/rm_protocol.hpp"

#include <array>
#include <cmath>
#include <cstring>

namespace mv::modules {

namespace {
const bool RM_SHOOTER_REGISTERED = [] {
  ::mv::Factory<::mv::IShooter>::Register("rm", [] { return std::make_unique<RmShooter>(); });
  return true;
}();
}  // namespace

RmShooter::RmShooter() = default;
RmShooter::~RmShooter() = default;

// ── Init ──────────────────────────────────────────────────────────────────

bool RmShooter::Init(const YAML::Node& config) {
  if (config && config["auto_aim"] && config["auto_aim"]["shooter"]) {
    const auto& shooter = config["auto_aim"]["shooter"];
    if (shooter["enable_ballistic_compensation"]) {
      ballistic_comp_ = shooter["enable_ballistic_compensation"].as<bool>();
    }
    if (shooter["bullet_speed"]) {
      bullet_speed_ = shooter["bullet_speed"].as<float>();
    }
    if (shooter["gravity"]) {
      gravity_ = shooter["gravity"].as<float>();
    }
  }
  initialized_ = true;
  MV_LOG_INFO("RmShooter", "Init OK — ballistic_comp={} v0={:.1f}m/s g={:.2f}m/s²", ballistic_comp_,
              bullet_speed_, gravity_);
  return true;
}

// ── Send ──────────────────────────────────────────────────────────────────

bool RmShooter::Send(hal::ISerial& serial, const GimbalControl& control) {
  if (!serial.IsOpen()) {
    MV_LOG_DEBUG("RmShooter", "Serial not open, skipping send");
    return false;
  }

  // 弹道补偿（可选）
  double pitch_out = control.pitch;
  if (ballistic_comp_ && control.tracking && control.distance > 0.01) {
    pitch_out = BallisticCompensation(control.pitch, control.distance);
  }

  // 角度转换：rad → × 10000 的 int16_t（Little-Endian）
  static constexpr double ANGLE_SCALE = 10000.0;
  const auto YAW_I16 = static_cast<int16_t>(control.yaw * ANGLE_SCALE);
  const auto PITCH_I16 = static_cast<int16_t>(pitch_out * ANGLE_SCALE);

  // 距离转换：m → × 100 的 uint16_t，超范围时饱和到 UINT16_MAX
  static constexpr double DIST_SCALE = 100.0;
  static constexpr double DIST_MAX = 655.35;  // 65535 / 100
  const double DIST_CLAMPED = (control.distance < DIST_MAX) ? control.distance : DIST_MAX;
  const auto DIST_U16 = static_cast<uint16_t>(DIST_CLAMPED * DIST_SCALE);

  // 开火脉冲：GimbalControl::fire == true 时发 1，否则 0
  const uint8_t FIRE_U8 = control.fire ? 1U : 0U;

  // 组帧：15 字节，详见文件头注释
  //
  //  [0]    0xAA  帧头1
  //  [1]    0x0F  帧头2（下行标识）
  //  [2]    0x0A  payload 长度
  //  [3]    seq   序列号
  //  [4]    det   识别状态
  //  [5]    fire  脉冲
  //  [6-7]  yaw   i16 LE
  //  [8-9]  pitch i16 LE
  //  [10-11] dist  u16 LE
  //  [12-13] crc16 u16 LE（覆盖[0]~[11]）
  //  [14]   0x0D  帧尾
  frame_buf_.fill(0);
  frame_buf_[0] = protocol::FRAME_HEAD_0;
  frame_buf_[1] = protocol::DOWN_HEAD_1;
  frame_buf_[2] = protocol::DOWN_PAYLOAD_LEN;
  frame_buf_[3] = seq_++;
  frame_buf_[4] = control.tracking ? 1U : 0U;
  frame_buf_[5] = FIRE_U8;
  frame_buf_[6] = static_cast<uint8_t>(YAW_I16 & 0xFF);
  frame_buf_[7] = static_cast<uint8_t>((YAW_I16 >> 8) & 0xFF);
  frame_buf_[8] = static_cast<uint8_t>(PITCH_I16 & 0xFF);
  frame_buf_[9] = static_cast<uint8_t>((PITCH_I16 >> 8) & 0xFF);
  frame_buf_[10] = static_cast<uint8_t>(DIST_U16 & 0xFF);
  frame_buf_[11] = static_cast<uint8_t>((DIST_U16 >> 8) & 0xFF);

  // CRC16-CCITT 覆盖 [0]~[11]（DOWN_CRC_LEN = 12）
  const auto CRC = protocol::Crc16Ccitt(frame_buf_.data(), protocol::DOWN_CRC_LEN);
  frame_buf_[12] = static_cast<uint8_t>(CRC & 0xFF);
  frame_buf_[13] = static_cast<uint8_t>((CRC >> 8) & 0xFF);
  frame_buf_[14] = protocol::FRAME_TAIL;

  const auto SEND_OK = serial.Send(frame_buf_.data(), protocol::DOWN_FRAME_LEN);
  if (!SEND_OK) {
    MV_LOG_WARN("RmShooter", "Serial.Send() failed (seq={} yaw={:.4f}rad pitch={:.4f}rad fire={})",
                frame_buf_[3], control.yaw, pitch_out, FIRE_U8);
  }
  return SEND_OK;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
double RmShooter::BallisticCompensation(double pitch_rad, double target_dist) const noexcept {
  // 简化重力补偿：Δpitch ≈ atan(g×d / (2×v0²)) （小角度近似）
  if (bullet_speed_ < 0.1F) {
    return pitch_rad;
  }
  const double DELTA =
      static_cast<double>(gravity_) * target_dist /
      (2.0 * static_cast<double>(bullet_speed_) * static_cast<double>(bullet_speed_));
  return pitch_rad + std::atan(DELTA);
}

}  // namespace mv::modules
