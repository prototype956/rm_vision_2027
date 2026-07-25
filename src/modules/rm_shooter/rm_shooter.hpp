/**
 * @file rm_shooter.hpp
 * @brief RoboMaster 协议串口发射器（正式协议 v1.0）
 *
 * 【下行帧格式】详见 src/hal/serial/rm_protocol.hpp
 *   - 总长 15 字节，帧头 0xAA/0x0F，帧尾 0x0D
 *   - yaw/pitch 以 × 10000 的 int16_t Little-Endian 传输（精度 ~0.1 mrad）
 *   - shoot 为脉冲触发：GimbalControl::fire==true 时发送 1，否则发送 0
 *   - 完整性校验：CRC16-CCITT（覆盖帧头到 payload 末尾，共 12 字节）
 *
 * 【弹道补偿】
 *   bullet_speed 和 gravity 从配置读取，对 pitch 做简单重力补偿：
 *   Δpitch ≈ atan(g × d / (2 × v0²)) （小角度近似）
 *   补偿默认关闭（enable_ballistic_compensation: false）。
 *
 * 【YAML 配置字段】（来自 vision.yaml 的 auto_aim 节点）
 * @code
 *   auto_aim:
 *     shooter:
 *       enable_ballistic_compensation: false
 *       bullet_speed:  15.0   # m/s
 *       gravity:        9.8   # m/s²
 * @endcode
 *
 * 工厂键：`"rm"`
 */
#pragma once

#include "hal/serial/rm_protocol.hpp"
#include "interfaces/i_shooter.hpp"

#include <array>
#include <cstdint>

namespace mv::modules {

class RmShooter final : public IShooter {
 public:
  RmShooter();
  ~RmShooter() override;

  RmShooter(const RmShooter&) = delete;
  RmShooter& operator=(const RmShooter&) = delete;
  RmShooter(RmShooter&&) = delete;
  RmShooter& operator=(RmShooter&&) = delete;

  bool Init(const YAML::Node& config) override;

  bool Send(hal::ISerial& serial, const GimbalControl& control) override;

  [[nodiscard]] bool IsInitialized() const noexcept override { return initialized_; }

 private:
  // ── 协议帧缓冲 ────────────────────────────────────────────────────────────

  /// 下行帧缓冲区（大小由 rm_protocol::DOWN_FRAME_LEN 决定）
  std::array<uint8_t, protocol::DOWN_FRAME_LEN> frame_buf_{};

  // ── 参数 ─────────────────────────────────────────────────────────────────

  bool ballistic_comp_{false};
  float bullet_speed_{15.0F};
  float gravity_{9.8F};

  bool initialized_{false};

  /// 下行帧序列号（0~255 循环滚动，每次 Send() 递增）
  uint8_t seq_{0};

  // ── 内部方法 ──────────────────────────────────────────────────────────────

  /** 弹道补偿：根据 pitch + distance 返回修正后的 pitch */
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  [[nodiscard]] double BallisticCompensation(double pitch_rad, double target_dist) const noexcept;
};

}  // namespace mv::modules
