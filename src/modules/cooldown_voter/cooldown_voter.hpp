/**
 * @file cooldown_voter.hpp
 * @brief 带冷却窗口的开火决策投票器（Pimpl 公开接口）
 *
 * 【设计动机】
 *   SimpleVoter 仅检查 is_tracking，实战中会导致以下问题：
 *   1. 目标刚进入视野时 EKF 尚未收敛，此时开火精度差；
 *   2. EKF 短暂丢帧（TEMP_LOST）后重锁，需要一段冷却才能再次开火；
 *   3. 云台到位但目标实际已偏离照门，需要对准度阈值约束。
 *
 *   CooldownVoter 在 SimpleVoter 基础上增加三层防护：
 *   ① 最小锁定帧数（min_lock_frames）: 连续跟踪帧数不足时禁止开火；
 *   ② 对准误差阈值（first_tolerance_rad）: 预测点与发射方向误差过大时禁止开火；
 *   ③ 冷却宽容帧（fire_tolerate_frames）: 短暂掉帧后（< tolerate 帧）
 *      继续允许开火，避免因 TCP 单帧抖动导致开火中断。
 *
 * 【Vote() 内部状态机】
 * @code
 *   状态变量：
 *     consecutive_track_frames_  : 连续跟踪帧数（TRACKING 状态下递增）
 *     fire_permit_               : 当前是否处于允许开火状态
 *     lost_fire_frames_          : 上次丢失跟踪后经过的帧数
 *
 *   每帧逻辑（简述）：
 *     if is_tracking:
 *       consecutive_track_frames_++
 *       lost_fire_frames_ = 0
 *       if consecutive_track_frames_ >= min_lock_frames &&
 *          abs(angular_error) <= first_tolerance_rad:
 *         fire_permit_ = true
 *     else:  // 非 TRACKING
 *       consecutive_track_frames_ = 0
 *       lost_fire_frames_++
 *       if lost_fire_frames_ > fire_tolerate_frames:
 *         fire_permit_ = false
 *
 *     return auto_fire_ && fire_permit_
 * @endcode
 *
 * 【YAML 配置字段】（vision.yaml 的 auto_aim.cooldown_voter 节点）
 * @code
 *   auto_aim:
 *     cooldown_voter:
 *       auto_fire:            true    # false 时 Vote() 始终返回 false
 *       min_lock_frames:      5       # 至少连续跟踪多少帧才允许首次开火
 *       first_tolerance_rad:  0.05    # 首次开火时允许的最大瞄准角度误差（rad）
 *       fire_tolerate_frames: 3       # 短暂丢帧后仍延续开火的最大帧数
 * @endcode
 *
 * 工厂键：`"cooldown"`
 */
#pragma once

#include "interfaces/i_voter.hpp"

#include <memory>

namespace mv::modules {

/**
 * @brief 带冷却窗口的开火投票器（Pimpl，工厂键 "cooldown"）
 */
class CooldownVoter final : public IVoter {
 public:
  CooldownVoter();
  ~CooldownVoter() override;

  CooldownVoter(const CooldownVoter&) = delete;
  CooldownVoter& operator=(const CooldownVoter&) = delete;
  CooldownVoter(CooldownVoter&&) = delete;
  CooldownVoter& operator=(CooldownVoter&&) = delete;

  // ── IVoter 接口实现 ────────────────────────────────────────────────────

  bool Init(const YAML::Node& config) override;

  /**
   * @brief 根据跟踪状态、对准误差、连续帧数决策是否开火
   *
   * @param target  本帧跟踪状态快照（读取 is_tracking、yaw_predicted 等）
   * @param control 本帧云台指令（读取 yaw / pitch 比较对准误差）
   * @return true = 允许开火
   */
  [[nodiscard]] bool Vote(const TrackTarget& target, const GimbalControl& control) override;

  /**
   * @brief 清除连续帧计数、冷却状态（切换目标或模式时调用）
   */
  void Reset() override;

  [[nodiscard]] bool IsInitialized() const noexcept override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::modules
