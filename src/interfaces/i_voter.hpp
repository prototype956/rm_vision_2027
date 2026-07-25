/**
 * @file i_voter.hpp
 * @brief 开火决策抽象接口 (IVoter)
 *
 * 【职责边界】
 *   IVoter 负责"根据跟踪状态决策是否允许开火"：
 *   - 输入：TrackTarget（跟踪器输出的目标状态快照）
 *   - 输出：bool（true = 允许开火）
 *
 *   IVoter 与 IPredictor 解耦的原因：
 *   - 预测器只关心"目标在哪里"，不关心"该不该打"；
 *   - 开火决策涉及冷却计时、优先级评分、掉帧容忍等逻辑，
 *     独立为 IVoter 后可单独测试/替换（如: 赛前换保守策略）；
 *   - 双缓冲设计：Predict() 先输出 GimbalControl，
 *     Voter 再对 fire 字段"签字"，二者串联。
 *
 * 【调用时序（Pipeline 中）】
 *   PredictNode:
 *     auto control  = predictor_->Predict(detections, ts, color);
 *     auto target   = predictor_->GetTrackTarget();
 *     control.fire  = voter_->Vote(target, control);   // Voter 签字
 *     output_ch_.Push({control, target, ts, frame_id});
 *
 * 【实现约定】
 *   - Vote() 无副作用地读取 target，内部维护冷却计时等状态；
 *   - Reset() 清除所有计时器和锁定状态（模式切换时调用）；
 *   - Vote() 非线程安全——在预测线程单线程调用。
 *
 * 【内置实现（TODO: Stage 4 后续）】
 *   - `SimpleVoter`   — 只要 is_tracking 就开火（最简，调试用）
 *   - `CooldownVoter` — 带冷却时间 + 掉帧容忍（正式比赛用）
 */
#pragma once

#include "types.hpp"

#include <yaml-cpp/yaml.h>

namespace mv {

class IVoter {
 public:
  // ── 生命周期 ─────────────────────────────────────────────────────────────
  IVoter() = default;
  virtual ~IVoter() = default;

  IVoter(const IVoter&) = delete;
  IVoter& operator=(const IVoter&) = delete;

 protected:
  IVoter(IVoter&&) = default;
  IVoter& operator=(IVoter&&) = default;

 public:
  // ── 核心接口 ─────────────────────────────────────────────────────────────

  /**
   * @brief 初始化投票器（加载冷却时间、掉帧容忍等配置）
   * @param config  YAML 配置节点
   * @return true 成功
   */
  virtual bool Init(const YAML::Node& config) = 0;

  /**
   * @brief 根据跟踪状态和云台指令决策是否允许开火
   *
   * @param target   本帧跟踪器状态快照（来自 IPredictor::GetTrackTarget()）
   * @param control  本帧云台指令（Voter 可读取 tracking 字段辅助决策）
   * @return true = 允许开火（调用方应将 GimbalControl::fire 置为 true）
   *
   * @note Vote() 可维护内部计时器（冷却、连续跟踪帧数等），
   *       但不应修改传入参数。
   */
  [[nodiscard]] virtual bool Vote(const TrackTarget& target,
                                  const GimbalControl& control) = 0;

  /**
   * @brief 重置所有内部状态（冷却计时器、连续帧计数等）
   *
   * 模式切换（AUTO_AIM → ENERGY_BUFF）或目标丢失后调用。
   */
  virtual void Reset() = 0;

  /** @return 投票器是否已完成初始化 */
  [[nodiscard]] virtual bool IsInitialized() const noexcept = 0;
};

}  // namespace mv
