/**
 * @file simple_voter.hpp
 * @brief 简单开火投票器
 *
 * 【策略说明】
 *   当 auto_fire = true（来自配置）且目标处于 tracking 状态时允许开火。
 *   不含冷却计时，适合调试阶段使用。
 *   正式比赛建议替换为 CooldownVoter（带冷却窗口 + 连续跟踪计数）。
 *
 * 【YAML 配置字段】（来自 vision.yaml 的 auto_aim.shooter 节点）
 * @code
 *   auto_aim:
 *     shooter:
 *       auto_fire: true    # false 时 Vote() 始终返回 false（锁定开火）
 * @endcode
 *
 * 工厂键：`"simple"`
 */
#pragma once

#include "interfaces/i_voter.hpp"

namespace mv::modules {

class SimpleVoter final : public IVoter {
 public:
  SimpleVoter();
  ~SimpleVoter() override;

  SimpleVoter(const SimpleVoter&) = delete;
  SimpleVoter& operator=(const SimpleVoter&) = delete;
  SimpleVoter(SimpleVoter&&) = delete;
  SimpleVoter& operator=(SimpleVoter&&) = delete;

  bool Init(const YAML::Node& config) override;

  [[nodiscard]] bool Vote(const TrackTarget& target, const GimbalControl& control) override;

  void Reset() override;

  [[nodiscard]] bool IsInitialized() const noexcept override { return initialized_; }

 private:
  bool auto_fire_{false};
  bool initialized_{false};
};

}  // namespace mv::modules
