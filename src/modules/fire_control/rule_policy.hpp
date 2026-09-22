#pragma once
#include "modules/fire_control/control_types.hpp"
namespace mv::modules {
/** @brief 沿用既有的目标选择与开火启发式规则，每个控制会话独占一个实例。 */
class RulePolicy final {
 public:
  explicit RulePolicy(FireControlConfig config) : config_(config) {}
  /** @brief 在预测时域上按观察角滞回和持续改善条件选择四装甲槽位。 */
  [[nodiscard]] int SelectSlot(const ControlInputSnapshot& input,
                               const geometry::Vector3& muzzle_world, double horizon_s,
                               const hal::GimbalFeedback& feedback,
                               std::chrono::steady_clock::time_point now,
                               ArmorSelectionDiagnostics& diagnostics);
  [[nodiscard]] FireRejectReason Evaluate(const ControlInputSnapshot& input,
                                          const hal::GimbalFeedback& feedback,
                                          FireControlResult& result);
  void ResetSelection() noexcept {
    locked_slot_ = -1;
    pending_slot_ = -1;
    pending_since_ = {};
    ResetFireReadiness();
  }
  void ResetFireReadiness() noexcept {
    last_stable_slot_ = -1;
    stable_cycles_ = 0;
  }
  [[nodiscard]] int SelectedSlot() const noexcept { return locked_slot_; }

 private:
  FireControlConfig config_;
  int locked_slot_{-1};   ///< 当前锁定的四装甲槽位。
  int pending_slot_{-1};  ///< 等待延时确认的切换候选槽位。
  std::chrono::steady_clock::time_point pending_since_{};  ///< 候选首次持续更优的时刻。
  int last_stable_slot_{-1};                               ///< 开火稳定计数对应的槽位。
  int stable_cycles_{0};  ///< 当前槽位连续进入开火窗口的周期数。
};
}  // namespace mv::modules
