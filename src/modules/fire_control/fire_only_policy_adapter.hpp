#pragma once
#include "modules/fire_control/rule_policy.hpp"

#include <algorithm>
#include <cmath>
namespace mv::modules {
/** @brief Fire-only ablation: the rule selector runs every cycle, even for no-shot decisions. */
class FireOnlyPolicyAdapter final {
 public:
  explicit FireOnlyPolicyAdapter(FireControlConfig config) : config_(config), rule_(config) {}
  [[nodiscard]] PolicyDecision Decide(const ControlInputSnapshot& input,
                                      const PolicyObservation& observation,
                                      std::chrono::steady_clock::time_point now, bool fire) {
    const auto IDENTITY =
        std::pair(input.prediction.source_round_id, input.prediction.track_generation);
    if (identity_ != IDENTITY || label_ != input.prediction.label ||
        type_ != input.prediction.type) {
      rule_.ResetSelection();
      identity_ = IDENTITY;
      label_ = input.prediction.label;
      type_ = input.prediction.type;
    }
    if (!std::any_of(observation.action_mask.begin() + 1, observation.action_mask.end(),
                     [](bool v) { return v; })) {
      rule_.ResetSelection();
      return {};
    }
    const auto MUZZLE = geometry::Compose(input.world_t_gimbal, input.gimbal_t_muzzle);
    const auto CENTER =
        ExtrapolatePrediction(input.prediction, observation.prediction_age_s).center_world;
    ArmorSelectionDiagnostics diagnostics;
    const int SLOT =
        rule_.SelectSlot(input, MUZZLE.translation,
                         observation.prediction_age_s + config_.command_delay_s +
                             (CENTER - MUZZLE.translation).norm() / config_.bullet_speed_mps,
                         observation.feedback, now, diagnostics);
    return {SLOT < 0 ? 0 : 1 + 2 * SLOT + int(fire)};
  }
  /** @brief Bind the action mask to the selected rule slot; WAIT is only available without a slot.
   */
  static void RestrictMask(PolicyObservation& observation, PolicyDecision choice) noexcept {
    const auto PREVIOUS = observation.action_mask;
    observation.action_mask.fill(false);
    if (!choice.Valid() || choice.Slot() < 0) {
      observation.action_mask[0] = true;
      return;
    }
    const int TRACK = 1 + 2 * choice.Slot();
    observation.action_mask[TRACK] = PREVIOUS[TRACK];
    observation.action_mask[TRACK + 1] = PREVIOUS[TRACK + 1];
    if (!observation.action_mask[TRACK])
      observation.action_mask[0] = true;
  }

 private:
  FireControlConfig config_;
  RulePolicy rule_;
  std::optional<std::pair<std::uint64_t, std::uint64_t>> identity_;
  std::optional<ArmorLabel> label_;
  std::optional<geometry::ArmorType> type_;
};
}  // namespace mv::modules
