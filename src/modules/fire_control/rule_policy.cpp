#include "modules/fire_control/rule_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <numbers>
namespace mv::modules {
namespace {
double Wrap(double a) noexcept {
  return std::remainder(a, 2.0 * std::numbers::pi);
}
double ViewAngle(const PredictedArmorPose& armor, const geometry::Vector3& muzzle) noexcept {
  const auto TO_MUZZLE = muzzle - armor.world_t_armor.translation;
  if (!TO_MUZZLE.allFinite() || TO_MUZZLE.norm() < 1.0e-9) {
    return std::numeric_limits<double>::infinity();
  }
  const auto NORMAL = armor.world_t_armor.rotation * geometry::Vector3::UnitZ();
  return std::acos(std::clamp(NORMAL.normalized().dot(TO_MUZZLE.normalized()), -1.0, 1.0));
}

}  // namespace
int RulePolicy::SelectSlot(const ControlInputSnapshot& input, const geometry::Vector3& muzzle_world,
                           double horizon_s, const hal::GimbalFeedback& feedback,
                           std::chrono::steady_clock::time_point now,
                           ArmorSelectionDiagnostics& diagnostics) {
  const int ORIGINAL_LOCKED_SLOT = locked_slot_;
  const auto HORIZON = ExtrapolatePrediction(input.prediction, std::max(0.0, horizon_s));
  diagnostics.horizon_s = HORIZON.seconds;
  diagnostics.switch_confirmation_s = config_.slot_switch_confirmation_s;
  for (int slot = 0; slot < 4; ++slot) {
    const auto& armor = HORIZON.armors[slot];
    const auto DELTA = armor.world_t_armor.translation - muzzle_world;
    const double YAW = std::atan2(DELTA.y(), DELTA.x());
    const double PITCH = std::atan2(DELTA.z(), std::hypot(DELTA.x(), DELTA.y()));
    const double SLEW = std::hypot(Wrap(YAW - feedback.yaw), PITCH - feedback.pitch);
    const double VIEW = ViewAngle(armor, muzzle_world);
    diagnostics.candidates[slot] = {
        .slot = slot,
        .predicted_pose = armor,
        .view_angle_rad = VIEW,
        .slew_angle_rad = SLEW,
        .enter_eligible = std::isfinite(VIEW) && VIEW <= config_.armor_enter_angle_rad,
        .leave_eligible = std::isfinite(VIEW) && VIEW <= config_.armor_leave_angle_rad};
  }
  const auto BEST =
      std::min_element(diagnostics.candidates.begin(), diagnostics.candidates.end(),
                       [](const auto& left, const auto& right) {
                         if (std::abs(left.view_angle_rad - right.view_angle_rad) > 1.0e-9)
                           return left.view_angle_rad < right.view_angle_rad;
                         return left.slew_angle_rad < right.slew_angle_rad;
                       });
  const bool BEST_VALID = BEST != diagnostics.candidates.end() && BEST->enter_eligible;

  // 临时丢失期间不发起新切换，仅在较宽的离开角内维持原锁定，避免外推目标抖动。
  if (input.prediction.state == TrackerState::TEMP_LOST) {
    pending_slot_ = -1;
    pending_since_ = {};
    if (locked_slot_ >= 0 && locked_slot_ < 4 &&
        diagnostics.candidates[static_cast<std::size_t>(locked_slot_)].leave_eligible) {
      diagnostics.decision = ArmorSelectionDecision::TEMP_LOST_HELD;
    } else {
      ResetSelection();
      diagnostics.decision = ArmorSelectionDecision::TEMP_LOST_CLEARED;
    }
    diagnostics.locked_slot = locked_slot_;
    return locked_slot_;
  }

  if (locked_slot_ >= 0 && locked_slot_ < 4) {
    const auto& locked = diagnostics.candidates[static_cast<std::size_t>(locked_slot_)];
    if (locked.leave_eligible) {
      // 新槽位必须同时满足观察角改善量和持续时间，抑制相邻装甲交界处来回切换。
      if (BEST_VALID && BEST->slot != locked_slot_ &&
          BEST->view_angle_rad + config_.slot_switch_improvement_rad <= locked.view_angle_rad) {
        if (pending_slot_ != BEST->slot) {
          pending_slot_ = BEST->slot;
          pending_since_ = now;
        }
        diagnostics.pending_duration_s =
            std::max(0.0, std::chrono::duration<double>(now - pending_since_).count());
        if (diagnostics.pending_duration_s >= config_.slot_switch_confirmation_s) {
          locked_slot_ = BEST->slot;
          pending_slot_ = -1;
          pending_since_ = {};
          diagnostics.switched = true;
          diagnostics.decision = ArmorSelectionDecision::SWITCHED;
        } else {
          diagnostics.decision = ArmorSelectionDecision::PENDING_SWITCH;
        }
      } else {
        pending_slot_ = -1;
        pending_since_ = {};
        diagnostics.decision = ArmorSelectionDecision::HELD;
      }
      diagnostics.locked_slot = locked_slot_;
      diagnostics.pending_slot = pending_slot_;
      return locked_slot_;
    }
    locked_slot_ = -1;
    pending_slot_ = -1;
    pending_since_ = {};
    diagnostics.decision = ArmorSelectionDecision::LOST_ANGLE;
  }

  if (BEST_VALID) {
    locked_slot_ = BEST->slot;
    diagnostics.switched = ORIGINAL_LOCKED_SLOT >= 0 && ORIGINAL_LOCKED_SLOT != locked_slot_;
    diagnostics.decision =
        diagnostics.switched ? ArmorSelectionDecision::SWITCHED : ArmorSelectionDecision::ACQUIRED;
  } else {
    diagnostics.decision = ArmorSelectionDecision::NO_CANDIDATE;
  }
  diagnostics.locked_slot = locked_slot_;
  diagnostics.pending_slot = pending_slot_;
  return locked_slot_;
}

FireRejectReason RulePolicy::Evaluate(const ControlInputSnapshot& input,
                                      const hal::GimbalFeedback& feedback,
                                      FireControlResult& result) {
  auto& output = result.output;
  auto& diagnostics = result.diagnostics;
  const int SLOT = output.selected_slot;
  diagnostics.yaw_error = Wrap(output.target_yaw - feedback.yaw);
  diagnostics.pitch_error = output.target_pitch - feedback.pitch;
  const double WIDTH = input.prediction.type == geometry::ArmorType::LARGE ? 0.225 : 0.135;
  constexpr double HEIGHT = 0.055;
  output.fire_yaw_window =
      std::clamp(std::atan2(0.5 * WIDTH * config_.fire_window_scale, output.ballistic.distance_m),
                 config_.min_fire_yaw_rad, config_.max_fire_yaw_rad);
  output.fire_pitch_window =
      std::clamp(std::atan2(0.5 * HEIGHT * config_.fire_window_scale, output.ballistic.distance_m),
                 config_.min_fire_pitch_rad, config_.max_fire_pitch_rad);

  const double POSITION_STD =
      std::sqrt(std::max(0.0, input.prediction.center_covariance_world.diagonal().maxCoeff()));
  const double YAW_STD = std::sqrt(std::max(0.0, input.prediction.yaw_variance_rad2));
  const bool UNCERTAINTY_OK =
      POSITION_STD <= config_.max_center_position_std_m && YAW_STD <= config_.max_yaw_std_rad;
  const bool AIM_OK = std::abs(diagnostics.yaw_error) <= output.fire_yaw_window &&
                      std::abs(diagnostics.pitch_error) <= output.fire_pitch_window;
  // 稳定计数与槽位绑定；换槽或离开窗口都会重新累计，防止切换瞬间误触发。
  if (SLOT == last_stable_slot_ && AIM_OK) {
    ++stable_cycles_;
  } else {
    last_stable_slot_ = SLOT;
    stable_cycles_ = AIM_OK ? 1 : 0;
  }
  output.stable_cycles = stable_cycles_;

  if (!UNCERTAINTY_OK)
    return FireRejectReason::HIGH_UNCERTAINTY;
  if (!AIM_OK)
    return FireRejectReason::AIM_ERROR_TOO_LARGE;
  if (stable_cycles_ < config_.stable_cycles)
    return FireRejectReason::AIM_NOT_STABLE;
  return FireRejectReason::NONE;
}
}  // namespace mv::modules
