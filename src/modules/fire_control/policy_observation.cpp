#include "modules/fire_control/fire_control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
namespace mv::modules {
bool ControlValuesFinite(const ControlInputSnapshot& input) noexcept {
  const auto& e = input.prediction;
  const auto VALID_TRANSFORM = [](const geometry::RigidTransform& t) {
    return t.translation.allFinite() && t.rotation.coeffs().allFinite() &&
           t.rotation.squaredNorm() > 1e-12;
  };
  return e.center_world.allFinite() && e.velocity_world.allFinite() &&
         e.orientation_world.coeffs().allFinite() && e.orientation_world.squaredNorm() > 1e-12 &&
         e.center_covariance_world.allFinite() &&
         (e.center_covariance_world.diagonal().array() >= 0).all() &&
         std::isfinite(e.yaw_variance_rad2) && e.yaw_variance_rad2 >= 0 &&
         std::isfinite(e.yaw_velocity_rad_s) && std::isfinite(e.armor_tilt_rad) &&
         std::isfinite(e.height_offset_m) &&
         std::all_of(e.radii_m.begin(), e.radii_m.end(),
                     [](double v) { return std::isfinite(v) && v > 0; }) &&
         std::all_of(e.state_vector.begin(), e.state_vector.end(),
                     [](double v) { return std::isfinite(v); }) &&
         std::all_of(e.covariance_diagonal.begin(), e.covariance_diagonal.end(),
                     [](double v) { return std::isfinite(v) && v >= 0; }) &&
         VALID_TRANSFORM(input.world_t_gimbal) && VALID_TRANSFORM(input.gimbal_t_muzzle);
}
FireRejectReason RefereeRejectReason(const RefereeObservation& v,
                                     std::chrono::steady_clock::time_point now,
                                     double max_age_s) noexcept {
  if (!v.valid || v.received_at > now || !std::isfinite(v.age_at_receive_s) ||
      v.age_at_receive_s < 0 || !std::isfinite(v.heat) || v.heat < 0 ||
      !std::isfinite(v.heat_limit) || v.heat_limit <= 0 || !std::isfinite(v.cooling_per_second) ||
      v.cooling_per_second < 0)
    return FireRejectReason::REFEREE_INVALID;
  const double AGE =
      v.age_at_receive_s + std::chrono::duration<double>(now - v.received_at).count();
  if (AGE > max_age_s)
    return FireRejectReason::REFEREE_STALE;
  if (!v.alive || !v.fire_permitted || v.fire_blocks != 0 ||
      (!v.unlimited && v.allowance_remaining == 0))
    return FireRejectReason::REFEREE_BLOCKED;
  return FireRejectReason::NONE;
}
PolicyObservation FireControl::Observe(const ControlInputSnapshot& input,
                                       const hal::GimbalFeedback& feedback,
                                       std::chrono::steady_clock::time_point now) const {
  PolicyObservation o;
  o.estimate = input.prediction;
  o.feedback = feedback;
  o.referee = input.referee;
  o.chassis_motion = input.chassis_motion;
  o.previous_slot = external_mode_ ? external_slot_ : rule_.SelectedSlot();
  if (last_fire_start_)
    o.since_request_s = std::chrono::duration<double>(now - *last_fire_start_).count();
  const auto SOURCE = input.prediction.source_steady_time
                          ? input.prediction.source_steady_time
                          : (!input.prediction.source_capture_timestamp_ns
                                 ? std::optional(input.prediction.source_receive_steady_time)
                                 : std::nullopt);
  o.prediction_age_s = SOURCE && *SOURCE <= now
                           ? std::chrono::duration<double>(now - *SOURCE).count()
                           : std::numeric_limits<double>::infinity();
  o.feedback_age_s = feedback.valid && feedback.timestamp <= now
                         ? std::chrono::duration<double>(now - feedback.timestamp).count()
                         : std::numeric_limits<double>::infinity();
  o.referee_age_s = input.referee.valid && input.referee.received_at <= now
                        ? input.referee.age_at_receive_s +
                              std::chrono::duration<double>(now - input.referee.received_at).count()
                        : std::numeric_limits<double>::infinity();
  o.action_mask[0] = true;
  const auto STATE = input.prediction.state;
  const bool STATE_OK =
      STATE == TrackerState::TRACKING || STATE == TrackerState::DETECTING ||
      (STATE == TrackerState::TEMP_LOST &&
       (!temp_lost_since_ || std::chrono::duration<double>(now - *temp_lost_since_).count() <=
                                 config_.max_temp_lost_control_s));
  if (!STATE_OK || o.prediction_age_s > config_.max_prediction_age_s ||
      o.feedback_age_s > config_.max_prediction_age_s || !std::isfinite(feedback.yaw) ||
      !std::isfinite(feedback.pitch) || !std::isfinite(feedback.yaw_velocity) ||
      !std::isfinite(feedback.pitch_velocity) || !ControlValuesFinite(input))
    return o;
  const auto MUZZLE = geometry::Compose(input.world_t_gimbal, input.gimbal_t_muzzle);
  if (!MUZZLE.translation.allFinite() || !MUZZLE.rotation.coeffs().allFinite())
    return o;
  const bool FIRE =
      config_.auto_fire && input.external_control_enabled && STATE == TrackerState::TRACKING &&
      RefereeRejectReason(input.referee, now, config_.max_referee_age_s) ==
          FireRejectReason::NONE &&
      (!last_fire_start_ ||
       std::chrono::duration<double>(now - *last_fire_start_).count() >= config_.fire_interval_s) &&
      (!pulse_until_ || now >= *pulse_until_);
  for (int i = 0; i < 4; ++i) {
    o.candidates[i] =
        SolveBallistic(input, i, MUZZLE, o.prediction_age_s + config_.command_delay_s);
    o.action_mask[1 + 2 * i] = o.candidates[i].valid;
    o.action_mask[2 + 2 * i] = o.candidates[i].valid && FIRE;
  }
  return o;
}
}  // namespace mv::modules
