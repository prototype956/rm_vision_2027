#pragma once
#include "modules/fire_control/control_types.hpp"
#include "simulation/combat_frame_data.hpp"

#include <cmath>
namespace mv::runtime {
/** @brief Whitelist sampled self data. Simulation and monotonic epochs are never subtracted. */
[[nodiscard]] inline modules::RefereeObservation AdaptRefereeObservation(
    const simulation::CombatFrameMeta* combat, std::uint64_t round,
    std::optional<std::chrono::steady_clock::time_point> frame_time,
    std::chrono::steady_clock::time_point now) noexcept {
  modules::RefereeObservation o;
  if (!combat || combat->referee_valid != 1 || combat->round_id != round ||
      combat->referee_sample_ns > combat->sim_time_ns || !frame_time || *frame_time > now)
    return o;
  const auto& s = combat->self_referee;
  if (s.life > 1 || s.fire_permitted > 1 || s.allowance_mode > 1 || !std::isfinite(s.heat) ||
      s.heat < 0 || !std::isfinite(s.heat_limit) || s.heat_limit <= 0 ||
      !std::isfinite(s.cooling_per_second) || s.cooling_per_second < 0)
    return o;
  o.valid = true;
  o.alive = s.life == 0;
  o.fire_permitted = s.fire_permitted == 1;
  o.unlimited = s.allowance_mode == 0;
  o.allowance_remaining = s.allowance_remaining;
  o.fire_blocks = s.fire_blocks;
  o.heat = s.heat;
  o.heat_limit = s.heat_limit;
  o.cooling_per_second = s.cooling_per_second;
  o.sample_sequence = combat->referee_sample_sequence;
  o.sample_time_ns = combat->referee_sample_ns;
  o.received_at = now;
  o.age_at_receive_s = static_cast<double>(combat->sim_time_ns - combat->referee_sample_ns) * 1e-9 +
                       std::chrono::duration<double>(now - *frame_time).count();
  return o;
}
}  // namespace mv::runtime
