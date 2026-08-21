#include "modules/armor_predictor/detail/armor_motion_model.hpp"

#include <cmath>

namespace mv::modules::detail {

void Inject(const ErrorVector& error, NominalState& state) {
  auto value = CastState<double>(state);
  Inject(error, value);
  state.position_world = value.position_world;
  state.velocity_world = value.velocity_world;
  state.world_q_car = value.world_q_car.normalized();
  state.yaw_velocity_rad_s = value.yaw_velocity_rad_s;
  state.log_radius_1 = value.log_radius_1;
  state.log_radius_2 = value.log_radius_2;
  state.height_offset_m = value.height_offset_m;
}

ErrorVector BoxMinus(const NominalState& reference, const NominalState& value) {
  return BoxMinus(CastState<double>(reference), CastState<double>(value));
}

NominalState PredictState(const NominalState& state, double dt) {
  const auto VALUE = PredictState(CastState<double>(state), dt);
  return {.position_world = VALUE.position_world,
          .velocity_world = VALUE.velocity_world,
          .world_q_car = VALUE.world_q_car.normalized(),
          .yaw_velocity_rad_s = VALUE.yaw_velocity_rad_s,
          .log_radius_1 = VALUE.log_radius_1,
          .log_radius_2 = VALUE.log_radius_2,
          .height_offset_m = VALUE.height_offset_m};
}

std::array<double, K_STATE_SIZE> DiagnosticState(const NominalState& state) {
  std::array<double, K_STATE_SIZE> result{};
  result[state_index::CX] = state.position_world.x();
  result[state_index::VX] = state.velocity_world.x();
  result[state_index::CY] = state.position_world.y();
  result[state_index::VY] = state.velocity_world.y();
  result[state_index::CZ] = state.position_world.z();
  result[state_index::VZ] = state.velocity_world.z();
  const auto ROTATION = So3Log(state.world_q_car);
  result[state_index::ROT_X] = ROTATION.x();
  result[state_index::ROT_Y] = ROTATION.y();
  result[state_index::ROT_Z] = ROTATION.z();
  result[state_index::VYAW] = state.yaw_velocity_rad_s;
  result[state_index::LOG_R1] = state.log_radius_1;
  result[state_index::LOG_R2] = state.log_radius_2;
  result[state_index::H] = state.height_offset_m;
  return result;
}

geometry::RigidTransform WorldArmorPose(const NominalState& state, ArmorMount mount) {
  const auto POSE = WorldTArmor(CastState<double>(state), mount);
  return {.translation = POSE.translation, .rotation = POSE.rotation.normalized()};
}

double HeadingYaw(const NominalState& state) noexcept {
  const auto HEADING = state.world_q_car * geometry::Vector3::UnitX();
  return std::atan2(HEADING.y(), HEADING.x());
}

}  // namespace mv::modules::detail
