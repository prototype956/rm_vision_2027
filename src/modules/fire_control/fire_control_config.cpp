#include "modules/fire_control/fire_control_config.hpp"

#include "core/config.hpp"

#include <cmath>

#include <numbers>

namespace mv::modules {

FireControlConfig ParseFireControlConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "fire control config";
  ConfigLoader::RejectUnknownKeys(root,
                                  {"schema_version", "enabled", "ballistics", "timing", "selection",
                                   "uncertainty", "fire_window", "pulse"},
                                  CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1) {
    throw ConfigError("fire control config schema_version must be 1");
  }
  const auto BALLISTICS = root["ballistics"];
  const auto TIMING = root["timing"];
  const auto SELECTION = root["selection"];
  const auto UNCERTAINTY = root["uncertainty"];
  const auto WINDOW = root["fire_window"];
  const auto PULSE = root["pulse"];
  ConfigLoader::RejectUnknownKeys(
      BALLISTICS, {"bullet_speed_mps", "gravity_mps2", "max_iterations", "time_tolerance_s"},
      "fire control config.ballistics");
  ConfigLoader::RejectUnknownKeys(
      TIMING, {"command_delay_s", "max_prediction_age_s", "max_temp_lost_control_s"},
      "fire control config.timing");
  ConfigLoader::RejectUnknownKeys(
      SELECTION,
      {"enter_angle_deg", "leave_angle_deg", "switch_improvement_deg", "switch_confirmation_s"},
      "fire control config.selection");
  ConfigLoader::RejectUnknownKeys(UNCERTAINTY, {"max_center_position_std_m", "max_yaw_std_rad"},
                                  "fire control config.uncertainty");
  ConfigLoader::RejectUnknownKeys(
      WINDOW, {"scale", "min_yaw_deg", "max_yaw_deg", "min_pitch_deg", "max_pitch_deg"},
      "fire control config.fire_window");
  ConfigLoader::RejectUnknownKeys(PULSE, {"stable_cycles", "interval_s", "width_s"},
                                  "fire control config.pulse");

  constexpr double DEG = std::numbers::pi / 180.0;
  FireControlConfig config;
  config.auto_fire = ConfigLoader::Require<bool>(root, "enabled", CONTEXT);
  config.bullet_speed_mps = ConfigLoader::Require<double>(BALLISTICS, "bullet_speed_mps", CONTEXT);
  config.gravity_mps2 = ConfigLoader::Require<double>(BALLISTICS, "gravity_mps2", CONTEXT);
  config.ballistic_max_iterations =
      ConfigLoader::Require<int>(BALLISTICS, "max_iterations", CONTEXT);
  config.ballistic_time_tolerance_s =
      ConfigLoader::Require<double>(BALLISTICS, "time_tolerance_s", CONTEXT);
  config.command_delay_s = ConfigLoader::Require<double>(TIMING, "command_delay_s", CONTEXT);
  config.max_prediction_age_s =
      ConfigLoader::Require<double>(TIMING, "max_prediction_age_s", CONTEXT);
  config.max_temp_lost_control_s =
      ConfigLoader::Require<double>(TIMING, "max_temp_lost_control_s", CONTEXT);
  config.armor_enter_angle_rad =
      ConfigLoader::Require<double>(SELECTION, "enter_angle_deg", CONTEXT) * DEG;
  config.armor_leave_angle_rad =
      ConfigLoader::Require<double>(SELECTION, "leave_angle_deg", CONTEXT) * DEG;
  config.slot_switch_improvement_rad =
      ConfigLoader::Require<double>(SELECTION, "switch_improvement_deg", CONTEXT) * DEG;
  config.slot_switch_confirmation_s =
      ConfigLoader::Require<double>(SELECTION, "switch_confirmation_s", CONTEXT);
  config.max_center_position_std_m =
      ConfigLoader::Require<double>(UNCERTAINTY, "max_center_position_std_m", CONTEXT);
  config.max_yaw_std_rad = ConfigLoader::Require<double>(UNCERTAINTY, "max_yaw_std_rad", CONTEXT);
  config.fire_window_scale = ConfigLoader::Require<double>(WINDOW, "scale", CONTEXT);
  config.min_fire_yaw_rad = ConfigLoader::Require<double>(WINDOW, "min_yaw_deg", CONTEXT) * DEG;
  config.max_fire_yaw_rad = ConfigLoader::Require<double>(WINDOW, "max_yaw_deg", CONTEXT) * DEG;
  config.min_fire_pitch_rad = ConfigLoader::Require<double>(WINDOW, "min_pitch_deg", CONTEXT) * DEG;
  config.max_fire_pitch_rad = ConfigLoader::Require<double>(WINDOW, "max_pitch_deg", CONTEXT) * DEG;
  config.stable_cycles = ConfigLoader::Require<int>(PULSE, "stable_cycles", CONTEXT);
  config.fire_interval_s = ConfigLoader::Require<double>(PULSE, "interval_s", CONTEXT);
  config.fire_pulse_width_s = ConfigLoader::Require<double>(PULSE, "width_s", CONTEXT);

  const bool VALID =
      config.bullet_speed_mps > 0.0 && config.gravity_mps2 > 0.0 &&
      config.ballistic_max_iterations > 0 && config.ballistic_time_tolerance_s > 0.0 &&
      config.command_delay_s >= 0.0 && config.max_prediction_age_s > 0.0 &&
      config.max_temp_lost_control_s >= 0.0 && config.armor_enter_angle_rad > 0.0 &&
      config.armor_leave_angle_rad > config.armor_enter_angle_rad &&
      config.armor_leave_angle_rad < std::numbers::pi / 2.0 &&
      config.slot_switch_improvement_rad >= 0.0 && config.slot_switch_confirmation_s > 0.0 &&
      config.max_center_position_std_m > 0.0 && config.max_yaw_std_rad > 0.0 &&
      config.fire_window_scale > 0.0 && config.fire_window_scale <= 1.0 &&
      config.min_fire_yaw_rad > 0.0 && config.max_fire_yaw_rad >= config.min_fire_yaw_rad &&
      config.min_fire_pitch_rad > 0.0 && config.max_fire_pitch_rad >= config.min_fire_pitch_rad &&
      config.stable_cycles > 0 && config.fire_interval_s > 0.0 && config.fire_pulse_width_s > 0.0 &&
      config.fire_pulse_width_s < config.fire_interval_s;
  if (!VALID)
    throw ConfigError("fire control config contains an invalid scalar range");
  return config;
}

}  // namespace mv::modules
