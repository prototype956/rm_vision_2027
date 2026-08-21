#include "modules/armor_predictor/armor_predictor_config.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

namespace mv::modules {
namespace {

template <typename T, std::size_t N>
std::array<T, N> RequireArray(const YAML::Node& parent, const std::string& key,
                              const std::string& context) {
  const auto NODE = parent[key];
  if (!NODE || !NODE.IsSequence() || NODE.size() != N) {
    throw ConfigError(context + "." + key + " must contain exactly " + std::to_string(N) +
                      " values");
  }
  std::array<T, N> result{};
  for (std::size_t index = 0; index < N; ++index) {
    try {
      result[index] = NODE[index].as<T>();
    } catch (const std::exception& error) {
      std::string message = context;
      message.append(".").append(key).append(" has invalid item: ").append(error.what());
      throw ConfigError(message);
    }
  }
  return result;
}

bool PositiveFinite(double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

ArmorPredictorConfig ParseArmorPredictorConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "armor predictor config";
  ConfigLoader::RejectUnknownKeys(
      root,
      {"schema_version", "tracking", "model", "process_noise", "association", "light_association",
       "measurement", "maneuver", "priority", "prediction_horizons_s"},
      CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) !=
      ARMOR_PREDICTOR_CONFIG_SCHEMA_VERSION) {
    throw ConfigError("armor predictor config schema_version must be " +
                      std::to_string(ARMOR_PREDICTOR_CONFIG_SCHEMA_VERSION));
  }

  const auto TRACKING = root["tracking"];
  const auto MODEL = root["model"];
  const auto PROCESS = root["process_noise"];
  const auto ASSOCIATION = root["association"];
  const auto LIGHT_ASSOCIATION = root["light_association"];
  const auto MEASUREMENT = root["measurement"];
  const auto MANEUVER = root["maneuver"];
  const auto PRIORITY = root["priority"];
  ConfigLoader::RejectUnknownKeys(TRACKING, {"min_detect_count", "max_temp_lost_count", "max_dt_s"},
                                  "armor predictor config.tracking");
  ConfigLoader::RejectUnknownKeys(
      MODEL,
      {"esekf_iterations", "vehicle_initial_radius_m", "hero_initial_radius_m",
       "vehicle_armor_tilt_rad", "hero_armor_tilt_rad", "min_radius_m", "max_radius_m",
       "max_height_offset_m", "max_yaw_velocity_rad_s", "initial_covariance_diagonal"},
      "armor predictor config.model");
  ConfigLoader::RejectUnknownKeys(
      PROCESS,
      {"body_acceleration_variance", "yaw_acceleration_variance",
       "roll_pitch_random_walk_variance_per_s", "radius_random_walk_variance_m2_per_s",
       "height_random_walk_variance_m2_per_s"},
      "armor predictor config.process_noise");
  ConfigLoader::RejectUnknownKeys(ASSOCIATION,
                                  {"visible_slot_count", "center_weight", "edge_angle_weight",
                                   "perimeter_ratio_weight", "initial_gate", "gate"},
                                  "armor predictor config.association");
  ConfigLoader::RejectUnknownKeys(
      LIGHT_ASSOCIATION,
      {"dedup_center_length_ratio", "dedup_angle_gate_rad", "dedup_log_length_gate",
       "match_log_length_gate", "match_angle_gate_rad", "match_endpoint_distance_length_ratio",
       "position_weight", "angle_weight", "length_weight"},
      "armor predictor config.light_association");
  ConfigLoader::RejectUnknownKeys(
      MEASUREMENT,
      {"uvl_center_sigma_length_ratio", "uvl_length_sigma_length_ratio", "uvl_angle_sigma_rad",
       "standalone_uvl_center_sigma_length_ratio", "standalone_uvl_length_sigma_length_ratio",
       "standalone_uvl_angle_sigma_rad", "depth_difference_sigma_m"},
      "armor predictor config.measurement");
  ConfigLoader::RejectUnknownKeys(
      MANEUVER,
      {"association_cost_trigger", "confirmation_frames", "confirmation_window_s", "hold_s",
       "trigger_yaw_acceleration_variance", "active_yaw_acceleration_variance", "recovery_gate",
       "nis_per_dof_gate"},
      "armor predictor config.maneuver");
  ConfigLoader::RejectUnknownKeys(PRIORITY, {"sentry", "one", "two", "three", "four"},
                                  "armor predictor config.priority");

  ArmorPredictorConfig config;
  config.min_detect_count = ConfigLoader::Require<int>(TRACKING, "min_detect_count", CONTEXT);
  config.max_temp_lost_count = ConfigLoader::Require<int>(TRACKING, "max_temp_lost_count", CONTEXT);
  config.max_dt_s = ConfigLoader::Require<double>(TRACKING, "max_dt_s", CONTEXT);
  config.esekf_iterations = ConfigLoader::Require<int>(MODEL, "esekf_iterations", CONTEXT);
  config.vehicle_initial_radius_m =
      ConfigLoader::Require<double>(MODEL, "vehicle_initial_radius_m", CONTEXT);
  config.hero_initial_radius_m =
      ConfigLoader::Require<double>(MODEL, "hero_initial_radius_m", CONTEXT);
  config.vehicle_armor_tilt_rad =
      ConfigLoader::Require<double>(MODEL, "vehicle_armor_tilt_rad", CONTEXT);
  config.hero_armor_tilt_rad = ConfigLoader::Require<double>(MODEL, "hero_armor_tilt_rad", CONTEXT);
  config.min_radius_m = ConfigLoader::Require<double>(MODEL, "min_radius_m", CONTEXT);
  config.max_radius_m = ConfigLoader::Require<double>(MODEL, "max_radius_m", CONTEXT);
  config.max_height_offset_m = ConfigLoader::Require<double>(MODEL, "max_height_offset_m", CONTEXT);
  config.max_yaw_velocity_rad_s =
      ConfigLoader::Require<double>(MODEL, "max_yaw_velocity_rad_s", CONTEXT);
  config.initial_covariance_diagonal =
      RequireArray<double, 13>(MODEL, "initial_covariance_diagonal", CONTEXT);

  config.body_acceleration_variance =
      RequireArray<double, 3>(PROCESS, "body_acceleration_variance", CONTEXT);
  config.yaw_acceleration_variance =
      ConfigLoader::Require<double>(PROCESS, "yaw_acceleration_variance", CONTEXT);
  config.roll_pitch_random_walk_variance_per_s =
      ConfigLoader::Require<double>(PROCESS, "roll_pitch_random_walk_variance_per_s", CONTEXT);
  config.radius_random_walk_variance_m2_per_s =
      ConfigLoader::Require<double>(PROCESS, "radius_random_walk_variance_m2_per_s", CONTEXT);
  config.height_random_walk_variance_m2_per_s =
      ConfigLoader::Require<double>(PROCESS, "height_random_walk_variance_m2_per_s", CONTEXT);

  config.visible_slot_count =
      ConfigLoader::Require<int>(ASSOCIATION, "visible_slot_count", CONTEXT);
  config.association_center_weight =
      ConfigLoader::Require<double>(ASSOCIATION, "center_weight", CONTEXT);
  config.association_edge_angle_weight =
      ConfigLoader::Require<double>(ASSOCIATION, "edge_angle_weight", CONTEXT);
  config.association_perimeter_ratio_weight =
      ConfigLoader::Require<double>(ASSOCIATION, "perimeter_ratio_weight", CONTEXT);
  config.association_initial_gate =
      ConfigLoader::Require<double>(ASSOCIATION, "initial_gate", CONTEXT);
  config.association_gate = ConfigLoader::Require<double>(ASSOCIATION, "gate", CONTEXT);
  config.light_dedup_center_length_ratio =
      ConfigLoader::Require<double>(LIGHT_ASSOCIATION, "dedup_center_length_ratio", CONTEXT);
  config.light_dedup_angle_gate_rad =
      ConfigLoader::Require<double>(LIGHT_ASSOCIATION, "dedup_angle_gate_rad", CONTEXT);
  config.light_dedup_log_length_gate =
      ConfigLoader::Require<double>(LIGHT_ASSOCIATION, "dedup_log_length_gate", CONTEXT);
  config.light_match_log_length_gate =
      ConfigLoader::Require<double>(LIGHT_ASSOCIATION, "match_log_length_gate", CONTEXT);
  config.light_match_angle_gate_rad =
      ConfigLoader::Require<double>(LIGHT_ASSOCIATION, "match_angle_gate_rad", CONTEXT);
  config.light_match_endpoint_distance_length_ratio = ConfigLoader::Require<double>(
      LIGHT_ASSOCIATION, "match_endpoint_distance_length_ratio", CONTEXT);
  config.light_match_position_weight =
      ConfigLoader::Require<double>(LIGHT_ASSOCIATION, "position_weight", CONTEXT);
  config.light_match_angle_weight =
      ConfigLoader::Require<double>(LIGHT_ASSOCIATION, "angle_weight", CONTEXT);
  config.light_match_length_weight =
      ConfigLoader::Require<double>(LIGHT_ASSOCIATION, "length_weight", CONTEXT);

  config.maneuver_association_cost_trigger =
      ConfigLoader::Require<double>(MANEUVER, "association_cost_trigger", CONTEXT);
  config.maneuver_confirmation_frames =
      ConfigLoader::Require<int>(MANEUVER, "confirmation_frames", CONTEXT);
  config.maneuver_confirmation_window_s =
      ConfigLoader::Require<double>(MANEUVER, "confirmation_window_s", CONTEXT);
  config.maneuver_hold_s = ConfigLoader::Require<double>(MANEUVER, "hold_s", CONTEXT);
  config.maneuver_trigger_yaw_acceleration_variance =
      ConfigLoader::Require<double>(MANEUVER, "trigger_yaw_acceleration_variance", CONTEXT);
  config.maneuver_active_yaw_acceleration_variance =
      ConfigLoader::Require<double>(MANEUVER, "active_yaw_acceleration_variance", CONTEXT);
  config.maneuver_recovery_gate = ConfigLoader::Require<double>(MANEUVER, "recovery_gate", CONTEXT);
  config.maneuver_nis_per_dof_gate =
      ConfigLoader::Require<double>(MANEUVER, "nis_per_dof_gate", CONTEXT);

  config.uvl_center_sigma_length_ratio =
      ConfigLoader::Require<double>(MEASUREMENT, "uvl_center_sigma_length_ratio", CONTEXT);
  config.uvl_length_sigma_length_ratio =
      ConfigLoader::Require<double>(MEASUREMENT, "uvl_length_sigma_length_ratio", CONTEXT);
  config.uvl_angle_sigma_rad =
      ConfigLoader::Require<double>(MEASUREMENT, "uvl_angle_sigma_rad", CONTEXT);
  config.standalone_uvl_center_sigma_length_ratio = ConfigLoader::Require<double>(
      MEASUREMENT, "standalone_uvl_center_sigma_length_ratio", CONTEXT);
  config.standalone_uvl_length_sigma_length_ratio = ConfigLoader::Require<double>(
      MEASUREMENT, "standalone_uvl_length_sigma_length_ratio", CONTEXT);
  config.standalone_uvl_angle_sigma_rad =
      ConfigLoader::Require<double>(MEASUREMENT, "standalone_uvl_angle_sigma_rad", CONTEXT);
  config.depth_difference_sigma_m =
      ConfigLoader::Require<double>(MEASUREMENT, "depth_difference_sigma_m", CONTEXT);
  config.label_priorities = {ConfigLoader::Require<int>(PRIORITY, "sentry", CONTEXT),
                             ConfigLoader::Require<int>(PRIORITY, "one", CONTEXT),
                             ConfigLoader::Require<int>(PRIORITY, "two", CONTEXT),
                             ConfigLoader::Require<int>(PRIORITY, "three", CONTEXT),
                             ConfigLoader::Require<int>(PRIORITY, "four", CONTEXT)};
  config.prediction_horizons_s = RequireArray<double, 4>(root, "prediction_horizons_s", CONTEXT);

  if (config.min_detect_count <= 0 || config.max_temp_lost_count < 0 ||
      !PositiveFinite(config.max_dt_s) || config.esekf_iterations <= 0 ||
      config.esekf_iterations > 20 || !PositiveFinite(config.min_radius_m) ||
      config.max_radius_m <= config.min_radius_m ||
      config.vehicle_initial_radius_m <= config.min_radius_m ||
      config.vehicle_initial_radius_m >= config.max_radius_m ||
      config.hero_initial_radius_m <= config.min_radius_m ||
      config.hero_initial_radius_m >= config.max_radius_m ||
      !std::isfinite(config.vehicle_armor_tilt_rad) || !std::isfinite(config.hero_armor_tilt_rad) ||
      !PositiveFinite(config.max_height_offset_m) ||
      !PositiveFinite(config.max_yaw_velocity_rad_s) || config.visible_slot_count < 1 ||
      config.visible_slot_count > 4 || !PositiveFinite(config.association_initial_gate) ||
      !PositiveFinite(config.association_gate) ||
      !PositiveFinite(config.light_dedup_center_length_ratio) ||
      !PositiveFinite(config.light_dedup_angle_gate_rad) ||
      !PositiveFinite(config.light_dedup_log_length_gate) ||
      !PositiveFinite(config.light_match_log_length_gate) ||
      !PositiveFinite(config.light_match_angle_gate_rad) ||
      !PositiveFinite(config.light_match_endpoint_distance_length_ratio) ||
      !PositiveFinite(config.maneuver_association_cost_trigger) ||
      config.maneuver_confirmation_frames < 2 || config.maneuver_confirmation_frames > 5 ||
      !PositiveFinite(config.maneuver_confirmation_window_s) ||
      !PositiveFinite(config.maneuver_hold_s) ||
      !PositiveFinite(config.maneuver_trigger_yaw_acceleration_variance) ||
      !PositiveFinite(config.maneuver_active_yaw_acceleration_variance) ||
      !PositiveFinite(config.maneuver_recovery_gate) ||
      !PositiveFinite(config.maneuver_nis_per_dof_gate) ||
      !PositiveFinite(config.uvl_center_sigma_length_ratio) ||
      !PositiveFinite(config.uvl_length_sigma_length_ratio) ||
      !PositiveFinite(config.uvl_angle_sigma_rad) ||
      !PositiveFinite(config.standalone_uvl_center_sigma_length_ratio) ||
      !PositiveFinite(config.standalone_uvl_length_sigma_length_ratio) ||
      !PositiveFinite(config.standalone_uvl_angle_sigma_rad) ||
      !PositiveFinite(config.depth_difference_sigma_m)) {
    throw ConfigError("armor predictor config contains an invalid scalar range");
  }
  if (config.maneuver_trigger_yaw_acceleration_variance <
          config.maneuver_active_yaw_acceleration_variance ||
      config.maneuver_active_yaw_acceleration_variance < config.yaw_acceleration_variance ||
      config.maneuver_recovery_gate <= config.association_gate ||
      config.maneuver_recovery_gate > config.association_initial_gate ||
      config.maneuver_association_cost_trigger >= config.association_gate) {
    throw ConfigError("armor predictor maneuver parameters are inconsistent with steady gates");
  }
  for (double value : config.initial_covariance_diagonal) {
    if (!PositiveFinite(value))
      throw ConfigError("armor predictor initial covariance must be positive and finite");
  }
  for (double value : config.body_acceleration_variance) {
    if (!std::isfinite(value) || value < 0.0)
      throw ConfigError("armor predictor acceleration variances must be nonnegative and finite");
  }
  for (double value :
       {config.yaw_acceleration_variance, config.roll_pitch_random_walk_variance_per_s,
        config.radius_random_walk_variance_m2_per_s, config.height_random_walk_variance_m2_per_s,
        config.association_center_weight, config.association_edge_angle_weight,
        config.association_perimeter_ratio_weight, config.light_match_position_weight,
        config.light_match_angle_weight, config.light_match_length_weight}) {
    if (!std::isfinite(value) || value < 0.0)
      throw ConfigError("armor predictor noise and weight values must be nonnegative and finite");
  }
  if (!std::is_sorted(config.prediction_horizons_s.begin(), config.prediction_horizons_s.end()) ||
      config.prediction_horizons_s.front() < 0.0 ||
      !std::all_of(config.prediction_horizons_s.begin(), config.prediction_horizons_s.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw ConfigError(
        "armor predictor prediction_horizons_s must be finite, nonnegative and sorted");
  }
  return config;
}

}  // namespace mv::modules
