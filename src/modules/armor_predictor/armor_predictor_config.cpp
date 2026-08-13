#include "modules/armor_predictor/armor_predictor_config.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <numbers>

namespace mv::modules {
namespace {

bool Finite(double value) noexcept {
  return std::isfinite(value);
}

template <std::size_t N>
std::array<double, N> RequireDoubleArray(const YAML::Node& parent, const std::string& key,
                                         const std::string& context) {
  const auto NODE = parent[key];
  if (!NODE || !NODE.IsSequence() || NODE.size() != N) {
    throw ConfigError(context + "." + key + " must contain exactly " + std::to_string(N) +
                      " numbers");
  }
  std::array<double, N> result{};
  for (std::size_t index = 0; index < N; ++index) {
    try {
      result[index] = NODE[index].as<double>();
    } catch (const std::exception& error) {
      throw ConfigError(context + "." + key + " has invalid item: " + error.what());
    }
    if (!Finite(result[index])) {
      throw ConfigError(context + "." + key + " items must be finite");
    }
  }
  return result;
}

}  // namespace

ArmorPredictorConfig ParseArmorPredictorConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "armor predictor config";
  ConfigLoader::RejectUnknownKeys(root,
                                  {"schema_version", "tracking", "association", "model",
                                   "measurement", "priority", "prediction_horizons_s"},
                                  CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 2) {
    throw ConfigError("armor predictor config schema_version must be 2");
  }
  const auto TRACKING = root["tracking"];
  const auto ASSOCIATION = root["association"];
  const auto MODEL = root["model"];
  const auto MEASUREMENT = root["measurement"];
  const auto PRIORITY = root["priority"];
  ConfigLoader::RejectUnknownKeys(TRACKING, {"min_detect_count", "max_temp_lost_count", "max_dt_s"},
                                  "armor predictor config.tracking");
  ConfigLoader::RejectUnknownKeys(ASSOCIATION, {"max_position_m", "max_yaw_rad"},
                                  "armor predictor config.association");
  ConfigLoader::RejectUnknownKeys(
      MODEL,
      {"vehicle_initial_radius_m", "hero_initial_radius_m", "vehicle_armor_roll_rad",
       "hero_armor_roll_rad", "min_radius_m", "max_radius_m", "linear_acceleration_variance",
       "angular_acceleration_variance", "initial_covariance_diagonal"},
      "armor predictor config.model");
  ConfigLoader::RejectUnknownKeys(
      MEASUREMENT,
      {"variance_base", "distance_variance_angle_scale", "armor_yaw_variance_distance_scale"},
      "armor predictor config.measurement");
  ConfigLoader::RejectUnknownKeys(PRIORITY, {"sentry", "one", "two", "three", "four"},
                                  "armor predictor config.priority");

  ArmorPredictorConfig config;
  config.min_detect_count =
      ConfigLoader::Require<int>(TRACKING, "min_detect_count", "armor predictor config.tracking");
  config.max_temp_lost_count = ConfigLoader::Require<int>(TRACKING, "max_temp_lost_count",
                                                          "armor predictor config.tracking");
  config.max_dt_s =
      ConfigLoader::Require<double>(TRACKING, "max_dt_s", "armor predictor config.tracking");
  config.association_max_position_m = ConfigLoader::Require<double>(
      ASSOCIATION, "max_position_m", "armor predictor config.association");
  config.association_max_yaw_rad = ConfigLoader::Require<double>(
      ASSOCIATION, "max_yaw_rad", "armor predictor config.association");
  config.vehicle_initial_radius_m = ConfigLoader::Require<double>(MODEL, "vehicle_initial_radius_m",
                                                                  "armor predictor config.model");
  config.hero_initial_radius_m =
      ConfigLoader::Require<double>(MODEL, "hero_initial_radius_m", "armor predictor config.model");
  config.vehicle_armor_roll_rad = ConfigLoader::Require<double>(MODEL, "vehicle_armor_roll_rad",
                                                                "armor predictor config.model");
  config.hero_armor_roll_rad =
      ConfigLoader::Require<double>(MODEL, "hero_armor_roll_rad", "armor predictor config.model");
  config.min_radius_m =
      ConfigLoader::Require<double>(MODEL, "min_radius_m", "armor predictor config.model");
  config.max_radius_m =
      ConfigLoader::Require<double>(MODEL, "max_radius_m", "armor predictor config.model");
  config.linear_acceleration_variance = ConfigLoader::Require<double>(
      MODEL, "linear_acceleration_variance", "armor predictor config.model");
  config.angular_acceleration_variance = ConfigLoader::Require<double>(
      MODEL, "angular_acceleration_variance", "armor predictor config.model");
  config.initial_covariance_diagonal =
      RequireDoubleArray<11>(MODEL, "initial_covariance_diagonal", "armor predictor config.model");
  config.measurement_variance_base =
      RequireDoubleArray<4>(MEASUREMENT, "variance_base", "armor predictor config.measurement");
  config.distance_variance_angle_scale = ConfigLoader::Require<double>(
      MEASUREMENT, "distance_variance_angle_scale", "armor predictor config.measurement");
  config.armor_yaw_variance_distance_scale = ConfigLoader::Require<double>(
      MEASUREMENT, "armor_yaw_variance_distance_scale", "armor predictor config.measurement");
  config.label_priorities = {
      ConfigLoader::Require<int>(PRIORITY, "sentry", "armor predictor config.priority"),
      ConfigLoader::Require<int>(PRIORITY, "one", "armor predictor config.priority"),
      ConfigLoader::Require<int>(PRIORITY, "two", "armor predictor config.priority"),
      ConfigLoader::Require<int>(PRIORITY, "three", "armor predictor config.priority"),
      ConfigLoader::Require<int>(PRIORITY, "four", "armor predictor config.priority")};
  config.prediction_horizons_s = RequireDoubleArray<4>(root, "prediction_horizons_s", CONTEXT);

  if (config.min_detect_count <= 0 || config.max_temp_lost_count < 0 || config.max_dt_s <= 0.0 ||
      config.min_radius_m <= 0.0 || config.max_radius_m <= config.min_radius_m ||
      config.vehicle_initial_radius_m <= config.min_radius_m ||
      config.vehicle_initial_radius_m >= config.max_radius_m ||
      config.hero_initial_radius_m <= config.min_radius_m ||
      config.hero_initial_radius_m >= config.max_radius_m ||
      !Finite(config.vehicle_armor_roll_rad) || !Finite(config.hero_armor_roll_rad) ||
      std::abs(config.vehicle_armor_roll_rad) >= std::numbers::pi / 2.0 ||
      std::abs(config.hero_armor_roll_rad) >= std::numbers::pi / 2.0 ||
      config.association_max_position_m <= 0.0 || config.association_max_yaw_rad <= 0.0 ||
      config.linear_acceleration_variance < 0.0 || config.angular_acceleration_variance < 0.0 ||
      config.distance_variance_angle_scale < 0.0 ||
      config.armor_yaw_variance_distance_scale < 0.0) {
    throw ConfigError("armor predictor config contains an invalid scalar range");
  }
  if (!std::is_sorted(config.prediction_horizons_s.begin(), config.prediction_horizons_s.end()) ||
      config.prediction_horizons_s.front() < 0.0) {
    throw ConfigError("armor predictor prediction_horizons_s must be nonnegative and sorted");
  }
  for (double value : config.initial_covariance_diagonal) {
    if (value < 0.0)
      throw ConfigError("armor predictor initial covariance must be nonnegative");
  }
  for (double value : config.measurement_variance_base) {
    if (value <= 0.0)
      throw ConfigError("armor predictor measurement variances must be positive");
  }
  return config;
}

}  // namespace mv::modules
