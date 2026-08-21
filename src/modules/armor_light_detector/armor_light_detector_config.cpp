#include "modules/armor_light_detector/armor_light_detector_config.hpp"

#include "core/config.hpp"

#include <cmath>

namespace mv::modules {

ArmorLightDetectorConfig ParseArmorLightDetectorConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "armor light detector config";
  ConfigLoader::RejectUnknownKeys(
      root, {"schema_version", "enabled", "threshold", "geometry", "color", "maximum_candidates"},
      CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) !=
      ARMOR_LIGHT_DETECTOR_CONFIG_SCHEMA_VERSION) {
    throw ConfigError("armor light detector config schema_version must be 1");
  }

  const auto THRESHOLD = root["threshold"];
  ConfigLoader::RequireMap(THRESHOLD, "armor light detector threshold config");
  ConfigLoader::RejectUnknownKeys(THRESHOLD,
                                  {"fixed", "network_reference_offset", "minimum", "maximum"},
                                  "armor light detector threshold config");
  const auto GEOMETRY = root["geometry"];
  ConfigLoader::RequireMap(GEOMETRY, "armor light detector geometry config");
  ConfigLoader::RejectUnknownKeys(
      GEOMETRY,
      {"minimum_contour_points", "minimum_contour_area_px2", "minimum_length_px",
       "minimum_width_length_ratio", "maximum_width_length_ratio", "maximum_tilt_rad"},
      "armor light detector geometry config");
  const auto COLOR = root["color"];
  ConfigLoader::RequireMap(COLOR, "armor light detector color config");
  ConfigLoader::RejectUnknownKeys(COLOR, {"minimum_channel_difference"},
                                  "armor light detector color config");

  ArmorLightDetectorConfig config{
      .enabled = ConfigLoader::Require<bool>(root, "enabled", CONTEXT),
      .fixed_binary_threshold =
          ConfigLoader::Require<int>(THRESHOLD, "fixed", "armor light detector threshold config"),
      .network_reference_offset = ConfigLoader::Require<int>(
          THRESHOLD, "network_reference_offset", "armor light detector threshold config"),
      .minimum_binary_threshold =
          ConfigLoader::Require<int>(THRESHOLD, "minimum", "armor light detector threshold config"),
      .maximum_binary_threshold =
          ConfigLoader::Require<int>(THRESHOLD, "maximum", "armor light detector threshold config"),
      .minimum_contour_points = ConfigLoader::Require<int>(GEOMETRY, "minimum_contour_points",
                                                           "armor light detector geometry config"),
      .minimum_contour_area_px2 = ConfigLoader::Require<double>(
          GEOMETRY, "minimum_contour_area_px2", "armor light detector geometry config"),
      .minimum_length_px = ConfigLoader::Require<double>(GEOMETRY, "minimum_length_px",
                                                         "armor light detector geometry config"),
      .minimum_width_length_ratio = ConfigLoader::Require<double>(
          GEOMETRY, "minimum_width_length_ratio", "armor light detector geometry config"),
      .maximum_width_length_ratio = ConfigLoader::Require<double>(
          GEOMETRY, "maximum_width_length_ratio", "armor light detector geometry config"),
      .maximum_tilt_rad = ConfigLoader::Require<double>(GEOMETRY, "maximum_tilt_rad",
                                                        "armor light detector geometry config"),
      .minimum_color_difference = ConfigLoader::Require<double>(
          COLOR, "minimum_channel_difference", "armor light detector color config"),
      .maximum_candidates = ConfigLoader::Require<int>(root, "maximum_candidates", CONTEXT),
  };
  const bool FINITE_GEOMETRY =
      std::isfinite(config.minimum_contour_area_px2) && std::isfinite(config.minimum_length_px) &&
      std::isfinite(config.minimum_width_length_ratio) &&
      std::isfinite(config.maximum_width_length_ratio) && std::isfinite(config.maximum_tilt_rad) &&
      std::isfinite(config.minimum_color_difference);
  if (!FINITE_GEOMETRY || config.fixed_binary_threshold < 0 ||
      config.fixed_binary_threshold > 255 || config.network_reference_offset < 0 ||
      config.minimum_binary_threshold < 0 || config.maximum_binary_threshold > 255 ||
      config.minimum_binary_threshold > config.maximum_binary_threshold ||
      config.minimum_contour_points < 3 || config.minimum_contour_area_px2 < 0.0 ||
      config.minimum_length_px <= 0.0 || config.minimum_width_length_ratio < 0.0 ||
      config.maximum_width_length_ratio <= config.minimum_width_length_ratio ||
      config.maximum_width_length_ratio > 1.0 || config.maximum_tilt_rad <= 0.0 ||
      config.maximum_tilt_rad >= 1.5707963267948966 || config.minimum_color_difference < 0.0 ||
      config.maximum_candidates <= 0) {
    throw ConfigError("armor light detector config contains invalid values");
  }
  return config;
}

}  // namespace mv::modules
