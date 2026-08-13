#include "modules/armor_pnp/armor_pnp_config.hpp"

#include "core/config.hpp"

namespace mv::modules {

ArmorPnpConfig ParseArmorPnpConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "armor PnP config";
  ConfigLoader::RejectUnknownKeys(
      root,
      {"schema_version", "small_width_m", "large_width_m", "height_m", "min_distance_m",
       "max_distance_m", "truth_match_min_iou", "truth_match_max_center_distance_ratio",
       "truth_match_max_corner_distance_ratio"},
      CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 2)
    throw ConfigError("armor PnP config schema_version must be 2");
  ArmorPnpConfig config{
      .small_width_m = ConfigLoader::Require<double>(root, "small_width_m", CONTEXT),
      .large_width_m = ConfigLoader::Require<double>(root, "large_width_m", CONTEXT),
      .height_m = ConfigLoader::Require<double>(root, "height_m", CONTEXT),
      .min_distance_m = ConfigLoader::Require<double>(root, "min_distance_m", CONTEXT),
      .max_distance_m = ConfigLoader::Require<double>(root, "max_distance_m", CONTEXT),
      .truth_match_min_iou = ConfigLoader::Require<double>(root, "truth_match_min_iou", CONTEXT),
      .truth_match_max_center_distance_ratio =
          ConfigLoader::Require<double>(root, "truth_match_max_center_distance_ratio", CONTEXT),
      .truth_match_max_corner_distance_ratio =
          ConfigLoader::Require<double>(root, "truth_match_max_corner_distance_ratio", CONTEXT)};
  if (!(config.small_width_m > 0.0 && config.large_width_m > config.small_width_m &&
        config.height_m > 0.0 && config.min_distance_m > 0.0 &&
        config.max_distance_m > config.min_distance_m && config.truth_match_min_iou >= 0.0 &&
        config.truth_match_min_iou <= 1.0 && config.truth_match_max_center_distance_ratio > 0.0 &&
        config.truth_match_max_corner_distance_ratio > 0.0)) {
    throw ConfigError("armor PnP dimensions or distance range are invalid");
  }
  return config;
}

}  // namespace mv::modules
