#pragma once

#include <yaml-cpp/yaml.h>

namespace mv::modules {

inline constexpr int ARMOR_LIGHT_DETECTOR_CONFIG_SCHEMA_VERSION = 1;

/** @brief 全图传统 CV 独立灯条检测参数。 */
struct ArmorLightDetectorConfig {
  bool enabled{true};
  int fixed_binary_threshold{120};
  int network_reference_offset{50};
  int minimum_binary_threshold{60};
  int maximum_binary_threshold{240};
  int minimum_contour_points{6};
  double minimum_contour_area_px2{2.0};
  double minimum_length_px{3.0};
  double minimum_width_length_ratio{0.02};
  double maximum_width_length_ratio{0.40};
  double maximum_tilt_rad{0.70};
  double minimum_color_difference{20.0};
  int maximum_candidates{64};
};

/** @brief 解析并严格校验独立灯条检测配置。 */
[[nodiscard]] ArmorLightDetectorConfig ParseArmorLightDetectorConfig(const YAML::Node& root);

}  // namespace mv::modules
