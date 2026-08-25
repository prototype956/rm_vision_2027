#pragma once

#include <yaml-cpp/yaml.h>

namespace mv::modules {

/** @brief 正式 PnP 使用的装甲物理尺寸和有效距离。 */
struct ArmorPnpConfig {
  double small_width_m{0.135};  ///< 小装甲物点宽度，单位为米。
  double large_width_m{0.225};  ///< 大装甲物点宽度，单位为米。
  double height_m{0.055};       ///< 两种装甲共用的物点高度，单位为米。
  double min_distance_m{0.1};   ///< 接受 IPPE 候选的最小相机距离。
  double max_distance_m{30.0};  ///< 接受 IPPE 候选的最大相机距离。
};

/**
 * @brief 解析并严格校验装甲 PnP 配置。
 * @throws ConfigError Schema 版本错误、字段缺失、未知键或参数范围无效。
 */
[[nodiscard]] ArmorPnpConfig ParseArmorPnpConfig(const YAML::Node& root);

}  // namespace mv::modules
