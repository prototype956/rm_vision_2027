#pragma once

#include "geometry/armor_type.hpp"
#include "geometry/rigid_transform.hpp"
#include "modules/armor_detector/armor_detector_output.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mv::modules {

/** @brief 将正式分类标签映射到 PnP 使用的装甲物理尺寸。 */
[[nodiscard]] geometry::ArmorType ArmorTypeForLabel(ArmorLabel label) noexcept;

/** @brief 一个通过正式几何约束的装甲位姿。 */
struct ArmorPoseEstimate {
  std::size_t input_index{0};
  std::uint8_t label{0};
  geometry::ArmorType type{geometry::ArmorType::SMALL};
  double width_m{0.0};
  double height_m{0.0};
  geometry::RigidTransform camera_t_armor;
};

/** @brief 当前帧全部成功的正式 PnP 位姿。 */
struct ArmorPnpOutput {
  std::vector<ArmorPoseEstimate> estimates;
};

}  // namespace mv::modules
