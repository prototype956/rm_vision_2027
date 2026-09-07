#pragma once

#include "simulation/combat_frame_data.hpp"

#include <cstdint>
#include <string>

#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::simulation {
/** @brief 编码自身裁判采样或独立仿真评估；两个通道都保留来源图像及采样时间。 */
[[nodiscard]] std::string EncodeCombat(const mv::simulation::CombatFrameMeta& combat,
                                       std::uint64_t sequence,
                                       const ::foxglove::schemas::Timestamp& timestamp,
                                       bool referee_only);
/** @brief 两个固定话题的完整 JSON Schema，供 Foxglove 展开字段及绘图。 */
[[nodiscard]] std::string CombatSchema(bool referee_only);
}  // namespace mv::tool::foxglove::simulation
