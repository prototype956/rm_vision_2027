#pragma once

#include "modules/armor_predictor/armor_prediction_types.hpp"
#include "modules/armor_predictor/armor_predictor_config.hpp"
#include "modules/armor_predictor/detail/armor_predictor_internal.hpp"

#include <vector>

#include <span>

namespace mv::modules::detail {

/**
 * @brief 在位置和 yaw 门限内执行观测到四个槽位的全局一对一关联。
 * @return 与 observations 同顺序的槽位编号；未关联项为 -1。
 */
[[nodiscard]] std::vector<int> AssociateArmors(std::span<const Observation> observations,
                                               const StateVector& state,
                                               const ArmorPredictorConfig& config,
                                               std::vector<ArmorAssociation>& diagnostics);

}  // namespace mv::modules::detail
