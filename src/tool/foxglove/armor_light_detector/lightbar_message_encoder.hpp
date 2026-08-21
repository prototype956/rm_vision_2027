#pragma once

#include "modules/armor_light_detector/armor_light_detector.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"

#include <cstdint>
#include <string>

#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::armor_light_detector {

/** @brief 编码原始灯条、预测灯条以及接受、去重和拒绝状态。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeAnnotations(
    const modules::LightbarDetectionResult& detection,
    const modules::ArmorPredictionResult& prediction,
    const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 编码独立灯条检测和 ESEKF 融合的单帧 JSON 指标。 */
[[nodiscard]] std::string EncodeStats(const modules::LightbarDetectionResult& detection,
                                      const modules::ArmorPredictionResult& prediction,
                                      std::uint64_t sequence,
                                      const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::armor_light_detector
