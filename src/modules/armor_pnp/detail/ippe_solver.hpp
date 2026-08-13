#pragma once

#include "modules/armor_pnp/armor_pnp_config.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"

#include <span>

namespace mv::modules::detail {

/**
 * @brief 运行平面 IPPE，按正深度、正面朝向和距离范围过滤候选并选择最小 RMSE 解。
 * @return 包含最终状态及可选最优姿态的单次解算结果。
 */
[[nodiscard]] ArmorPnpAttempt SolveIppe(const ArmorPnpConfig& config,
                                        std::span<const cv::Point2f, 4> image_corners,
                                        hal::CameraFrame::ArmorType type,
                                        const hal::CameraFrame::Calibration& calibration,
                                        PnpInputSource source, std::size_t input_index,
                                        std::uint8_t label);

}  // namespace mv::modules::detail
