#pragma once

#include "frame/frame_types.hpp"
#include "modules/armor_pnp/armor_pnp_config.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"

#include <span>

namespace mv::modules {

/** @brief 可由正式链和评估工具复用的无状态 IPPE 装甲位姿求解器。 */
class ArmorPnpSolver final {
 public:
  explicit ArmorPnpSolver(ArmorPnpConfig config);

  [[nodiscard]] ArmorPnpSolveResult Solve(std::span<const cv::Point2f, 4> image_corners,
                                          geometry::ArmorType type,
                                          const frame::CameraModel& camera_model,
                                          std::size_t input_index, std::uint8_t label = 0) const;

 private:
  ArmorPnpConfig config_;
};

}  // namespace mv::modules
