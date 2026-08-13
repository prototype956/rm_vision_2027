#pragma once

#include "modules/armor_pnp/armor_pnp_config.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"

#include <array>
#include <vector>

#include <optional>
#include <span>

namespace mv::modules::detail {

/** @brief 一块通过可见性过滤的仿真装甲及其同帧图像投影。 */
struct VisibleArmorTruth {
  const hal::CameraFrame::GroundTruthArmor* armor{nullptr};  ///< 指向当前 FrameGeometry 的真值。
  std::array<cv::Point2f, 4> pixels{};  ///< TL、TR、BR、BL 顺序的无畸变投影点。
};

/** @brief 投影正面朝向相机、四角位于相机前方且与图像相交的仿真装甲。 */
[[nodiscard]] std::optional<std::array<cv::Point2f, 4>> ProjectVisibleTruth(
    const hal::CameraFrame::GroundTruthArmor& armor,
    const hal::CameraFrame::FrameGeometry& geometry);

/**
 * @brief 按队伍、标签和几何门限构造代价，并执行检测到可见真值的 Hungarian 一对一匹配。
 * @return 与 detections 同顺序的真值索引；未匹配项等于 truths.size()。
 */
[[nodiscard]] std::vector<std::size_t> MatchDetectionsToTruth(
    std::span<const ArmorDetection> detections, std::span<const VisibleArmorTruth> truths,
    const ArmorPnpConfig& config);

/** @brief 将 estimate - truth 定义的二维、位置、深度及姿态误差写入估计结果。 */
void AddTruthErrors(ArmorPoseEstimate& estimate, const hal::CameraFrame::GroundTruthArmor& truth,
                    const hal::CameraFrame::FrameGeometry& geometry,
                    const std::array<cv::Point2f, 4>& truth_pixels);

}  // namespace mv::modules::detail
