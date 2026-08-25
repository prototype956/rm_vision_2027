#pragma once

#include "geometry/armor_type.hpp"
#include "geometry/rigid_transform.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <optional>

namespace mv::modules {

/** @brief 一组输入四角的最终求解状态。 */
enum class PnpStatus : std::uint8_t {
  SUCCESS = 0,
  INVALID_INPUT,
  NO_SOLUTION,
  NEGATIVE_DEPTH,
  BACK_FACING,
  OUT_OF_RANGE,
};

/** @brief 将求解状态转换为稳定的日志与统计字段名称。 */
[[nodiscard]] const char* PnpStatusName(PnpStatus status) noexcept;

/** @brief 一个通过几何约束的正式装甲位姿估计。 */
struct ArmorPoseEstimate {
  std::size_t input_index{0};
  std::uint8_t label{0};
  geometry::ArmorType type{geometry::ArmorType::SMALL};
  double width_m{0.0};
  double height_m{0.0};
  geometry::RigidTransform camera_t_armor;
  std::array<cv::Point2f, 4> image_corners{};
  std::array<cv::Point2f, 4> reprojected_corners{};
  std::size_t candidate_index{0};
  std::optional<double> candidate_rmse_gap_px;
  double reprojection_rmse_px{0.0};
  double image_width_px{0.0};
  double image_height_px{0.0};
  double distance_m{0.0};
  double viewing_angle_deg{0.0};
};

/** @brief 正式检测角点的一次 PnP 尝试及其角点精修诊断。 */
struct ArmorPnpAttempt {
  std::size_t input_index{0};
  PnpStatus status{PnpStatus::INVALID_INPUT};
  std::optional<ArmorPoseEstimate> estimate;
  std::optional<CornerRefinementResult> refinement;
};

/** @brief 一组指标样本的数量、P50 与 P95 最近秩分位数。 */
struct PnpPercentiles {
  std::size_t samples{0};
  double p50{0.0};
  double p95{0.0};
};

/** @brief 正式 PnP 自洽重投影指标。 */
struct PnpDetectionSummary {
  PnpPercentiles reprojection_rmse_px;
};

/** @brief 角点精修累计计数和耗时。 */
struct CornerRefinementSummary {
  std::size_t attempted{0};
  std::size_t succeeded{0};
  std::size_t fallback{0};
  std::map<std::string, std::size_t> failure_reasons;
  PnpPercentiles elapsed_ms;
};

/** @brief 正式检测链的累计 PnP 求解健康状态。 */
struct PnpSolveSummary {
  std::size_t attempted{0};
  std::size_t succeeded{0};
  std::map<std::string, std::size_t> rejection_reasons;
};

/** @brief 当前帧正式检测 PnP 结果及累计健康快照。 */
struct ArmorPnpFrameResult {
  std::uint64_t summary_sequence{0};
  std::vector<ArmorPnpAttempt> attempts;
  PnpDetectionSummary detection_summary;
  PnpSolveSummary solve_summary;
  CornerRefinementSummary refinement_summary;
};

/** @brief 将检测标签映射到物点尺寸；ONE 与 BASE_BIG 使用大装甲。 */
[[nodiscard]] geometry::ArmorType ArmorTypeForLabel(ArmorLabel label) noexcept;

}  // namespace mv::modules
