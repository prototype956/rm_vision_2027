#pragma once

#include <array>
#include <vector>

#include <opencv2/core.hpp>

namespace mv::modules {

enum class ArmorColor { RED, BLUE };

enum class ArmorLabel { SENTRY, ONE, TWO, THREE, FOUR, FIVE, OUTPOST, BASE_SMALL, BASE_BIG };

[[nodiscard]] const char* ArmorColorName(ArmorColor color) noexcept;
[[nodiscard]] const char* ArmorLabelName(ArmorLabel label) noexcept;

/** @brief 单个装甲板的正式检测输出。 */
struct ArmorDetection {
  ArmorColor color{ArmorColor::RED};
  ArmorLabel label{ArmorLabel::SENTRY};
  float objectness{0.0F};
  cv::Rect2f bounding_box;
  std::array<cv::Point2f, 4> corners{};
};

/** @brief 当前帧检测器产生的正式候选集合。 */
struct ArmorDetectorOutput {
  std::vector<ArmorDetection> detections;
};

}  // namespace mv::modules
