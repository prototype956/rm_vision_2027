#pragma once

#include <array>

#include <opencv2/core.hpp>

namespace mv::modules {

/** @brief 下游算法消费的最终装甲四角。 */
struct CornerRefinementOutput {
  std::array<cv::Point2f, 4> corners{};  ///< TL、TR、BR、BL 顺序的原子提交结果。
  bool refined{false};                   ///< true 表示全部四角均采用精修结果。
};

}  // namespace mv::modules
