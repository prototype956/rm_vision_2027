#pragma once

#include "modules/armor_detector/armor_detector_output.hpp"

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace mv::modules {

/** @brief 单根全图传统 CV 灯条正式检测输出。 */
struct LightbarDetection {
  std::size_t input_index{0};
  ArmorColor color{ArmorColor::RED};
  cv::Point2f top{};
  cv::Point2f bottom{};
  cv::Point2f center{};
  double length_px{0.0};
  double width_px{0.0};
  double angle_rad{0.0};
  double color_difference{0.0};
  double score{0.0};
};

struct LightbarDetectorOutput {
  std::vector<LightbarDetection> detections;
};

}  // namespace mv::modules
