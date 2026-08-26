#pragma once

#include "modules/armor_corner_refiner/armor_corner_refiner_output.hpp"
#include "modules/armor_detector/armor_detector_output.hpp"
#include "modules/armor_light_detector/armor_light_detector_output.hpp"
#include "modules/armor_pnp/armor_pnp_output.hpp"
#include "modules/armor_predictor/armor_prediction_output.hpp"

#include <vector>

namespace mv::runtime {

/** @brief 当前帧沿正式算法、预测和控制链流动的数据。 */
struct VisionFrameOutput {
  std::vector<modules::ArmorDetection> detections;
  std::vector<modules::CornerRefinementOutput> refinements;
  modules::LightbarDetectorOutput lightbars;
  modules::ArmorPnpOutput pnp;
  modules::ArmorPredictionOutput prediction;
};

}  // namespace mv::runtime
