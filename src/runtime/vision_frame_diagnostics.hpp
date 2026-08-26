#pragma once

#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_light_detector/armor_light_detector.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"

#include <vector>

namespace mv::runtime {

/** @brief 当前帧只交给评估、日志、窗口和可视化的诊断数据。 */
struct VisionFrameDiagnostics {
  modules::ArmorDetectorDiagnostics detector;
  std::vector<modules::CornerRefinementDiagnostics> refinements;
  modules::LightbarDetectorDiagnostics lightbars;
  modules::ArmorPnpDiagnostics pnp;
  modules::ArmorPredictionDiagnostics prediction;
};

}  // namespace mv::runtime
