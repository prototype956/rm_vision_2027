#pragma once

#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_light_detector/armor_light_detector.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"

#include <vector>
#include <string>

namespace mv::runtime {

/** @brief 当前帧只交给评估、日志、窗口和可视化的诊断数据。 */
struct VisionFrameDiagnostics {
  std::string real_geometry_status;  ///< 实机同步及占位外参诊断，不参与算法。
  modules::ArmorDetectorDiagnostics detector;
  std::vector<modules::CornerRefinementDiagnostics> refinements;
  modules::LightbarDetectorDiagnostics lightbars;
  modules::ArmorPnpDiagnostics pnp;
  modules::ArmorPredictionDiagnostics prediction;
};

}  // namespace mv::runtime
