#pragma once

#include <yaml-cpp/yaml.h>

namespace mv::tool::simulation_evaluation {

/** @brief 检测四边形与同帧仿真真值的一对一匹配门限。 */
struct SimulationEvaluationConfig {
  double truth_match_min_iou{0.05};
  double truth_match_max_center_distance_ratio{0.75};
  double truth_match_max_corner_distance_ratio{0.75};
};

/** @brief 解析并严格校验仿真评估配置。 */
[[nodiscard]] SimulationEvaluationConfig ParseSimulationEvaluationConfig(const YAML::Node& root);

}  // namespace mv::tool::simulation_evaluation
