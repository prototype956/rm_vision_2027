#include "tool/simulation_evaluation/simulation_evaluation_config.hpp"

#include "core/config.hpp"

namespace mv::tool::simulation_evaluation {

SimulationEvaluationConfig ParseSimulationEvaluationConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "simulation evaluation config";
  ConfigLoader::RejectUnknownKeys(
      root,
      {"schema_version", "truth_match_min_iou", "truth_match_max_center_distance_ratio",
       "truth_match_max_corner_distance_ratio"},
      CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1)
    throw ConfigError("simulation evaluation config schema_version must be 1");
  SimulationEvaluationConfig config{
      .truth_match_min_iou = ConfigLoader::Require<double>(root, "truth_match_min_iou", CONTEXT),
      .truth_match_max_center_distance_ratio =
          ConfigLoader::Require<double>(root, "truth_match_max_center_distance_ratio", CONTEXT),
      .truth_match_max_corner_distance_ratio =
          ConfigLoader::Require<double>(root, "truth_match_max_corner_distance_ratio", CONTEXT)};
  if (!(config.truth_match_min_iou >= 0.0 && config.truth_match_min_iou <= 1.0 &&
        config.truth_match_max_center_distance_ratio > 0.0 &&
        config.truth_match_max_corner_distance_ratio > 0.0)) {
    throw ConfigError("simulation evaluation truth matching thresholds are invalid");
  }
  return config;
}

}  // namespace mv::tool::simulation_evaluation
