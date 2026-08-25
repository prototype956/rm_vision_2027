#pragma once

#include "modules/armor_pnp/armor_pnp_config.hpp"
#include "tool/simulation_evaluation/simulation_evaluation_config.hpp"
#include "tool/simulation_evaluation/simulation_evaluation_types.hpp"

#include <memory>

namespace mv::tool::simulation_evaluation {

/** @brief 在正式结果生成后同步计算同帧仿真精度指标。 */
class SimulationEvaluator final {
 public:
  SimulationEvaluator(SimulationEvaluationConfig config, modules::ArmorPnpConfig pnp_config);
  ~SimulationEvaluator();

  SimulationEvaluator(const SimulationEvaluator&) = delete;
  SimulationEvaluator& operator=(const SimulationEvaluator&) = delete;
  SimulationEvaluator(SimulationEvaluator&&) = delete;
  SimulationEvaluator& operator=(SimulationEvaluator&&) = delete;

  [[nodiscard]] SimulationEvaluationResult Evaluate(const SimulationEvaluationInput& input);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::tool::simulation_evaluation
