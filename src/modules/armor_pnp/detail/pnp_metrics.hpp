#pragma once

#include "modules/armor_pnp/armor_pnp_types.hpp"

#include <vector>

namespace mv::modules::detail {

/** @brief 累计正式 PnP 与角点精修的真值无关健康指标。 */
class PnpMetrics final {
 public:
  void RecordRefinement(const CornerRefinementResult& refinement);
  void RecordDetectionSolve(const ArmorPnpAttempt& attempt);
  void PopulateSnapshot(std::uint64_t sequence, ArmorPnpFrameResult& result);

 private:
  PnpSolveSummary solve_summary_;
  CornerRefinementSummary refinement_summary_;
  PnpSolveSummary solve_snapshot_;
  CornerRefinementSummary refinement_snapshot_;
  std::vector<double> reprojection_samples_;
  std::vector<double> refinement_elapsed_samples_;
  PnpDetectionSummary detection_snapshot_;
  std::uint64_t summary_sequence_{0};
  bool summary_initialized_{false};
};

}  // namespace mv::modules::detail
