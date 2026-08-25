#include "modules/armor_pnp/detail/pnp_metrics.hpp"

#include <algorithm>
#include <cmath>

namespace mv::modules::detail {
namespace {

PnpPercentiles Percentiles(const std::vector<double>& samples) {
  if (samples.empty())
    return {};
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  const auto AT = [&](double fraction) {
    const auto INDEX =
        static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(sorted.size())) - 1.0);
    return sorted[std::min(INDEX, sorted.size() - 1)];
  };
  return {.samples = sorted.size(), .p50 = AT(0.50), .p95 = AT(0.95)};
}

}  // namespace

void PnpMetrics::RecordRefinement(const CornerRefinementDiagnostics& refinement) {
  ++refinement_summary_.attempted;
  refinement_elapsed_samples_.push_back(refinement.elapsed_ms);
  if (refinement.success && !refinement.fallback) {
    ++refinement_summary_.succeeded;
  } else {
    ++refinement_summary_.fallback;
    ++refinement_summary_.failure_reasons[CornerRefinementStatusName(refinement.status)];
  }
}

void PnpMetrics::RecordDetectionSolve(const ArmorPnpSolveResult& result) {
  ++solve_summary_.attempted;
  if (result.output && result.diagnostics.pose) {
    ++solve_summary_.succeeded;
    reprojection_samples_.push_back(result.diagnostics.pose->reprojection_rmse_px);
  } else {
    ++solve_summary_.rejection_reasons[PnpStatusName(result.diagnostics.status)];
  }
}

void PnpMetrics::PopulateSnapshot(std::uint64_t sequence, ArmorPnpDiagnostics& diagnostics) {
  if (!summary_initialized_ || sequence % 100 == 0) {
    summary_initialized_ = true;
    summary_sequence_ = sequence;
    detection_snapshot_.reprojection_rmse_px = Percentiles(reprojection_samples_);
    refinement_summary_.elapsed_ms = Percentiles(refinement_elapsed_samples_);
    solve_snapshot_ = solve_summary_;
    refinement_snapshot_ = refinement_summary_;
  }
  diagnostics.summary_sequence = summary_sequence_;
  diagnostics.detection_summary = detection_snapshot_;
  diagnostics.solve_summary = solve_snapshot_;
  diagnostics.refinement_summary = refinement_snapshot_;
}

}  // namespace mv::modules::detail
