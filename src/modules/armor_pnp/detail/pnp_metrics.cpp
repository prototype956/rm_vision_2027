#include "modules/armor_pnp/detail/pnp_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mv::modules::detail {
namespace {

PnpPercentiles Percentiles(const std::vector<double>& samples) {
  if (samples.empty())
    return {};
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  // 使用最近秩定义，输出值始终等于一个实际观测样本。
  const auto AT = [&](double fraction) {
    const auto INDEX =
        static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(sorted.size())) - 1.0);
    return sorted[std::min(INDEX, sorted.size() - 1)];
  };
  return {.samples = sorted.size(), .p50 = AT(0.50), .p95 = AT(0.95)};
}

PnpSourceSummary Summarize(const PnpMetricSamples& samples) {
  return {.reprojection_rmse_px = Percentiles(samples.reprojection),
          .mean_corner_error_px = Percentiles(samples.corner),
          .position_error_m = Percentiles(samples.position),
          .depth_error_m = Percentiles(samples.depth),
          .rotation_error_deg = Percentiles(samples.rotation),
          .position_jitter_m = Percentiles(samples.jitter)};
}

std::string DistanceGroup(double distance) {
  if (distance < 3.0)
    return "lt_3m";
  if (distance < 5.0)
    return "3_5m";
  if (distance < 7.0)
    return "5_7m";
  if (distance < 9.0)
    return "7_9m";
  return "ge_9m";
}

std::string AngleGroup(double angle) {
  if (angle < 10.0)
    return "lt_10deg";
  if (angle < 20.0)
    return "10_20deg";
  if (angle < 30.0)
    return "20_30deg";
  if (angle < 45.0)
    return "30_45deg";
  return "ge_45deg";
}

std::string SizeGroup(hal::CameraFrame::ArmorType type) {
  return type == hal::CameraFrame::ArmorType::LARGE ? "large" : "small";
}

void AddSamples(PnpMetricSamples& samples, const ArmorPoseEstimate& estimate) {
  samples.reprojection.push_back(estimate.reprojection_rmse_px);
  if (estimate.mean_corner_error_px)
    samples.corner.push_back(*estimate.mean_corner_error_px);
  if (estimate.position_error_m)
    samples.position.push_back(*estimate.position_error_m);
  if (estimate.depth_error_m)
    samples.depth.push_back(*estimate.depth_error_m);
  if (estimate.rotation_error_deg)
    samples.rotation.push_back(*estimate.rotation_error_deg);
  if (estimate.position_jitter_m)
    samples.jitter.push_back(*estimate.position_jitter_m);
}

std::map<std::string, PnpSourceSummary> SummarizeGroups(
    const std::map<std::string, PnpMetricSamples>& groups) {
  std::map<std::string, PnpSourceSummary> result;
  for (const auto& [name, samples] : groups)
    result.emplace(name, Summarize(samples));
  return result;
}

}  // namespace

void PnpMetrics::RecordRefinement(const CornerRefinementResult& refinement) {
  ++refinement_summary_.attempted;
  refinement_elapsed_samples_.push_back(refinement.elapsed_ms);
  if (refinement.success && !refinement.fallback) {
    ++refinement_summary_.succeeded;
  } else {
    ++refinement_summary_.fallback;
    ++refinement_summary_.failure_reasons[CornerRefinementStatusName(refinement.status)];
  }
}

void PnpMetrics::RecordMatchedCorners(const CornerRefinementResult& refinement,
                                      const std::array<cv::Point2f, 4>& final_corners,
                                      const std::array<cv::Point2f, 4>& truth_corners) {
  double raw_error = 0.0;
  double final_error = 0.0;
  for (std::size_t corner = 0; corner < 4; ++corner) {
    raw_error += cv::norm(refinement.original_corners[corner] - truth_corners[corner]);
    final_error += cv::norm(final_corners[corner] - truth_corners[corner]);
  }
  raw_corner_error_samples_.push_back(raw_error / 4.0);
  final_corner_error_samples_.push_back(final_error / 4.0);
}

void PnpMetrics::RecordDetectionSolve(const ArmorPnpAttempt& attempt) {
  ++solve_summary_.attempted;
  if (attempt.estimate) {
    ++solve_summary_.succeeded;
  } else {
    ++solve_summary_.rejection_reasons[PnpStatusName(attempt.status)];
  }
}

void PnpMetrics::RecordAttempts(std::span<ArmorPnpAttempt> attempts, std::uint64_t sequence) {
  for (auto& attempt : attempts) {
    if (!attempt.estimate)
      continue;
    auto& estimate = *attempt.estimate;
    if (estimate.truth_id && estimate.position_error_camera_m) {
      const auto values = *estimate.position_error_camera_m;
      const geometry::Vector3 error(values[0], values[1], values[2]);
      const auto key = std::make_pair(attempt.source, *estimate.truth_id);
      const auto previous = previous_error_.find(key);
      const auto previous_sequence = previous_sequence_.find(key);
      // 只比较同一真值目标的连续帧，目标消失后重现不会产生伪抖动或伪候选切换。
      const bool consecutive =
          previous_sequence != previous_sequence_.end() &&
          previous_sequence->second != std::numeric_limits<std::uint64_t>::max() &&
          previous_sequence->second + 1 == sequence;
      if (previous != previous_error_.end() && consecutive)
        estimate.position_jitter_m = (error - previous->second).norm();
      previous_error_[key] = error;
      const auto previous_candidate = previous_candidate_.find(key);
      if (previous_candidate != previous_candidate_.end() && consecutive &&
          previous_candidate->second != estimate.candidate_index &&
          attempt.source != PnpInputSource::GROUND_TRUTH) {
        ++solve_summary_.candidate_switches;
      }
      previous_candidate_[key] = estimate.candidate_index;
      previous_sequence_[key] = sequence;
    }
    if (attempt.source == PnpInputSource::GROUND_TRUTH) {
      AddSamples(truth_samples_, estimate);
    } else {
      AddSamples(detection_samples_, estimate);
      if (estimate.truth_distance_m && estimate.truth_viewing_angle_deg && estimate.truth_type) {
        AddSamples(distance_samples_[DistanceGroup(*estimate.truth_distance_m)], estimate);
        AddSamples(angle_samples_[AngleGroup(*estimate.truth_viewing_angle_deg)], estimate);
        AddSamples(size_samples_[SizeGroup(*estimate.truth_type)], estimate);
      }
    }
  }
}

void PnpMetrics::PopulateSnapshot(std::uint64_t sequence, ArmorPnpFrameResult& result) {
  // 全局、分组、求解和精修摘要共用同一序号，消费者不会读到跨周期混合结果。
  if (!summary_initialized_ || sequence % 100 == 0) {
    summary_initialized_ = true;
    summary_sequence_ = sequence;
    truth_summary_ = Summarize(truth_samples_);
    detection_summary_ = Summarize(detection_samples_);
    distance_summaries_ = SummarizeGroups(distance_samples_);
    angle_summaries_ = SummarizeGroups(angle_samples_);
    size_summaries_ = SummarizeGroups(size_samples_);
    refinement_summary_.elapsed_ms = Percentiles(refinement_elapsed_samples_);
    refinement_summary_.raw_mean_corner_error_px = Percentiles(raw_corner_error_samples_);
    refinement_summary_.final_mean_corner_error_px = Percentiles(final_corner_error_samples_);
    solve_snapshot_ = solve_summary_;
    refinement_snapshot_ = refinement_summary_;
  }
  result.summary_sequence = summary_sequence_;
  result.ground_truth_summary = truth_summary_;
  result.detection_summary = detection_summary_;
  result.distance_groups = distance_summaries_;
  result.angle_groups = angle_summaries_;
  result.size_groups = size_summaries_;
  result.solve_summary = solve_snapshot_;
  result.refinement_summary = refinement_snapshot_;
}

}  // namespace mv::modules::detail
