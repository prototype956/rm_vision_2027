#include "modules/armor_pnp/armor_pnp.hpp"

#include "modules/armor_pnp/detail/ippe_solver.hpp"
#include "modules/armor_pnp/detail/pnp_metrics.hpp"
#include "modules/armor_pnp/detail/truth_evaluator.hpp"

#include <utility>
#include <vector>

namespace mv::modules {

struct ArmorPnp::Impl {
  explicit Impl(ArmorPnpConfig value) : config(std::move(value)) {}

  [[nodiscard]] ArmorPnpAttempt Solve(std::span<const cv::Point2f, 4> image_corners,
                                      hal::CameraFrame::ArmorType type,
                                      const hal::CameraFrame::Calibration& calibration,
                                      PnpInputSource source, std::size_t input_index,
                                      std::uint8_t label) const;
  [[nodiscard]] ArmorPnpFrameResult ProcessFrame(
      const hal::CameraFrame& frame, std::span<const ArmorDetection> detections,
      std::span<const CornerRefinementResult> refinements);

  ArmorPnpConfig config;
  detail::PnpMetrics metrics;
};

const char* PnpStatusName(PnpStatus status) noexcept {
  switch (status) {
    case PnpStatus::SUCCESS:
      return "success";
    case PnpStatus::INVALID_INPUT:
      return "invalid_input";
    case PnpStatus::NO_SOLUTION:
      return "no_solution";
    case PnpStatus::NEGATIVE_DEPTH:
      return "negative_depth";
    case PnpStatus::BACK_FACING:
      return "back_facing";
    case PnpStatus::OUT_OF_RANGE:
      return "out_of_range";
  }
  return "unknown";
}

const char* PnpInputSourceName(PnpInputSource source) noexcept {
  switch (source) {
    case PnpInputSource::GROUND_TRUTH:
      return "ground_truth";
    case PnpInputSource::DETECTION:
      return "detection";
  }
  return "unknown";
}

hal::CameraFrame::ArmorType ArmorTypeForLabel(ArmorLabel label) noexcept {
  return label == ArmorLabel::ONE || label == ArmorLabel::BASE_BIG
             ? hal::CameraFrame::ArmorType::LARGE
             : hal::CameraFrame::ArmorType::SMALL;
}

ArmorPnpAttempt ArmorPnp::Impl::Solve(std::span<const cv::Point2f, 4> image_corners,
                                      hal::CameraFrame::ArmorType type,
                                      const hal::CameraFrame::Calibration& calibration,
                                      PnpInputSource source, std::size_t input_index,
                                      std::uint8_t label) const {
  return detail::SolveIppe(config, image_corners, type, calibration, source, input_index, label);
}

ArmorPnpFrameResult ArmorPnp::Impl::ProcessFrame(
    const hal::CameraFrame& frame, std::span<const ArmorDetection> detections,
    std::span<const CornerRefinementResult> refinements) {
  ArmorPnpFrameResult result;
  if (!frame.geometry || refinements.size() != detections.size())
    return result;
  const auto& geometry = *frame.geometry;
  std::vector<detail::VisibleArmorTruth> visible_truth;
  // 真值投影复用正式 IPPE 求解器，用来验证物点顺序、坐标约定和数值基线。
  for (std::size_t index = 0; index < geometry.armors.size(); ++index) {
    const auto pixels = detail::ProjectVisibleTruth(geometry.armors[index], geometry);
    if (!pixels)
      continue;
    visible_truth.push_back({&geometry.armors[index], *pixels});
    auto attempt = Solve(*pixels, geometry.armors[index].type, geometry.calibration,
                         PnpInputSource::GROUND_TRUTH, index, geometry.armors[index].label);
    if (attempt.estimate)
      detail::AddTruthErrors(*attempt.estimate, geometry.armors[index], geometry, *pixels);
    result.attempts.push_back(std::move(attempt));
  }

  // 全局一对一匹配只服务误差统计；未匹配检测仍正常输出正式 PnP 位姿。
  const auto truth_matches = detail::MatchDetectionsToTruth(detections, visible_truth, config);
  for (std::size_t index = 0; index < detections.size(); ++index) {
    const auto& detection = detections[index];
    CornerRefinementResult refinement = refinements[index];
    metrics.RecordRefinement(refinement);
    // 每个检测只解算一次：精修成功采用全部精修角点，否则原子回退全部网络角点。
    const auto& final_corners =
        refinement.success ? refinement.refined_corners : refinement.original_corners;
    auto detection_attempt =
        Solve(final_corners, ArmorTypeForLabel(detection.label), geometry.calibration,
              PnpInputSource::DETECTION, index, static_cast<std::uint8_t>(detection.label));
    metrics.RecordDetectionSolve(detection_attempt);

    const std::size_t best_truth = truth_matches[index];
    if (best_truth < visible_truth.size()) {
      const auto& truth = visible_truth[best_truth];
      metrics.RecordMatchedCorners(refinement, final_corners, truth.pixels);
      if (detection_attempt.estimate)
        detail::AddTruthErrors(*detection_attempt.estimate, *truth.armor, geometry, truth.pixels);
    }
    detection_attempt.refinement = std::move(refinement);
    result.attempts.push_back(std::move(detection_attempt));
  }

  // 先补充本帧连续性指标，再生成快照，保证快照包含当前帧的全部有效样本。
  metrics.RecordAttempts(result.attempts, frame.sequence);
  metrics.PopulateSnapshot(frame.sequence, result);
  return result;
}

ArmorPnp::ArmorPnp(ArmorPnpConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}

ArmorPnp::~ArmorPnp() = default;

ArmorPnp::ArmorPnp(const ArmorPnp& other)
    : impl_(other.impl_ ? std::make_unique<Impl>(*other.impl_) : nullptr) {}

ArmorPnp& ArmorPnp::operator=(const ArmorPnp& other) {
  if (this != &other)
    impl_ = other.impl_ ? std::make_unique<Impl>(*other.impl_) : nullptr;
  return *this;
}

ArmorPnp::ArmorPnp(ArmorPnp&& other) noexcept = default;

ArmorPnp& ArmorPnp::operator=(ArmorPnp&& other) noexcept = default;

ArmorPnpAttempt ArmorPnp::Solve(std::span<const cv::Point2f, 4> image_corners,
                                hal::CameraFrame::ArmorType type,
                                const hal::CameraFrame::Calibration& calibration,
                                PnpInputSource source, std::size_t input_index,
                                std::uint8_t label) const {
  return impl_->Solve(image_corners, type, calibration, source, input_index, label);
}

ArmorPnpFrameResult ArmorPnp::ProcessFrame(const hal::CameraFrame& frame,
                                           std::span<const ArmorDetection> detections,
                                           std::span<const CornerRefinementResult> refinements) {
  return impl_->ProcessFrame(frame, detections, refinements);
}

}  // namespace mv::modules
