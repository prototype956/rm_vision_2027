#include "modules/armor_pnp/armor_pnp.hpp"

#include "modules/armor_pnp/armor_pnp_solver.hpp"
#include "modules/armor_pnp/detail/pnp_metrics.hpp"

#include <utility>

namespace mv::modules {

struct ArmorPnp::Impl {
  explicit Impl(ArmorPnpConfig config) : solver(config) {}

  ArmorPnpSolver solver;
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

geometry::ArmorType ArmorTypeForLabel(ArmorLabel label) noexcept {
  return label == ArmorLabel::ONE || label == ArmorLabel::BASE_BIG ? geometry::ArmorType::LARGE
                                                                   : geometry::ArmorType::SMALL;
}

ArmorPnp::ArmorPnp(ArmorPnpConfig config) : impl_(std::make_unique<Impl>(config)) {}

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

ArmorPnpFrameResult ArmorPnp::ProcessFrame(std::uint64_t sequence,
                                           const frame::CameraModel& camera_model,
                                           std::span<const ArmorDetection> detections,
                                           std::span<const CornerRefinementResult> refinements) {
  ArmorPnpFrameResult result;
  if (refinements.size() != detections.size())
    return result;
  result.attempts.reserve(detections.size());
  for (std::size_t index = 0; index < detections.size(); ++index) {
    const auto& detection = detections[index];
    CornerRefinementResult refinement = refinements[index];
    impl_->metrics.RecordRefinement(refinement);
    const auto& final_corners =
        refinement.success ? refinement.refined_corners : refinement.original_corners;
    auto attempt =
        impl_->solver.Solve(final_corners, ArmorTypeForLabel(detection.label), camera_model, index,
                            static_cast<std::uint8_t>(detection.label));
    impl_->metrics.RecordDetectionSolve(attempt);
    attempt.refinement = std::move(refinement);
    result.attempts.push_back(std::move(attempt));
  }
  impl_->metrics.PopulateSnapshot(sequence, result);
  return result;
}

}  // namespace mv::modules
