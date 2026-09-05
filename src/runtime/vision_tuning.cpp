#include "runtime/vision_tuning.hpp"

#include <utility>

namespace mv::runtime {

VisionTuningMailbox::VisionTuningMailbox(VisionFrontendTuningConfig startup_config)
    : startup_config_(startup_config), active_config_(startup_config) {}

VisionTuningSubmitResult VisionTuningMailbox::Submit(std::uint64_t base_revision,
                                                     const VisionFrontendTuningConfig& config) {
  std::lock_guard lock(mutex_);
  if (pending_) {
    return {.status = VisionTuningSubmitStatus::PENDING_REQUEST};
  }
  if (base_revision != active_revision_) {
    return {.status = VisionTuningSubmitStatus::REVISION_CONFLICT};
  }
  const std::uint64_t REQUEST_ID = next_request_id_++;
  const std::uint64_t TARGET_REVISION = active_revision_ + 1;
  pending_ = VisionTuningRequest{REQUEST_ID, TARGET_REVISION, config};
  pending_taken_ = false;
  last_error_.clear();
  return {VisionTuningSubmitStatus::ACCEPTED, REQUEST_ID, TARGET_REVISION};
}

std::optional<VisionTuningRequest> VisionTuningMailbox::TryTakePending() {
  std::lock_guard lock(mutex_);
  if (!pending_ || pending_taken_) {
    return std::nullopt;
  }
  pending_taken_ = true;
  return pending_;
}

void VisionTuningMailbox::MarkApplied(const VisionTuningApplied& applied) {
  std::lock_guard lock(mutex_);
  if (!pending_ || pending_->request_id != applied.request_id) {
    return;
  }
  active_config_ = pending_->config;
  active_revision_ = pending_->target_revision;
  last_applied_sequence_ = applied.source_sequence;
  last_error_.clear();
  pending_.reset();
  pending_taken_ = false;
}

void VisionTuningMailbox::MarkFailed(std::uint64_t request_id, std::string message) {
  std::lock_guard lock(mutex_);
  if (!pending_ || pending_->request_id != request_id) {
    return;
  }
  last_error_ = std::move(message);
  pending_.reset();
  pending_taken_ = false;
}

VisionTuningSnapshot VisionTuningMailbox::Snapshot() const {
  std::lock_guard lock(mutex_);
  VisionTuningSnapshot snapshot{.startup_config = startup_config_,
                                .active_config = active_config_,
                                .active_revision = active_revision_,
                                .pending = pending_.has_value(),
                                .last_applied_sequence = last_applied_sequence_,
                                .last_error = last_error_};
  if (pending_) {
    snapshot.pending_request_id = pending_->request_id;
    snapshot.target_revision = pending_->target_revision;
  }
  return snapshot;
}

}  // namespace mv::runtime
