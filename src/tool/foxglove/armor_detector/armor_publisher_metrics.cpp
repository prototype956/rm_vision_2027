#include "tool/foxglove/armor_detector/armor_publisher_metrics.hpp"

#include <algorithm>
#include <cmath>

namespace mv::tool::foxglove::armor_detector {
namespace {

double Percentile(std::vector<double> values, double quantile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double INDEX = quantile * static_cast<double>(values.size() - 1);
  const auto LOWER = static_cast<std::size_t>(std::floor(INDEX));
  const auto UPPER = static_cast<std::size_t>(std::ceil(INDEX));
  const double FRACTION = INDEX - static_cast<double>(LOWER);
  return values[LOWER] + (values[UPPER] - values[LOWER]) * FRACTION;
}

LatencyPercentiles Summarize(const std::vector<double>& values) {
  return {.p50_ms = Percentile(values, 0.50),
          .p95_ms = Percentile(values, 0.95),
          .p99_ms = Percentile(values, 0.99)};
}

}  // namespace

void ArmorPublisherMetrics::OnSubmitted() noexcept {
  submitted_frames_.fetch_add(1, std::memory_order_relaxed);
}

void ArmorPublisherMetrics::OnEnqueued(bool overwritten) noexcept {
  enqueued_frames_.fetch_add(1, std::memory_order_relaxed);
  if (overwritten) {
    queue_overwritten_frames_.fetch_add(1, std::memory_order_relaxed);
  }
}

void ArmorPublisherMetrics::OnRateLimited() noexcept {
  rate_limited_frames_.fetch_add(1, std::memory_order_relaxed);
}

std::uint64_t ArmorPublisherMetrics::OnEncodingError() noexcept {
  return encoding_errors_.fetch_add(1, std::memory_order_relaxed) + 1;
}

void ArmorPublisherMetrics::OnEncoded() noexcept {
  encoded_frames_.fetch_add(1, std::memory_order_relaxed);
}

void ArmorPublisherMetrics::OnLivePublished() noexcept {
  live_published_frames_.fetch_add(1, std::memory_order_relaxed);
}

void ArmorPublisherMetrics::OnRecorded() noexcept {
  recorded_frames_.fetch_add(1, std::memory_order_relaxed);
}

void ArmorPublisherMetrics::AddLatency(std::optional<double> jpeg_ms,
                                       double publish_latency_ms) noexcept {
  try {
    std::lock_guard lock(latency_mutex_);
    if (jpeg_ms.has_value()) {
      jpeg_samples_.push_back(*jpeg_ms);
    }
    publish_latency_samples_.push_back(publish_latency_ms);
  } catch (...) {
  }
}

PipelineCounts ArmorPublisherMetrics::Counts() const noexcept {
  return {
      .rate_limited_frames = rate_limited_frames_.load(std::memory_order_relaxed),
      .queue_overwritten_frames = queue_overwritten_frames_.load(std::memory_order_relaxed),
  };
}

PublisherStats ArmorPublisherMetrics::Snapshot(
    const runtime::SessionSnapshot& session,
    const runtime::SubscriptionSnapshot& image_subscription,
    const runtime::SubscriptionSnapshot& annotation_subscription,
    const runtime::SubscriptionSnapshot& stats_subscription) const noexcept {
  PublisherStats result;
  result.submitted_frames = submitted_frames_.load(std::memory_order_relaxed);
  result.enqueued_frames = enqueued_frames_.load(std::memory_order_relaxed);
  result.rate_limited_frames = rate_limited_frames_.load(std::memory_order_relaxed);
  result.queue_overwritten_frames = queue_overwritten_frames_.load(std::memory_order_relaxed);
  result.encoded_frames = encoded_frames_.load(std::memory_order_relaxed);
  result.live_published_frames = live_published_frames_.load(std::memory_order_relaxed);
  result.recorded_frames = recorded_frames_.load(std::memory_order_relaxed);
  result.encoding_errors = encoding_errors_.load(std::memory_order_relaxed);
  result.live_errors = session.live_errors;
  result.recording_errors = session.recording_errors;
  result.client_connects = session.client_connects;
  result.client_disconnects = session.client_disconnects;
  result.current_clients = session.current_clients;
  result.image_subscribers = image_subscription.subscribers;
  result.annotation_subscribers = annotation_subscription.subscribers;
  result.stats_subscribers = stats_subscription.subscribers;
  result.image_ever_subscribed = image_subscription.ever_subscribed;
  result.annotation_ever_subscribed = annotation_subscription.ever_subscribed;
  result.stats_ever_subscribed = stats_subscription.ever_subscribed;
  result.live_active = session.live_active;
  result.recording_active = session.recording_active;
  result.recording_closed_cleanly = session.recording_closed_cleanly;
  try {
    std::lock_guard lock(latency_mutex_);
    result.jpeg_encode = Summarize(jpeg_samples_);
    result.publish_latency = Summarize(publish_latency_samples_);
  } catch (...) {
  }
  return result;
}

}  // namespace mv::tool::foxglove::armor_detector
