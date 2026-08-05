#include "tool/foxglove/runtime/subscription_registry.hpp"

#include <utility>

namespace mv::tool::foxglove::runtime {

void SubscriptionRegistry::Register(std::uint64_t channel_id) {
  std::lock_guard lock(mutex_);
  subscriptions_.try_emplace(channel_id);
}

void SubscriptionRegistry::Subscribe(std::uint64_t channel_id) noexcept {
  try {
    std::lock_guard lock(mutex_);
    const auto iterator = subscriptions_.find(channel_id);
    if (iterator == subscriptions_.end()) {
      return;
    }
    ++iterator->second.subscribers;
    iterator->second.ever_subscribed = true;
  } catch (...) {
  }
}

void SubscriptionRegistry::Unsubscribe(std::uint64_t channel_id) noexcept {
  try {
    std::lock_guard lock(mutex_);
    const auto iterator = subscriptions_.find(channel_id);
    if (iterator != subscriptions_.end() && iterator->second.subscribers > 0) {
      --iterator->second.subscribers;
    }
  } catch (...) {
  }
}

SubscriptionSnapshot SubscriptionRegistry::Snapshot(std::uint64_t channel_id) const noexcept {
  try {
    std::lock_guard lock(mutex_);
    const auto iterator = subscriptions_.find(channel_id);
    if (iterator != subscriptions_.end()) {
      return iterator->second;
    }
  } catch (...) {
  }
  return {};
}

}  // namespace mv::tool::foxglove::runtime
