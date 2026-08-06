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
    const auto ITERATOR = subscriptions_.find(channel_id);
    if (ITERATOR == subscriptions_.end()) {
      return;
    }
    ++ITERATOR->second.subscribers;
    ITERATOR->second.ever_subscribed = true;
  } catch (...) {
  }
}

void SubscriptionRegistry::Unsubscribe(std::uint64_t channel_id) noexcept {
  try {
    std::lock_guard lock(mutex_);
    const auto ITERATOR = subscriptions_.find(channel_id);
    if (ITERATOR != subscriptions_.end() && ITERATOR->second.subscribers > 0) {
      --ITERATOR->second.subscribers;
    }
  } catch (...) {
  }
}

SubscriptionSnapshot SubscriptionRegistry::Snapshot(std::uint64_t channel_id) const noexcept {
  try {
    std::lock_guard lock(mutex_);
    const auto ITERATOR = subscriptions_.find(channel_id);
    if (ITERATOR != subscriptions_.end()) {
      return ITERATOR->second;
    }
  } catch (...) {
  }
  return {};
}

}  // namespace mv::tool::foxglove::runtime
