#pragma once

#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace mv::tool::foxglove::runtime {

/**
 * @brief 单个实时频道的线程安全订阅状态快照。
 */
struct SubscriptionSnapshot {
  std::uint64_t subscribers{0};  ///< 当前订阅该频道的客户端数量。
  bool ever_subscribed{false};   ///< 会话启动后该频道是否至少被订阅过一次。
};

/**
 * @brief 维护 WebSocket 回调线程与发布线程之间共享的频道订阅状态。
 *
 * 频道必须在 WebSocket Server 启动前注册。未知频道的订阅事件会被忽略，避免客户端
 * 回调意外扩张注册表。
 */
class SubscriptionRegistry final {
 public:
  /**
   * @brief 注册一个允许跟踪的 Foxglove 频道 ID。
   * @param channel_id Foxglove Context 内唯一的频道 ID。
   */
  void Register(std::uint64_t channel_id);

  /** @brief 记录一次订阅事件；未知频道不会产生状态。 */
  void Subscribe(std::uint64_t channel_id) noexcept;

  /** @brief 记录一次取消订阅事件，计数最低保持为 0。 */
  void Unsubscribe(std::uint64_t channel_id) noexcept;

  /** @brief 获取指定频道的订阅状态，未知频道返回全零快照。 */
  [[nodiscard]] SubscriptionSnapshot Snapshot(std::uint64_t channel_id) const noexcept;

 private:
  mutable std::mutex mutex_;  ///< 保护注册表及每个频道的复合状态。
  std::unordered_map<std::uint64_t, SubscriptionSnapshot> subscriptions_;  ///< ID 到状态。
};

}  // namespace mv::tool::foxglove::runtime
