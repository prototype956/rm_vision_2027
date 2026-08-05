#pragma once

#include "tool/foxglove/armor_debug_publisher.hpp"
#include "tool/foxglove/runtime/foxglove_session.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

#include <optional>

namespace mv::tool::foxglove::armor_detector {

/**
 * @brief 写入每帧 stats JSON 的流水线累计计数子集。
 */
struct PipelineCounts {
  std::uint64_t rate_limited_frames{0};       ///< 累计限流帧数。
  std::uint64_t queue_overwritten_frames{0};  ///< 累计队列覆盖帧数。
};

/**
 * @brief 聚合装甲发布流水线的原子计数和延迟样本。
 *
 * 生产者、后台发布线程和测试报告线程可以并发更新或读取。共享会话的连接、订阅与
 * sink 错误由 FoxgloveSession 维护，在 Snapshot() 时合并为公开 PublisherStats。
 */
class ArmorPublisherMetrics final {
 public:
  /** @brief 记录一次 Publish() 调用。 */
  void OnSubmitted() noexcept;
  /** @brief 记录一次成功入队以及是否覆盖旧帧。 */
  void OnEnqueued(bool overwritten) noexcept;
  /** @brief 记录一次 max_fps 限流。 */
  void OnRateLimited() noexcept;
  /** @brief 记录一次入队或编码错误，并返回更新后的错误总数。 */
  [[nodiscard]] std::uint64_t OnEncodingError() noexcept;
  /** @brief 记录一帧 JPEG 编码完成。 */
  void OnEncoded() noexcept;
  /** @brief 记录一帧所有已订阅实时话题发布成功。 */
  void OnLivePublished() noexcept;
  /** @brief 记录一帧三个 MCAP 话题写入成功。 */
  void OnRecorded() noexcept;
  /** @brief 保存一帧的可选编码耗时和端到端发布延迟样本。 */
  void AddLatency(std::optional<double> jpeg_ms, double publish_latency_ms) noexcept;

  /** @brief 获取消息编码器需要写入 stats JSON 的轻量计数快照。 */
  [[nodiscard]] PipelineCounts Counts() const noexcept;

  /**
   * @brief 合并流水线、共享会话和三个实时频道的完整公开统计。
   *
   * 延迟分位数计算失败时仍返回其他累计字段，诊断接口不会向主链路传播异常。
   */
  [[nodiscard]] PublisherStats Snapshot(
      const runtime::SessionSnapshot& session,
      const runtime::SubscriptionSnapshot& image_subscription,
      const runtime::SubscriptionSnapshot& annotation_subscription,
      const runtime::SubscriptionSnapshot& stats_subscription) const noexcept;

 private:
  // 高频计数均使用 relaxed 原子操作；统计只要求单字段原子快照，不要求跨字段事务一致。
  std::atomic<std::uint64_t> submitted_frames_{0};
  std::atomic<std::uint64_t> enqueued_frames_{0};
  std::atomic<std::uint64_t> rate_limited_frames_{0};
  std::atomic<std::uint64_t> queue_overwritten_frames_{0};
  std::atomic<std::uint64_t> encoded_frames_{0};
  std::atomic<std::uint64_t> live_published_frames_{0};
  std::atomic<std::uint64_t> recorded_frames_{0};
  std::atomic<std::uint64_t> encoding_errors_{0};

  mutable std::mutex latency_mutex_;             ///< 保护两个可增长样本序列。
  std::vector<double> jpeg_samples_;             ///< 所有已编码帧的 JPEG 耗时。
  std::vector<double> publish_latency_samples_;  ///< 所有已处理帧的发布延迟。
};

}  // namespace mv::tool::foxglove::armor_detector
