#pragma once

#include "tool/foxglove/pipeline/vision_message_encoder.hpp"
#include "tool/foxglove/runtime/foxglove_session.hpp"
#include "tool/foxglove/vision_debug_publisher.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

#include <optional>

namespace mv::tool::foxglove::pipeline {

/** @brief 汇总视觉调试流水线计数与延迟样本的线程安全统计器。 */
class VisionPublisherMetrics final {
 public:
  /** @brief 记录一次 Publish() 调用。 */
  void OnSubmitted() noexcept;
  /** @brief 记录成功入队，并按参数累计覆盖帧。 */
  void OnEnqueued(bool overwritten) noexcept;
  /** @brief 记录一帧被 max_fps 限流。 */
  void OnRateLimited() noexcept;
  /** @brief 累计一次入队或编码错误并返回最新错误序号。 */
  [[nodiscard]] std::uint64_t OnEncodingError() noexcept;
  /** @brief 记录一帧完成 JPEG 编码。 */
  void OnEncoded() noexcept;
  /** @brief 记录一帧全部适用实时话题发布成功。 */
  void OnLivePublished() noexcept;
  /** @brief 记录一帧全部适用录制话题写入成功。 */
  void OnRecorded() noexcept;
  /** @brief 保存可选 JPEG 耗时和端到端发布延迟样本。 */
  void AddLatency(std::optional<double> jpeg_ms, double publish_latency_ms) noexcept;

  /** @brief 返回编码 debug stats 所需的限流与覆盖累计值。 */
  [[nodiscard]] PipelineCounts Counts() const noexcept;
  /**
   * @brief 合并流水线、会话及关键实时频道订阅状态。
   *
   * 延迟分位数分配失败时仍返回其他累计字段，不向发布主链路传播异常。
   */
  [[nodiscard]] VisionPublisherStats Snapshot(
      const runtime::SessionSnapshot& session,
      const runtime::SubscriptionSnapshot& image_subscription,
      const runtime::SubscriptionSnapshot& armor_annotation_subscription,
      const runtime::SubscriptionSnapshot& armor_stats_subscription,
      const runtime::SubscriptionSnapshot& debug_stats_subscription) const noexcept;

 private:
  std::atomic<std::uint64_t> submitted_frames_{0};          ///< Publish() 调用数。
  std::atomic<std::uint64_t> enqueued_frames_{0};           ///< 成功入队帧数。
  std::atomic<std::uint64_t> rate_limited_frames_{0};       ///< 限流跳过帧数。
  std::atomic<std::uint64_t> queue_overwritten_frames_{0};  ///< 队列覆盖帧数。
  std::atomic<std::uint64_t> encoded_frames_{0};            ///< JPEG 编码帧数。
  std::atomic<std::uint64_t> live_published_frames_{0};     ///< 实时完整发布帧数。
  std::atomic<std::uint64_t> recorded_frames_{0};           ///< 录制完整写入帧数。
  std::atomic<std::uint64_t> encoding_errors_{0};           ///< 入队与编码错误数。
  mutable std::mutex latency_mutex_;                        ///< 保护两个延迟样本数组。
  std::vector<double> jpeg_samples_;                        ///< JPEG 毫秒耗时样本。
  std::vector<double> publish_latency_samples_;             ///< 发布延迟毫秒样本。
};

}  // namespace mv::tool::foxglove::pipeline
