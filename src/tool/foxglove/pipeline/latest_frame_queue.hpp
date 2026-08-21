#pragma once

#include "tool/foxglove/pipeline/vision_debug_frame.hpp"

#include <condition_variable>
#include <mutex>

#include <optional>
#include <span>

namespace mv::tool::foxglove::pipeline {

/** @brief 一次 Push() 的限流和容量覆盖结果。 */
struct QueuePushResult {
  bool enqueued{false};      ///< 当前帧是否写入队列。
  bool rate_limited{false};  ///< 当前帧是否因 max_fps 被跳过。
  bool overwritten{false};   ///< 入队时是否覆盖了一帧尚未消费的数据。
};

/**
 * @brief 容量为 1 的非阻塞最新帧队列，限流作用于整个调试批次。
 *
 * 生产者不等待后台编码；消费者落后时，新帧覆盖旧帧。Stop() 后拒绝新帧，但
 * WaitPop() 仍会交付停止前保留的最后一帧，然后返回空值。
 */
class LatestFrameQueue final {
 public:
  /** @brief 创建指定最大发布频率的队列；调用方保证 max_fps 为有效正数。 */
  explicit LatestFrameQueue(double max_fps);

  /**
   * @brief 尝试限流并用当前同帧数据更新队列。
   * @throws std::bad_alloc 复制检测结果或空间元数据失败。
   */
  [[nodiscard]] QueuePushResult Push(const hal::CameraFrame& frame,
                                     std::span<const modules::ArmorDetection> detections,
                                     const modules::DetectorStats& detector_stats,
                                     const modules::LightbarDetectionResult& lightbar_result,
                                     const modules::ArmorPnpFrameResult& pnp_result,
                                     const modules::ArmorPredictionResult& prediction_result,
                                     std::optional<modules::ArmorSelectionSnapshot> selection);
  /** @brief 阻塞等待下一帧；停止且队列排空后返回空值。 */
  [[nodiscard]] std::optional<VisionDebugFrame> WaitPop() noexcept;
  /** @brief 幂等停止生产者并唤醒等待中的唯一消费者。 */
  void Stop() noexcept;

 private:
  const SteadyClock::duration PERIOD;                 ///< max_fps 对应的帧间隔。
  const SteadyClock::duration JITTER_TOLERANCE;       ///< 调度边界的小抖动容差。
  std::mutex mutex_;                                  ///< 保护调度、队列与停止状态。
  std::condition_variable condition_;                 ///< 唤醒唯一后台消费者。
  std::optional<VisionDebugFrame> queued_frame_;      ///< 当前唯一待处理帧。
  SteadyClock::time_point next_publish_timestamp_{};  ///< 下一个允许入队的时刻。
  bool stopped_{false};                               ///< 停止后永久拒绝 Push()。
};

}  // namespace mv::tool::foxglove::pipeline
