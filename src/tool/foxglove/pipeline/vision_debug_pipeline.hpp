#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "tool/foxglove/foxglove_config.hpp"
#include "tool/foxglove/pipeline/latest_frame_queue.hpp"
#include "tool/foxglove/pipeline/vision_channel_set.hpp"
#include "tool/foxglove/pipeline/vision_message_encoder.hpp"
#include "tool/foxglove/pipeline/vision_publisher_metrics.hpp"
#include "tool/foxglove/runtime/foxglove_session.hpp"
#include "tool/foxglove/vision_debug_publisher.hpp"

#include <atomic>
#include <memory>
#include <thread>

#include <span>

namespace mv::tool::foxglove::pipeline {

/**
 * @brief 全视觉调试帧的单线程异步编码与发布流水线。
 *
 * 前台 Publish() 只完成需求检查、限流和容量 1 入队；JPEG、领域消息编码、WebSocket
 * 与 MCAP 写入都由唯一后台线程串行执行。FoxgloveSession 必须比本对象存活更久。
 */
class VisionDebugPipeline final {
 public:
  /** @brief 创建实时/录制频道和队列，但不启动后台线程。 */
  VisionDebugPipeline(const Config& config, runtime::FoxgloveSession& session);
  /** @brief 幂等停止、排空最后一帧并关闭频道。 */
  ~VisionDebugPipeline();

  VisionDebugPipeline(const VisionDebugPipeline&) = delete;
  VisionDebugPipeline& operator=(const VisionDebugPipeline&) = delete;

  /** @brief 会话启动后创建后台线程；无可用 sink 时保持停止。 */
  void Start() noexcept;
  /** @brief 非阻塞提交同帧图像、检测结果和指标。 */
  void Publish(const hal::CameraFrame& frame, std::span<const modules::ArmorDetection> detections,
               const modules::DetectorStats& detector_stats,
               const modules::ArmorPnpFrameResult& pnp_result) noexcept;
  /** @brief 获取流水线、会话和关键实时订阅的线程安全组合快照。 */
  [[nodiscard]] VisionPublisherStats SnapshotStats() const noexcept;
  /** @brief 查询流水线是否仍接受帧且至少有一个 sink 可用。 */
  [[nodiscard]] bool IsRunning() const noexcept;
  /** @brief 停止入队、排空最后一帧、等待线程并关闭两组频道。 */
  void Stop() noexcept;

 private:
  /** @brief 将全部实时频道订阅数转换为当前按需编码集合。 */
  [[nodiscard]] TopicDemand LiveDemand() const noexcept;
  /** @brief 持续消费最新帧，直至 Stop() 后队列排空。 */
  void WorkerLoop() noexcept;
  /** @brief 合并实时与录制需求，编码一次并分发到两个 sink。 */
  void ProcessFrame(const VisionDebugFrame& frame);
  /** @brief 将逐话题 SDK 错误上报给对应会话 sink。 */
  void ReportPublishErrors(const ChannelPublishResult& result, bool recording) noexcept;

  runtime::FoxgloveSession& session_;  ///< 非拥有引用，由外层发布器管理生命周期。
  LatestFrameQueue queue_;             ///< 容量为 1 的非阻塞最新帧队列。
  VisionMessageEncoder encoder_;       ///< 实时与录制共享的按需编码器。
  VisionPublisherMetrics metrics_;     ///< 线程安全累计统计。
  std::unique_ptr<VisionChannelSet> live_channels_;       ///< WebSocket Context 频道集合。
  std::unique_ptr<VisionChannelSet> recording_channels_;  ///< MCAP Context 频道集合。
  ChannelIds live_channel_ids_;           ///< 查询实时订阅需求所需的频道 ID。
  std::thread worker_;                    ///< 唯一编码和频道发布线程。
  std::atomic<bool> accepting_{false};    ///< Publish() 是否允许继续入队。
  std::atomic<bool> stop_called_{false};  ///< 保证 Stop() 只执行一次。
};

}  // namespace mv::tool::foxglove::pipeline
