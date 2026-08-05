#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "tool/foxglove/armor_debug_publisher.hpp"
#include "tool/foxglove/armor_detector/armor_channel_set.hpp"
#include "tool/foxglove/armor_detector/armor_message_encoder.hpp"
#include "tool/foxglove/armor_detector/armor_publisher_metrics.hpp"
#include "tool/foxglove/armor_detector/latest_frame_queue.hpp"
#include "tool/foxglove/foxglove_config.hpp"
#include "tool/foxglove/runtime/foxglove_session.hpp"

#include <atomic>
#include <memory>
#include <thread>

#include <span>

namespace mv::tool::foxglove::armor_detector {

/**
 * @brief 装甲检测调试数据的异步处理与频道发布组件。
 *
 * 组件在共享 FoxgloveSession 的实时和录制 Context 中创建等价频道集合，自身拥有
 * 最新帧队列、编码器和单个后台线程。共享会话必须比组件存活更久，并由外层门面
 * 在所有业务组件完成频道注册后启动。
 */
class ArmorDebugComponent final {
 public:
  /**
   * @brief 创建频道、队列和编码器，但暂不启动工作线程。
   * @param config 已完成校验的模块配置。
   * @param session 所有业务组件共享的传输会话。
   */
  ArmorDebugComponent(const Config& config, runtime::FoxgloveSession& session);

  /** @brief 幂等停止工作线程并关闭频道。 */
  ~ArmorDebugComponent();

  ArmorDebugComponent(const ArmorDebugComponent&) = delete;
  ArmorDebugComponent& operator=(const ArmorDebugComponent&) = delete;

  /**
   * @brief 在共享会话启动后创建后台发布线程。
   *
   * 两个 sink 都不可用时保持停止状态；线程创建失败只记录诊断错误。
   */
  void Start() noexcept;

  /**
   * @brief 非阻塞提交一帧原图、检测结果和性能指标。
   * @param frame 当前相机帧；入队后调用方不得并发改写其底层像素。
   * @param detections 当前帧装甲检测结果。
   * @param detector_stats 当前帧检测阶段耗时和候选数量。
   */
  void Publish(const hal::CameraFrame& frame, std::span<const modules::ArmorDetection> detections,
               const modules::DetectorStats& detector_stats) noexcept;

  /** @brief 获取装甲流水线、共享会话和订阅状态的组合快照。 */
  [[nodiscard]] PublisherStats SnapshotStats() const noexcept;
  /** @brief 查询组件是否接受帧且至少有一个 sink 可用。 */
  [[nodiscard]] bool IsRunning() const noexcept;
  /** @brief 停止入队、排空最后一帧、等待工作线程并关闭频道。 */
  void Stop() noexcept;

 private:
  /** @brief 将三个实时频道的订阅数转换为当前消息需求。 */
  [[nodiscard]] TopicDemand LiveDemand() const noexcept;
  /** @brief 持续消费最新帧，直到 Stop() 后队列完全排空。 */
  void WorkerLoop() noexcept;
  /** @brief 合并 sink 需求、编码一次并分别发布到两个频道集合。 */
  void ProcessFrame(const FrameItem& item);
  /** @brief 将逐话题 SDK 错误上报给对应共享 sink。 */
  void ReportPublishErrors(const ChannelPublishResult& result, bool recording) noexcept;

  runtime::FoxgloveSession& session_;               ///< 生命周期由根层门面管理。
  LatestFrameQueue queue_;                          ///< 容量为 1 的非阻塞最新帧队列。
  ArmorMessageEncoder encoder_;                     ///< 实时与录制共享的消息编码器。
  ArmorPublisherMetrics metrics_;                   ///< 装甲发布流水线累计统计。
  std::unique_ptr<ArmorChannelSet> live_channels_;  ///< 实时 Context 频道集合。
  std::unique_ptr<ArmorChannelSet> recording_channels_;  ///< 录制 Context 频道集合。
  ChannelIds live_channel_ids_;                          ///< 用于查询实时订阅需求。
  std::thread worker_;                                   ///< 唯一 JPEG 与频道发布线程。
  std::atomic<bool> accepting_{false};                   ///< Publish() 是否允许继续入队。
  std::atomic<bool> stop_called_{false};                 ///< 保证 Stop() 只执行一次。
};

}  // namespace mv::tool::foxglove::armor_detector
