#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "tool/foxglove/foxglove_config.hpp"

#include <cstdint>
#include <memory>

#include <filesystem>
#include <span>

namespace mv::tool::foxglove {

/**
 * @brief 一组耗时样本的常用分位数，单位统一为毫秒。
 */
struct LatencyPercentiles {
  double p50_ms{0.0};  ///< 中位数。
  double p95_ms{0.0};  ///< 95% 样本不超过的耗时。
  double p99_ms{0.0};  ///< 99% 样本不超过的耗时。
};

/**
 * @brief Foxglove 发布器自启动以来的线程安全快照。
 */
struct PublisherStats {
  std::uint64_t submitted_frames{0};          ///< Publish() 接收的总帧数。
  std::uint64_t enqueued_frames{0};           ///< 通过限流并写入最新帧队列的帧数。
  std::uint64_t rate_limited_frames{0};       ///< 因 max_fps 限制跳过的帧数。
  std::uint64_t queue_overwritten_frames{0};  ///< 尚未消费便被更新帧覆盖的帧数。
  std::uint64_t encoded_frames{0};            ///< 完成 JPEG 编码的帧数。
  std::uint64_t live_published_frames{0};  ///< 所有已订阅实时话题均发布成功的帧数。
  std::uint64_t recorded_frames{0};        ///< 三个 MCAP 话题均写入成功的帧数。
  std::uint64_t encoding_errors{0};        ///< 入队、工作线程或编码阶段的错误数。
  std::uint64_t live_errors{0};            ///< WebSocket 初始化、发布及关闭错误数。
  std::uint64_t recording_errors{0};       ///< MCAP 初始化、写入及关闭错误数。
  std::uint64_t client_connects{0};        ///< 累计客户端连接次数。
  std::uint64_t client_disconnects{0};     ///< 累计客户端断开次数。
  std::uint64_t current_clients{0};        ///< 当前连接的客户端数量。
  std::uint64_t image_subscribers{0};      ///< 当前压缩图像话题订阅数。
  std::uint64_t annotation_subscribers{0};  ///< 当前装甲标注话题订阅数。
  std::uint64_t stats_subscribers{0};       ///< 当前性能指标话题订阅数。
  bool image_ever_subscribed{false};        ///< 压缩图像话题是否曾被订阅。
  bool annotation_ever_subscribed{false};   ///< 装甲标注话题是否曾被订阅。
  bool stats_ever_subscribed{false};        ///< 性能指标话题是否曾被订阅。
  bool live_active{false};                  ///< WebSocket sink 当前是否可用。
  bool recording_active{false};             ///< MCAP sink 当前是否可用。
  bool recording_closed_cleanly{false};     ///< MCAP Writer 是否成功写入尾部并关闭。
  LatencyPercentiles jpeg_encode;           ///< JPEG 编码耗时分位数。
  LatencyPercentiles publish_latency;  ///< 相机时间戳到消息构造完成的延迟分位数。
};

/**
 * @brief 异步发布原始图像、装甲标注和检测指标到 Foxglove。
 *
 * Publish() 只保留最新调试帧，不会等待 JPEG、网络或磁盘。
 */
class ArmorDebugPublisher final {
 public:
  /**
   * @brief 按配置创建共享 Foxglove 会话和装甲调试组件。
   *
   * WebSocket 或 MCAP 初始化失败会记录错误并停用对应 sink，不向主程序传播异常。
   *
   * @param config 已完成校验的 Foxglove 配置。
   */
  explicit ArmorDebugPublisher(Config config);

  /**
   * @brief 停止后台线程并关闭实时及录制资源。
   */
  ~ArmorDebugPublisher();

  ArmorDebugPublisher(const ArmorDebugPublisher&) = delete;
  ArmorDebugPublisher& operator=(const ArmorDebugPublisher&) = delete;
  ArmorDebugPublisher(ArmorDebugPublisher&&) = delete;
  ArmorDebugPublisher& operator=(ArmorDebugPublisher&&) = delete;

  /**
   * @brief 非阻塞提交一帧装甲检测调试数据。
   *
   * 函数只执行订阅检查、限流和最新帧入队；JPEG、网络和磁盘操作均在后台线程完成。
   *
   * @param frame 原始相机帧；入队后调用方不得并发改写其底层像素。
   * @param detections 与该帧对应的装甲检测结果，调用期间复制到队列。
   * @param detector_stats 与该帧对应的检测阶段统计。
   */
  void Publish(const hal::CameraFrame& frame, std::span<const modules::ArmorDetection> detections,
               const modules::DetectorStats& detector_stats) noexcept;

  /**
   * @brief 获取发布器自启动以来的线程安全累计统计。
   */
  [[nodiscard]] PublisherStats SnapshotStats() const noexcept;

  /**
   * @brief 查询是否至少有一个 sink 可用且后台组件仍接受帧。
   */
  [[nodiscard]] bool IsRunning() const noexcept;

  /**
   * @brief 获取本次会话选定的 MCAP 路径，未启用录制时为空。
   */
  [[nodiscard]] std::filesystem::path RecordingPath() const;

  /**
   * @brief 幂等停止发布器，并保证已入队的最后一帧得到处理。
   */
  void Stop() noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::tool::foxglove
