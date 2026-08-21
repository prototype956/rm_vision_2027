#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_light_detector/armor_light_detector.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"
#include "modules/fire_control/fire_control.hpp"
#include "tool/foxglove/foxglove_config.hpp"

#include <cstdint>
#include <memory>

#include <filesystem>
#include <span>

namespace mv::tool::foxglove {

/** @brief 一组毫秒耗时样本的常用分位数。 */
struct LatencyPercentiles {
  double p50_ms{0.0};  ///< 中位数。
  double p95_ms{0.0};  ///< 95% 样本不超过的耗时。
  double p99_ms{0.0};  ///< 99% 样本不超过的耗时。
};

/** @brief Foxglove 视觉调试发布器的线程安全累计快照。 */
struct VisionPublisherStats {
  std::uint64_t submitted_frames{0};          ///< Publish() 接收的总帧数。
  std::uint64_t enqueued_frames{0};           ///< 通过限流并写入队列的帧数。
  std::uint64_t rate_limited_frames{0};       ///< 因 max_fps 跳过的帧数。
  std::uint64_t queue_overwritten_frames{0};  ///< 尚未消费便被新帧覆盖的帧数。
  std::uint64_t dropped_control_samples{0};   ///< 控制队列满时丢弃的最旧样本数。
  std::uint64_t encoded_frames{0};            ///< 完成 JPEG 编码的帧数。
  std::uint64_t live_published_frames{0};     ///< 全部适用实时话题发布成功的帧数。
  std::uint64_t recorded_frames{0};           ///< 全部适用 MCAP 话题写入成功的帧数。
  std::uint64_t encoding_errors{0};           ///< 入队、线程或编码阶段错误数。
  std::uint64_t live_errors{0};               ///< WebSocket 初始化、发布及关闭错误数。
  std::uint64_t recording_errors{0};          ///< MCAP 初始化、写入及关闭错误数。
  std::uint64_t client_connects{0};           ///< 累计客户端连接次数。
  std::uint64_t client_disconnects{0};        ///< 累计客户端断开次数。
  std::uint64_t current_clients{0};           ///< 当前 WebSocket 客户端数量。
  std::uint64_t image_subscribers{0};         ///< 当前压缩图像订阅数。
  std::uint64_t armor_annotation_subscribers{0};  ///< 当前装甲标注订阅数。
  std::uint64_t armor_stats_subscribers{0};       ///< 当前检测器指标订阅数。
  std::uint64_t debug_stats_subscribers{0};       ///< 当前流水线指标订阅数。
  bool image_ever_subscribed{false};              ///< 图像是否曾被订阅。
  bool armor_annotation_ever_subscribed{false};   ///< 装甲标注是否曾被订阅。
  bool armor_stats_ever_subscribed{false};        ///< 检测器指标是否曾被订阅。
  bool debug_stats_ever_subscribed{false};        ///< 流水线指标是否曾被订阅。
  bool live_active{false};                        ///< WebSocket sink 当前是否可用。
  bool recording_active{false};                   ///< MCAP sink 当前是否可用。
  bool recording_closed_cleanly{false};           ///< MCAP 是否成功写入尾部并关闭。
  LatencyPercentiles jpeg_encode;                 ///< JPEG 编码耗时分位数。
  LatencyPercentiles publish_latency;  ///< HAL 收帧到消息构造完成的延迟分位数。
};

/**
 * @brief 异步发布同帧图像、检测、空间与仿真调试数据。
 *
 * 该类是 Foxglove 模块的稳定外观接口。Publish() 不执行 JPEG、网络或磁盘操作，
 * 仅将最新调试帧交给内部流水线，避免反压视觉检测主链路。
 */
class VisionDebugPublisher final {
 public:
  /**
   * @brief 创建会话和频道并启动后台流水线。
   *
   * 单个 sink 初始化失败只会停用该 sink 并记录诊断，不向视觉主程序传播异常。
   */
  explicit VisionDebugPublisher(const Config& config);
  /** @brief 幂等停止后台流水线并关闭实时与录制资源。 */
  ~VisionDebugPublisher();

  VisionDebugPublisher(const VisionDebugPublisher&) = delete;
  VisionDebugPublisher& operator=(const VisionDebugPublisher&) = delete;
  VisionDebugPublisher(VisionDebugPublisher&&) = delete;
  VisionDebugPublisher& operator=(VisionDebugPublisher&&) = delete;

  /**
   * @brief 非阻塞提交一帧完整视觉调试数据。
   * @param frame 原始相机帧；入队后调用方不得并发改写其像素。
   * @param detections 与该图像对应的检测结果，调用期间复制。
   * @param detector_stats 与该图像对应的检测性能统计。
   * @param lightbar_result 与该图像对应的独立灯条和检测统计。
   * @param pnp_result 与该图像对应的 PnP 解算、基准及角点精修结果。
   */
  void Publish(const hal::CameraFrame& frame, std::span<const modules::ArmorDetection> detections,
               const modules::DetectorStats& detector_stats,
               const modules::LightbarDetectionResult& lightbar_result,
               const modules::ArmorPnpFrameResult& pnp_result,
               const modules::ArmorPredictionResult& prediction_result) noexcept;
  /** @brief 非阻塞提交一个 100 Hz 控制诊断样本。 */
  void PublishControl(const modules::FireControlResult& result) noexcept;
  /** @brief 获取自启动以来的线程安全累计统计。 */
  [[nodiscard]] VisionPublisherStats SnapshotStats() const noexcept;
  /** @brief 查询至少一个 sink 可用且流水线仍接受帧。 */
  [[nodiscard]] bool IsRunning() const noexcept;
  /** @brief 返回本次 MCAP 路径；未启用或初始化失败时为空。 */
  [[nodiscard]] std::filesystem::path RecordingPath() const;
  /** @brief 幂等停止并保证已入队的最后一帧得到处理。 */
  void Stop() noexcept;

 private:
  struct Impl;                  ///< 隔离会话与流水线的实现依赖。
  std::unique_ptr<Impl> impl_;  ///< 唯一拥有的实现对象。
};

}  // namespace mv::tool::foxglove
