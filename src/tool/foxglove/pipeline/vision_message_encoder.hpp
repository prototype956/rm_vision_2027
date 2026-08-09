#pragma once

#include "tool/foxglove/foxglove_config.hpp"
#include "tool/foxglove/pipeline/vision_debug_frame.hpp"

#include <chrono>
#include <cstdint>
#include <string>

#include <foxglove/schemas.hpp>
#include <optional>

namespace mv::tool::foxglove::pipeline {

/** @brief 某个 sink 在当前帧需要的视觉话题集合。 */
struct TopicDemand {
  bool image{false};                   ///< 是否需要 JPEG 原图。
  bool armor_annotations{false};       ///< 是否需要二维装甲标注。
  bool armor_stats{false};             ///< 是否需要装甲检测器指标。
  bool debug_stats{false};             ///< 是否需要发布流水线指标。
  bool transforms{false};              ///< 是否需要同帧 TF。
  bool calibration{false};             ///< 是否需要相机标定。
  bool frustum{false};                 ///< 是否需要三维视锥。
  bool ground_truth{false};            ///< 是否需要仿真三维真值。
  bool projection_annotations{false};  ///< 是否需要真值二维重投影点。

  /** @brief 查询是否至少需求一个话题。 */
  [[nodiscard]] bool Any() const noexcept;
};

/** @brief 合并实时订阅和录制需求，使每种消息每帧最多编码一次。 */
[[nodiscard]] TopicDemand Merge(TopicDemand left, TopicDemand right) noexcept;

/** @brief 写入调试统计消息的流水线累计计数。 */
struct PipelineCounts {
  std::uint64_t rate_limited_frames{0};       ///< 因 max_fps 跳过的累计帧数。
  std::uint64_t queue_overwritten_frames{0};  ///< 容量 1 队列覆盖的累计帧数。
};

/**
 * @brief 可同时发布到实时与录制 sink 的同帧预编码消息。
 *
 * 所有存在的消息使用同一个采集时间戳；未被需求或缺少 geometry 的领域保持为空。
 */
struct PreparedFrame {
  std::uint64_t epoch_nanos{0};  ///< SDK log() 使用的 Unix epoch 纳秒时间戳。
  std::optional<::foxglove::schemas::CompressedImage> image;  ///< 按需生成的 JPEG 消息。
  std::optional<::foxglove::schemas::ImageAnnotations> armor_annotations;  ///< 装甲标注。
  std::optional<std::string> armor_stats_json;  ///< 符合固定 Schema 的检测器指标。
  std::optional<std::string> debug_stats_json;  ///< 符合固定 Schema 的流水线指标。
  std::optional<::foxglove::schemas::FrameTransforms> transforms;     ///< 同帧 TF 树。
  std::optional<::foxglove::schemas::CameraCalibration> calibration;  ///< 同帧标定。
  std::optional<::foxglove::schemas::SceneUpdate> frustum;            ///< 三维视锥图元。
  std::optional<::foxglove::schemas::SceneUpdate> ground_truth;       ///< 三维仿真真值。
  std::optional<::foxglove::schemas::ImageAnnotations> projection_annotations;  ///< 真值投影点。
  std::optional<double> jpeg_ms;   ///< JPEG 编码耗时，未编码图像时为空。
  double publish_latency_ms{0.0};  ///< HAL 收帧到消息构造完成的延迟，单位为毫秒。
};

/**
 * @brief 生成全领域共用时间戳并按需调度各领域编码器。
 *
 * 数据源提供 capture_timestamp_ns 时直接使用；否则通过构造时记录的 steady/system
 * 双时钟锚点换算 epoch，避免运行中系统校时破坏消息顺序。
 */
class VisionMessageEncoder final {
 public:
  /** @brief 保存不可变的图像坐标系、JPEG 质量和双时钟锚点。 */
  explicit VisionMessageEncoder(const ImageConfig& config);

  /**
   * @brief 按合并需求编码单帧消息，结果可被实时和录制频道共享。
   * @throws std::runtime_error JPEG 编码失败。
   */
  [[nodiscard]] PreparedFrame Encode(const VisionDebugFrame& frame, TopicDemand demand,
                                     PipelineCounts counts) const;

 private:
  ImageConfig config_;                     ///< frame_id、max_fps 和 JPEG 质量配置副本。
  SteadyClock::time_point steady_anchor_;  ///< 构造时的单调时钟锚点。
  std::chrono::system_clock::time_point system_anchor_;  ///< 与其对应的 epoch 锚点。
};

}  // namespace mv::tool::foxglove::pipeline
