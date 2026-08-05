#pragma once

#include "tool/foxglove/armor_detector/armor_publisher_metrics.hpp"
#include "tool/foxglove/armor_detector/latest_frame_queue.hpp"
#include "tool/foxglove/foxglove_config.hpp"

#include <chrono>
#include <cstdint>
#include <string>

#include <foxglove/schemas.hpp>
#include <optional>

namespace mv::tool::foxglove::armor_detector {

/**
 * @brief 某个 sink 当前需要发布的装甲调试话题集合。
 */
struct TopicDemand {
  bool image{false};        ///< 是否需要压缩原图。
  bool annotations{false};  ///< 是否需要二维装甲标注。
  bool stats{false};        ///< 是否需要检测与发布指标。

  /** @brief 查询集合中是否至少包含一个话题。 */
  [[nodiscard]] bool Any() const noexcept { return image || annotations || stats; }
};

/**
 * @brief 合并实时订阅和 MCAP 录制需求，确保每种消息每帧最多构造一次。
 */
[[nodiscard]] TopicDemand Merge(TopicDemand left, TopicDemand right) noexcept;

/**
 * @brief 已完成时间同步和按需编码、可同时交给两个 sink 的消息批次。
 */
struct PreparedFrame {
  std::uint64_t epoch_nanos{0};  ///< 三个话题共同使用的 Unix epoch 纳秒时间戳。
  std::optional<::foxglove::schemas::CompressedImage> image;  ///< 按需生成的 JPEG 消息。
  std::optional<::foxglove::schemas::ImageAnnotations> annotations;  ///< 按需生成的标注。
  std::optional<std::string> stats_json;  ///< 按需生成且符合固定 Schema 的 JSON 指标。
  std::optional<double> jpeg_ms;          ///< 本帧 JPEG 编码耗时，未编码时为空。
  double publish_latency_ms{0.0};         ///< 相机时间戳到消息构造完成的延迟。
};

/**
 * @brief 将装甲检测帧转换为 Foxglove 图像、标注和统计消息。
 *
 * 编码器在构造时锚定 steady_clock 与 system_clock，只用相对单调时钟差推导 epoch
 * 时间，避免系统时钟调整破坏同一运行期间的帧顺序。
 */
class ArmorMessageEncoder final {
 public:
  /**
   * @brief 创建使用指定 frame_id 和 JPEG 质量的编码器。
   */
  explicit ArmorMessageEncoder(const ImageConfig& config);

  /**
   * @brief 按需求构造单帧消息批次。
   *
   * 三个话题共用同一个 epoch 时间戳；零检测帧仍生成携带时间戳的不可见标注，
   * 用于清除 Foxglove 中上一帧的装甲框。
   *
   * @param item 最新帧队列取出的完整调试数据。
   * @param demand 实时与录制 sink 合并后的话题需求。
   * @param counts 写入 stats JSON 的累计限流和覆盖计数。
   * @return 可被实时和录制频道共享的消息批次。
   * @throws std::runtime_error JPEG 编码失败。
   */
  [[nodiscard]] PreparedFrame Encode(const FrameItem& item, TopicDemand demand,
                                     PipelineCounts counts) const;

 private:
  ImageConfig config_;                     ///< frame_id 与 JPEG 质量的不可变运行参数。
  SteadyClock::time_point steady_anchor_;  ///< 构造时的单调时钟锚点。
  std::chrono::system_clock::time_point system_anchor_;  ///< 与其对应的 epoch 锚点。
};

}  // namespace mv::tool::foxglove::armor_detector
