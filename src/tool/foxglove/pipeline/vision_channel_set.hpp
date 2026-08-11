#pragma once

#include "tool/foxglove/pipeline/vision_message_encoder.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::pipeline {

/** @brief 统一视觉流水线发布的固定话题。 */
enum class VisionTopic {
  IMAGE,                   ///< JPEG 压缩原图。
  ARMOR_ANNOTATIONS,       ///< 装甲四角框与标签。
  ARMOR_STATS,             ///< 装甲检测器性能 JSON。
  DEBUG_STATS,             ///< 调试发布流水线性能 JSON。
  TRANSFORMS,              ///< world -> gimbal -> camera_optical TF。
  CALIBRATION,             ///< 相机内参与畸变参数。
  FRUSTUM,                 ///< camera_optical 下的三维视锥。
  GROUND_TRUTH,            ///< world 下的仿真三维真值。
  PROJECTION_ANNOTATIONS,  ///< 真值探针在相机图像上的重投影点。
  PNP_ESTIMATES,
  PNP_CORNERS,
  PNP_REPROJECTION,
  PNP_ERROR_VECTORS,
  CORNER_REFINER_AXES,
  CORNER_REFINER_CANDIDATES,
  PNP_STATS,
};

/** @brief 单个 Foxglove Context 中全部视觉频道的 SDK 标识。 */
struct ChannelIds {
  std::uint64_t image{0};                   ///< 压缩图像频道 ID。
  std::uint64_t armor_annotations{0};       ///< 装甲标注频道 ID。
  std::uint64_t armor_stats{0};             ///< 检测器指标频道 ID。
  std::uint64_t debug_stats{0};             ///< 调试流水线指标频道 ID。
  std::uint64_t transforms{0};              ///< TF 频道 ID。
  std::uint64_t calibration{0};             ///< 相机标定频道 ID。
  std::uint64_t frustum{0};                 ///< 三维视锥频道 ID。
  std::uint64_t ground_truth{0};            ///< 三维真值频道 ID。
  std::uint64_t projection_annotations{0};  ///< 真值重投影标注频道 ID。
  std::uint64_t pnp_estimates{0};
  std::uint64_t pnp_corners{0};
  std::uint64_t pnp_reprojection{0};
  std::uint64_t pnp_error_vectors{0};
  std::uint64_t corner_refiner_axes{0};
  std::uint64_t corner_refiner_candidates{0};
  std::uint64_t pnp_stats{0};
};

/** @brief 单个话题的一次 Foxglove SDK 发布错误。 */
struct ChannelPublishError {
  VisionTopic topic{VisionTopic::IMAGE};                           ///< 发生错误的话题。
  ::foxglove::FoxgloveError error{::foxglove::FoxgloveError::Ok};  ///< SDK 错误码。
};

/** @brief 一批按需频道发布的无动态分配结果。 */
struct ChannelPublishResult {
  bool attempted{false};                         ///< 是否至少调用了一个频道的 log()。
  bool success{true};                            ///< 所有已尝试频道是否均成功。
  std::array<ChannelPublishError, 16> errors{};  ///< 每个固定话题最多记录一个错误。
  std::size_t error_count{0};                    ///< errors 中的有效元素数量。
};

/**
 * @brief 统一管理单个 Foxglove Context 内的全部视觉调试频道。
 *
 * 实时与录制 sink 各持有一个实例；两组话题名称和 Schema 相同，但 Context 与
 * 频道生命周期相互隔离。
 */
class VisionChannelSet final {
 public:
  /** @brief 创建全部固定频道；任一频道创建失败时抛出 std::runtime_error。 */
  explicit VisionChannelSet(const ::foxglove::Context& context);
  /** @brief 幂等关闭尚未关闭的全部频道。 */
  ~VisionChannelSet();

  VisionChannelSet(const VisionChannelSet&) = delete;
  VisionChannelSet& operator=(const VisionChannelSet&) = delete;

  /** @brief 返回频道 ID，供实时订阅注册和按需编码查询使用。 */
  [[nodiscard]] ChannelIds Ids() const noexcept;
  /** @brief 将批次中已编码且被需求选中的消息写入当前 Context。 */
  [[nodiscard]] ChannelPublishResult Publish(const PreparedFrame& frame,
                                             TopicDemand demand) noexcept;
  /** @brief 幂等关闭全部频道。 */
  void Close() noexcept;

 private:
  std::unique_ptr<::foxglove::schemas::CompressedImageChannel> image_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> armor_annotations_;
  std::unique_ptr<::foxglove::RawChannel> armor_stats_;
  std::unique_ptr<::foxglove::RawChannel> debug_stats_;
  std::unique_ptr<::foxglove::schemas::FrameTransformsChannel> transforms_;
  std::unique_ptr<::foxglove::schemas::CameraCalibrationChannel> calibration_;
  std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> frustum_;
  std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> ground_truth_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> projection_annotations_;
  std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> pnp_estimates_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> pnp_corners_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> pnp_reprojection_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> pnp_error_vectors_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> corner_refiner_axes_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> corner_refiner_candidates_;
  std::unique_ptr<::foxglove::RawChannel> pnp_stats_;
  bool closed_{false};  ///< 保证显式 Close() 与析构关闭幂等。
};

/** @brief 将内部话题枚举转换为稳定日志名称。 */
[[nodiscard]] const char* TopicName(VisionTopic topic) noexcept;

}  // namespace mv::tool::foxglove::pipeline
