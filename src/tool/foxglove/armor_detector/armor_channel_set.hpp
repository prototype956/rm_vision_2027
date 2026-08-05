#pragma once

#include "tool/foxglove/armor_detector/armor_message_encoder.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::armor_detector {

/**
 * @brief 装甲调试组件固定发布的三类话题。
 */
enum class ArmorTopic {
  IMAGE,        ///< JPEG 压缩原图。
  ANNOTATIONS,  ///< 装甲四角与标签标注。
  STATS,        ///< JSON 性能指标。
};

/**
 * @brief 同一个 Context 中三个装甲频道的 SDK 标识。
 */
struct ChannelIds {
  std::uint64_t image{0};        ///< 压缩图像频道 ID。
  std::uint64_t annotations{0};  ///< 二维标注频道 ID。
  std::uint64_t stats{0};        ///< JSON 指标频道 ID。
};

/**
 * @brief 单个话题的一次 Foxglove SDK 发布错误。
 */
struct ChannelPublishError {
  ArmorTopic topic{ArmorTopic::IMAGE};                             ///< 发生错误的话题。
  ::foxglove::FoxgloveError error{::foxglove::FoxgloveError::Ok};  ///< SDK 错误码。
};

/**
 * @brief 一批按需频道发布的无动态分配结果。
 */
struct ChannelPublishResult {
  bool attempted{false};                        ///< 是否至少调用了一个频道的 log()。
  bool success{true};                           ///< 所有已尝试频道是否均成功。
  std::array<ChannelPublishError, 3> errors{};  ///< 每个固定话题最多记录一个错误。
  std::size_t error_count{0};                   ///< errors 中的有效元素数量。
};

/**
 * @brief 绑定到单个 Context 的装甲图像、标注和指标频道集合。
 *
 * 实时和录制 sink 分别持有一个实例，频道定义保持一致，但 Context 相互隔离，因此
 * 任一 sink 关闭不会影响另一方。
 */
class ArmorChannelSet final {
 public:
  /**
   * @brief 在指定 Context 中创建三个固定话题。
   * @throws std::runtime_error 任一 SDK 频道创建失败。
   */
  explicit ArmorChannelSet(const ::foxglove::Context& context);

  /** @brief 幂等关闭仍打开的频道。 */
  ~ArmorChannelSet();

  ArmorChannelSet(const ArmorChannelSet&) = delete;
  ArmorChannelSet& operator=(const ArmorChannelSet&) = delete;

  /** @brief 获取频道 ID，用于 WebSocket 订阅注册与需求查询。 */
  [[nodiscard]] ChannelIds Ids() const noexcept;

  /**
   * @brief 按需求将同一个预编码消息批次写入当前 Context。
   * @return 已尝试话题的整体结果及逐话题 SDK 错误。
   */
  [[nodiscard]] ChannelPublishResult Publish(const PreparedFrame& frame,
                                             TopicDemand demand) noexcept;

  /** @brief 幂等关闭三个频道。 */
  void Close() noexcept;

 private:
  std::unique_ptr<::foxglove::schemas::CompressedImageChannel> image_;         ///< JPEG 频道。
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> annotations_;  ///< 标注频道。
  std::unique_ptr<::foxglove::RawChannel> stats_;  ///< JSON Schema 指标频道。
  bool closed_{false};                             ///< 保证显式关闭幂等。
};

/**
 * @brief 将内部话题枚举转换为稳定的日志名称。
 */
[[nodiscard]] const char* TopicName(ArmorTopic topic) noexcept;

}  // namespace mv::tool::foxglove::armor_detector
