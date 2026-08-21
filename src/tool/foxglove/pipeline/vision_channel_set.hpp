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
  IMAGE,                           ///< JPEG 压缩原图。
  ARMOR_ANNOTATIONS,               ///< 装甲四角框与标签。
  ARMOR_STATS,                     ///< 装甲检测器性能 JSON。
  LIGHTBAR_ANNOTATIONS,            ///< 独立灯条原始、预测及关联标注。
  LIGHTBAR_STATS,                  ///< 灯条检测与融合性能 JSON。
  DEBUG_STATS,                     ///< 调试发布流水线性能 JSON。
  TRANSFORMS,                      ///< world -> gimbal -> camera_optical TF。
  CALIBRATION,                     ///< 相机内参与畸变参数。
  FRUSTUM,                         ///< camera_optical 下的三维视锥。
  GROUND_TRUTH,                    ///< world 下的仿真三维真值。
  PROJECTILE_STATS,                ///< 仿真弹丸发射与命中累计统计。
  PROJECTION_ANNOTATIONS,          ///< 真值探针在相机图像上的重投影点。
  PNP_ESTIMATES,                   ///< 相机系与世界系下的 PnP 三维估计。
  PNP_CORNERS,                     ///< PnP 原始及精修输入角点。
  PNP_REPROJECTION,                ///< PnP 模型重投影线框。
  PNP_ERROR_VECTORS,               ///< 检测角点到仿真真值的误差向量。
  CORNER_REFINER_AXES,             ///< 角点精修灯条 PCA 轴。
  CORNER_REFINER_CANDIDATES,       ///< 角点精修搜索区间及候选点。
  PNP_STATS,                       ///< PnP 解算与角点精修指标 JSON。
  PREDICTION_SCENE,                ///< 预测整车与装甲三维场景。
  PREDICTION_STATE,                ///< 预测 EKF 状态 JSON。
  PREDICTION_TRUTH_OVERLAY,        ///< 预测中心与仿真真值误差线。
  PREDICTION_CURRENT_ANNOTATIONS,  ///< 当前预测装甲二维重投影。
  PREDICTION_FUTURE_ANNOTATIONS,   ///< 100 ms 预测装甲二维重投影。
  SELECTED_ARMOR_ANNOTATIONS,      ///< 火控选中装甲同帧二维重投影。
};

/** @brief 单个 Foxglove Context 中全部视觉频道的 SDK 标识。 */
struct ChannelIds {
  std::uint64_t image{0};                           ///< 压缩图像频道 ID。
  std::uint64_t armor_annotations{0};               ///< 装甲标注频道 ID。
  std::uint64_t armor_stats{0};                     ///< 检测器指标频道 ID。
  std::uint64_t lightbar_annotations{0};            ///< 灯条标注频道 ID。
  std::uint64_t lightbar_stats{0};                  ///< 灯条检测与融合指标频道 ID。
  std::uint64_t debug_stats{0};                     ///< 调试流水线指标频道 ID。
  std::uint64_t transforms{0};                      ///< TF 频道 ID。
  std::uint64_t calibration{0};                     ///< 相机标定频道 ID。
  std::uint64_t frustum{0};                         ///< 三维视锥频道 ID。
  std::uint64_t ground_truth{0};                    ///< 三维真值频道 ID。
  std::uint64_t projectile_stats{0};                ///< 弹丸统计频道 ID。
  std::uint64_t projection_annotations{0};          ///< 真值重投影标注频道 ID。
  std::uint64_t pnp_estimates{0};                   ///< PnP 三维估计频道 ID。
  std::uint64_t pnp_corners{0};                     ///< PnP 输入角点频道 ID。
  std::uint64_t pnp_reprojection{0};                ///< PnP 重投影频道 ID。
  std::uint64_t pnp_error_vectors{0};               ///< PnP 角点误差向量频道 ID。
  std::uint64_t corner_refiner_axes{0};             ///< 角点精修 PCA 轴频道 ID。
  std::uint64_t corner_refiner_candidates{0};       ///< 角点精修候选点频道 ID。
  std::uint64_t pnp_stats{0};                       ///< PnP 指标频道 ID。
  std::uint64_t prediction_scene{0};                ///< 预测场景频道 ID。
  std::uint64_t prediction_state{0};                ///< 预测状态频道 ID。
  std::uint64_t prediction_truth_overlay{0};        ///< 预测真值对照频道 ID。
  std::uint64_t prediction_current_annotations{0};  ///< 当前预测图像标注频道 ID。
  std::uint64_t prediction_future_annotations{0};   ///< 100 ms 预测图像标注频道 ID。
  std::uint64_t selected_armor_annotations{0};      ///< 火控选中装甲标注频道 ID。
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
  std::array<ChannelPublishError, 25> errors{};  ///< 每个固定话题最多记录一个错误。
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
  std::unique_ptr<::foxglove::schemas::CompressedImageChannel> image_;  ///< JPEG 图像频道。
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel>
      armor_annotations_;                                ///< 二维装甲标注频道。
  std::unique_ptr<::foxglove::RawChannel> armor_stats_;  ///< 装甲检测器指标频道。
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel>
      lightbar_annotations_;                                ///< 独立灯条标注频道。
  std::unique_ptr<::foxglove::RawChannel> lightbar_stats_;  ///< 灯条检测与融合指标频道。
  std::unique_ptr<::foxglove::RawChannel> debug_stats_;     ///< 调试流水线指标频道。
  std::unique_ptr<::foxglove::schemas::FrameTransformsChannel> transforms_;     ///< TF 频道。
  std::unique_ptr<::foxglove::schemas::CameraCalibrationChannel> calibration_;  ///< 标定频道。
  std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> frustum_;  ///< 三维视锥频道。
  std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> ground_truth_;  ///< 仿真真值频道。
  std::unique_ptr<::foxglove::RawChannel> projectile_stats_;  ///< 仿真弹丸累计统计频道。
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel>
      projection_annotations_;  ///< 真值重投影频道。
  std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> pnp_estimates_;  ///< PnP 估计频道。
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> pnp_corners_;  ///< 输入角点频道。
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel>
      pnp_reprojection_;  ///< PnP 重投影频道。
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel>
      pnp_error_vectors_;  ///< PnP 误差向量频道。
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel>
      corner_refiner_axes_;  ///< 角点精修 PCA 轴频道。
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel>
      corner_refiner_candidates_;                      ///< 角点精修候选点频道。
  std::unique_ptr<::foxglove::RawChannel> pnp_stats_;  ///< PnP 指标频道。
  std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> prediction_scene_;
  std::unique_ptr<::foxglove::RawChannel> prediction_state_;
  std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> prediction_truth_overlay_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> prediction_current_annotations_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> prediction_future_annotations_;
  std::unique_ptr<::foxglove::schemas::ImageAnnotationsChannel> selected_armor_annotations_;
  bool closed_{false};  ///< 保证显式 Close() 与析构关闭幂等。
};

/** @brief 将内部话题枚举转换为稳定日志名称。 */
[[nodiscard]] const char* TopicName(VisionTopic topic) noexcept;

}  // namespace mv::tool::foxglove::pipeline
