#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"

#include <cstdint>
#include <string>

#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::pnp {

/**
 * @brief 将正式检测链的有效 PnP 位姿编码为相机系和世界系装甲线框及相机观测射线。
 * @return 不包含有效检测估计时返回空 SceneUpdate。
 */
[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeEstimates(
    const modules::ArmorPnpFrameResult& result, const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 编码检测输入的青色原始角点，以及成功应用后的洋红色精修角点。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeCorners(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 将正式 PnP 最优位姿的模型重投影四角编码为绿色闭合图像线框。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeReprojection(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 编码角点到匹配仿真真值投影点的二维误差向量。
 *
 * 灰色表示原始网络角点误差，洋红色表示成功应用后的精修角点误差。
 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeErrorVectors(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 以蓝色线段和点编码角点精修诊断中的灯条 PCA 主轴与灰度质心。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeCornerRefinerAxes(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 编码角点精修的梯度搜索区间、逐扫描线候选点和融合结果。
 *
 * 融合结果成功应用时为绿色，否则为红色；无有效搜索范围或候选时自然省略。
 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeCornerRefinerCandidates(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 编码单帧 PnP 尝试、累计分组指标及角点精修诊断 JSON。
 *
 * 输出字段必须与 VisionChannelSet 注册的 mv.vision.ArmorPnpStats JSON Schema 保持同步。
 * result 中的累计摘要可能来自较早的 summary_sequence，逐目标 attempts 始终对应当前帧。
 */
[[nodiscard]] std::string EncodeStats(const modules::ArmorPnpFrameResult& result,
                                      std::uint64_t sequence,
                                      const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::pnp
