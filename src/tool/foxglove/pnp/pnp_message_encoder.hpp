#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_pnp/armor_pnp.hpp"

#include <cstdint>
#include <string>

#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::pnp {

/** @brief 将有效 PnP 位姿编码为相机系和世界系下的装甲线框及观测射线。 */
[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeEstimates(
    const modules::ArmorPnpFrameResult& result, const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 编码检测输入的原始角点，以及成功精修后的最终角点。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeCorners(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 将有效 PnP 解算结果的模型重投影四角编码为闭合图像线框。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeReprojection(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 编码原始及成功精修角点到仿真真值投影点的二维误差向量。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeErrorVectors(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 编码角点精修诊断中的灯条 PCA 主轴与中心点。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeCornerRefinerAxes(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 编码角点精修的梯度搜索区间、扫描候选点和融合结果。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeCornerRefinerCandidates(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 编码单帧 PnP 尝试、累计分组指标及角点精修诊断 JSON。
 *
 * 输出字段必须与 VisionChannelSet 注册的 mv.vision.ArmorPnpStats JSON Schema 保持同步。
 */
[[nodiscard]] std::string EncodeStats(const modules::ArmorPnpFrameResult& result,
                                      std::uint64_t sequence,
                                      const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::pnp
