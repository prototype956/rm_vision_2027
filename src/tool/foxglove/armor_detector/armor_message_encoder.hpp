#pragma once

#include "modules/armor_detector/armor_detector.hpp"

#include <cstdint>
#include <string>

#include <foxglove/schemas.hpp>
#include <span>

namespace mv::tool::foxglove::armor_detector {

/**
 * @brief 将装甲检测结果编码为二维图像标注。
 *
 * 每个目标生成一个四角闭合线框和一条颜色、类别、置信度文本；零检测帧仍携带
 * 一个不可见时间戳点，使 Foxglove 能清除上一帧标注。
 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeAnnotations(
    std::span<const modules::ArmorDetection> detections,
    const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 编码只属于装甲检测器的单帧 JSON 指标。
 *
 * 输出字段必须与 VisionChannelSet 注册的 mv.vision.ArmorDetectorStats JSON Schema
 * 保持同步，耗时字段单位统一为毫秒。
 */
[[nodiscard]] std::string EncodeDetectorStats(const modules::DetectorStats& stats,
                                              std::uint64_t sequence,
                                              const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::armor_detector
