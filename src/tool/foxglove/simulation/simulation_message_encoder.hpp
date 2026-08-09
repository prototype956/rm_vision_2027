#pragma once

#include "hal/camera/i_camera.hpp"

#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::simulation {

/**
 * @brief 将机器人中心、朝向和装甲投影探针编码为 world 下的三维真值图元。
 *
 * 图元带有限生命周期；仿真器删除目标后，即使没有显式删除消息也会自动消失。
 */
[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeGroundTruth(
    const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 使用同帧内外参将 world 中的装甲中心探针重投影到相机图像。
 *
 * 相机后方和图像边界外的探针会被过滤。没有可见探针时仍返回一个空 POINTS
 * 标注，以便 Foxglove 清除上一帧黄色点。
 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeProjectionAnnotations(
    const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::simulation
