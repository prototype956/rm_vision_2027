/**
 * @file light_vis_painter.hpp
 * @brief 灯条过滤原因色码可视化（从 BasicArmorDetector 解耦出的调试工具）
 *
 * 职责：
 *   给定形态学后的二值图 + 原始帧 + 检测参数，重新遍历所有轮廓并按过滤原因
 *   用不同颜色绘制旋转框：
 *     绿色  —— 通过所有过滤（ratio ✓ && angle ✓）
 *     橙色  —— 宽高比不符（ratio ✗）
 *     紫色  —— 角度超限（angle ✗）
 *   绿色灯条额外在顶端/底端画橙/蓝圆点。
 *
 * 用法（通常在 ViewRenderer::Render LIGHTS 分支调用）：
 * @code
 *   cv::Mat vis = mv::tool::PaintLightBarsVis(dbg.binary, raw_frame, params);
 * @endcode
 *
 * 设计说明：
 *   纯自由函数，无状态，线程安全（只读取入参，生成新矩阵）。
 *   依赖 BasicArmorDetector::Params（公开结构体），不依赖检测器对象本身。
 */
#pragma once

#include "modules/armor_detector/basic_armor_detector.hpp"

#include <opencv2/core.hpp>

namespace mv::tool {

/**
 * @brief 生成灯条过滤原因色码可视化图
 *
 * @param binary    形态学处理后的二值图（BasicArmorDetector::DebugData::binary）
 * @param raw_frame 原始彩色帧（作为背景）
 * @param params    当前检测器参数（过滤条件来源）
 * @return          原始帧叠加各色旋转框的彩色图像（与 raw_frame 同尺寸）
 */
[[nodiscard]] cv::Mat PaintLightBarsVis(
    const cv::Mat&                                  binary,
    const cv::Mat&                                  raw_frame,
    const mv::modules::BasicArmorDetector::Params& params);

}  // namespace mv::tool
