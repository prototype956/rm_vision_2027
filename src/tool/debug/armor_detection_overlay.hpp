/**
 * @file armor_detection_overlay.hpp
 * @brief 装甲检测结果的统一 OpenCV 调试绘制接口。
 */
#pragma once

#include "modules/armor_detector/armor_detector.hpp"

#include <vector>

#include <opencv2/core/mat.hpp>

namespace mv::tool {

/**
 * @brief 在图像上原地绘制装甲四角框、颜色、类别和置信度。
 *
 * 空检测结果不会修改图像。此函数只负责单个目标的可视化，不绘制调用方特有的
 * 帧号、性能指标或统计 HUD。
 *
 * @param image 待绘制的非空 CV_8UC3 BGR 图像。
 * @param detections 装甲检测器输出的结果集合。
 * @throws std::invalid_argument image 为空或不是 CV_8UC3 时抛出。
 */
void DrawArmorDetections(cv::Mat& image, const std::vector<modules::ArmorDetection>& detections);

}  // namespace mv::tool
