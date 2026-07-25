/**
 * @file light_vis_painter.cpp
 * @brief PaintLightBarsVis 实现
 *
 * 【为什么在这里重跑 findContours？】
 *   BasicArmorDetector::Detect() 不对外暴露灯条中间列表（属于 Impl 私有）。
 *   为保证颜色编码与检测器实际过滤结果一一对应，此处对同一张 binary 图
 *   重跑 findContours + minAreaRect / fitEllipse，以相同阈值判定过滤原因：
 *     绿色 —— ratio ✓ && angle ✓（通过所有过滤）
 *     橙色 —— ratio ✗（宽高比不符）
 *     紫色 —— angle ✗（角度超限）
 *
 * 【有意的轻微重复】
 *   过滤判据代码与 BasicArmorDetector::Impl::FindLightBars() 有意镜像，
 *   将可视化工具与检测器内部实现彻底解耦：修改检测器不会破坏可视化，
 *   修改可视化也不会影响检测逻辑，符合单一职责原则。
 */
#include "tool/debug/light_vis_painter.hpp"

#include <array>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace mv::tool {

cv::Mat PaintLightBarsVis(const cv::Mat&                                   binary,
                          const cv::Mat&                                   raw_frame,
                          const mv::modules::BasicArmorDetector::Params&  params) {
  cv::Mat vis = raw_frame.clone();

  if (binary.empty() || raw_frame.empty()) {
    return vis;
  }

  // 重新遍历轮廓，按过滤原因分色绘制
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) {
    const auto area = static_cast<float>(cv::contourArea(contour));
    if (area < params.min_area || contour.size() < 5) {
      continue;  // 面积/点数不足，不绘制
    }

    // 归一化 RotatedRect（height >= width）
    cv::RotatedRect rect = cv::minAreaRect(contour);
    if (rect.size.width > rect.size.height) {
      std::swap(rect.size.width, rect.size.height);
      rect.angle += 90.0F;
    }

    const float ratio = rect.size.width / rect.size.height;
    // fitEllipse 角度同 BasicArmorDetector::Impl::FindLightBars
    const float tilt = std::abs(cv::fitEllipse(contour).angle - 90.0F);

    cv::Scalar box_color;
    bool passed = false;
    if (ratio < params.min_light_ratio || ratio > params.max_light_ratio) {
      box_color = {0, 128, 255};  // 橙：宽高比不符
    } else if (tilt > params.max_light_angle) {
      box_color = {255, 0, 255};  // 紫：角度超限
    } else {
      box_color = {0, 255, 0};    // 绿：通过
      passed    = true;
    }

    // 绘制放大 1.4× 的旋转框（外扩让小灯条轮廓更易看清）
    std::array<cv::Point2f, 4> corners{};
    rect.points(corners.data());
    constexpr float kExpand = 1.4F;
    for (auto& corner : corners) {
      corner = rect.center + kExpand * (corner - rect.center);
    }
    for (int idx = 0; idx < 4; ++idx) {
      cv::line(vis, corners[static_cast<std::size_t>(idx)],
               corners[static_cast<std::size_t>((idx + 1) % 4)], box_color, 2);
    }

    // 绿色通过的灯条额外标注顶/底端点
    if (passed) {
      const float ax_rad = (rect.angle + 90.0F) * static_cast<float>(CV_PI) / 180.0F;
      const cv::Point2f half(std::cos(ax_rad) * rect.size.height * 0.5F,
                             std::sin(ax_rad) * rect.size.height * 0.5F);
      const cv::Point2f point_a = rect.center - half;
      const cv::Point2f point_b = rect.center + half;
      const cv::Point2f top_pt    = (point_a.y < point_b.y) ? point_a : point_b;
      const cv::Point2f bottom_pt = (point_a.y < point_b.y) ? point_b : point_a;
      cv::circle(vis, top_pt,    5, cv::Scalar(255, 120, 0),   -1);  // 橙：顶
      cv::circle(vis, bottom_pt, 5, cv::Scalar(0,   120, 255), -1);  // 蓝：底
    }
  }

  return vis;
}

}  // namespace mv::tool
