/**
 * @file traditional_vision_debug_publisher.cpp
 * @brief TraditionalVisionDebugPublisher 实现
 *
 * 【实现策略】
 *   - 复用 ImagePublisher，统一走 RawImage/mono8 发布路径；
 *   - 在本模块内完成 ROI 局部图 -> 全图尺寸的还原，调用方只需传当前 roi_rect；
 *   - 单帧内按固定 topic 发布：vision/debug/diff、vision/debug/binary、
 *     vision/debug/lights、vision/debug/roi。
 */
#include "tool/foxglove/detail/traditional_vision_debug_publisher.hpp"

#include "tool/foxglove/detail/image_publisher.hpp"

#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace mv::tool::detail {

TraditionalVisionDebugPublisher::TraditionalVisionDebugPublisher(foxglove::Context ctx)
    : ctx_(std::move(ctx)), image_pub_(std::make_unique<ImagePublisher>(ctx_, false, 80, 0, 0)) {}

TraditionalVisionDebugPublisher::~TraditionalVisionDebugPublisher() = default;

cv::Mat TraditionalVisionDebugPublisher::RestoreToFullFrame(const cv::Mat& src,
                                                            const cv::Rect2i& roi_rect,
                                                            const cv::Size& frame_size) {
  if (src.empty()) {
    return {};
  }
  if (roi_rect.area() == 0 || src.size() == frame_size) {
    return src;
  }

  cv::Mat full = cv::Mat::zeros(frame_size, src.type());
  const cv::Rect PASTE_RECT{roi_rect.tl(), src.size()};
  const cv::Rect SAFE = PASTE_RECT & cv::Rect(cv::Point{0, 0}, frame_size);
  if (SAFE.area() <= 0) {
    return full;
  }

  const cv::Rect SRC_RECT{cv::Point{0, 0}, SAFE.size()};
  src(SRC_RECT).copyTo(full(SAFE));
  return full;
}

void TraditionalVisionDebugPublisher::Publish(const cv::Mat& diff, const cv::Mat& binary,
                                              const cv::Rect2i& roi_rect,
                                              const cv::Size& frame_size, const cv::Mat& raw_frame,
                                              const LightVisParams& light_params, uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);

  const cv::Mat FULL_DIFF = RestoreToFullFrame(diff, roi_rect, frame_size);
  if (!FULL_DIFF.empty()) {
    image_pub_->Publish(FULL_DIFF, "vision/debug/diff", "camera", ts_ns);
  }

  const cv::Mat FULL_BINARY = RestoreToFullFrame(binary, roi_rect, frame_size);
  if (!FULL_BINARY.empty()) {
    image_pub_->Publish(FULL_BINARY, "vision/debug/binary", "camera", ts_ns);

    const cv::Mat LIGHTS_VIS = BuildLightsVisualization(FULL_BINARY, raw_frame, light_params);
    if (!LIGHTS_VIS.empty()) {
      image_pub_->Publish(LIGHTS_VIS, "vision/debug/lights", "camera", ts_ns);
    }
  }

  const cv::Mat ROI_VIS = BuildRoiVisualization(raw_frame, roi_rect);
  if (!ROI_VIS.empty()) {
    image_pub_->Publish(ROI_VIS, "vision/debug/roi", "camera", ts_ns);
  }
}

cv::Mat TraditionalVisionDebugPublisher::BuildLightsVisualization(
    const cv::Mat& binary_full, const cv::Mat& raw_frame, const LightVisParams& light_params) {
  cv::Mat visualization = raw_frame.clone();
  if (binary_full.empty() || raw_frame.empty()) {
    return visualization;
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_full, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) {
    const float AREA = static_cast<float>(cv::contourArea(contour));
    if (AREA < light_params.min_area || contour.size() < 5) {
      continue;
    }

    cv::RotatedRect rect = cv::minAreaRect(contour);
    if (rect.size.width > rect.size.height) {
      std::swap(rect.size.width, rect.size.height);
      rect.angle += 90.0F;
    }

    const float RATIO = rect.size.width / rect.size.height;
    const float TILT = std::abs(cv::fitEllipse(contour).angle - 90.0F);

    cv::Scalar box_color;
    bool is_passed = false;
    if (RATIO < light_params.min_light_ratio || RATIO > light_params.max_light_ratio) {
      box_color = {0, 128, 255};  // 橙：宽高比不符
    } else if (TILT > light_params.max_light_angle) {
      box_color = {255, 0, 255};  // 紫：角度超限
    } else {
      box_color = {0, 255, 0};  // 绿：通过
      is_passed = true;
    }

    std::array<cv::Point2f, 4> corners{};
    rect.points(corners.data());
    constexpr float K_EXPAND_FACTOR = 1.4F;
    for (auto& corner : corners) {
      corner = rect.center + K_EXPAND_FACTOR * (corner - rect.center);
    }
    cv::line(visualization, corners[0], corners[1], box_color, 2);
    cv::line(visualization, corners[1], corners[2], box_color, 2);
    cv::line(visualization, corners[2], corners[3], box_color, 2);
    cv::line(visualization, corners[3], corners[0], box_color, 2);

    if (is_passed) {
      const float LONG_AXIS_RAD = (rect.angle + 90.0F) * static_cast<float>(CV_PI) / 180.0F;
      const cv::Point2f HALF(std::cos(LONG_AXIS_RAD) * rect.size.height * 0.5F,
                             std::sin(LONG_AXIS_RAD) * rect.size.height * 0.5F);
      const cv::Point2f POINT_A = rect.center - HALF;
      const cv::Point2f POINT_B = rect.center + HALF;
      const cv::Point2f TOP_POINT = (POINT_A.y < POINT_B.y) ? POINT_A : POINT_B;
      const cv::Point2f BOTTOM_POINT = (POINT_A.y < POINT_B.y) ? POINT_B : POINT_A;
      cv::circle(visualization, TOP_POINT, 5, cv::Scalar(255, 120, 0), -1);     // 顶端
      cv::circle(visualization, BOTTOM_POINT, 5, cv::Scalar(0, 120, 255), -1);  // 底端
    }
  }

  return visualization;
}

cv::Mat TraditionalVisionDebugPublisher::BuildRoiVisualization(const cv::Mat& raw_frame,
                                                               const cv::Rect2i& roi_rect) {
  cv::Mat visualization = raw_frame.clone();
  if (visualization.empty()) {
    return visualization;
  }

  if (roi_rect.area() > 0) {
    cv::rectangle(visualization, roi_rect, cv::Scalar(0, 255, 255), 2);
    cv::putText(visualization, "ROI", roi_rect.tl() + cv::Point(2, -4), cv::FONT_HERSHEY_SIMPLEX,
                0.50, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  } else {
    cv::putText(visualization, "ROI: FULL FRAME", cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.50,
                cv::Scalar(80, 80, 255), 1, cv::LINE_AA);
  }

  return visualization;
}

}  // namespace mv::tool::detail
