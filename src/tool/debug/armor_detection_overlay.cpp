#include "tool/debug/armor_detection_overlay.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

namespace mv::tool {
namespace {

constexpr int K_BOX_THICKNESS = 2;
constexpr int K_TEXT_THICKNESS = 1;
constexpr int K_TEXT_OUTLINE_THICKNESS = 3;
constexpr int K_TEXT_TOP_MARGIN = 6;
constexpr double K_TEXT_SCALE = 0.55;

// OpenCV 使用 BGR 顺序；蓝色使用较亮色调，保证在深色画面上清晰可见。
cv::Scalar DetectionColor(modules::ArmorColor color) noexcept {
  return color == modules::ArmorColor::RED ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 128, 0);
}

// 将标签基线限制在图像内，并尽可能保证整段文字不会越过左右边界。
cv::Point LabelOrigin(const cv::Mat& image, const modules::ArmorDetection& detection,
                      const std::string& text) {
  int baseline = 0;
  const auto TEXT_SIZE =
      cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, K_TEXT_SCALE, K_TEXT_THICKNESS, &baseline);
  const int MAX_X = std::max(0, image.cols - TEXT_SIZE.width);
  const int MIN_Y = std::min(image.rows - 1, TEXT_SIZE.height);
  const int MAX_Y = std::max(MIN_Y, image.rows - baseline - 1);
  return {
      std::clamp(cvRound(detection.bounding_box.x), 0, MAX_X),
      std::clamp(cvRound(detection.bounding_box.y) - K_TEXT_TOP_MARGIN, MIN_Y, MAX_Y),
  };
}

// 黑色粗字作为底层描边，使目标标签在明暗背景上均保持可读。
void DrawOutlinedText(cv::Mat& image, const std::string& text, const cv::Point& origin,
                      const cv::Scalar& color) {
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, K_TEXT_SCALE, cv::Scalar(0, 0, 0),
              K_TEXT_OUTLINE_THICKNESS, cv::LINE_AA);
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, K_TEXT_SCALE, color, K_TEXT_THICKNESS,
              cv::LINE_AA);
}

}  // namespace

void DrawArmorDetections(cv::Mat& image, const std::vector<modules::ArmorDetection>& detections) {
  if (image.empty()) {
    throw std::invalid_argument("armor detection overlay image must not be empty");
  }
  if (image.type() != CV_8UC3) {
    throw std::invalid_argument("armor detection overlay image must be CV_8UC3 BGR");
  }

  for (const auto& detection : detections) {
    std::vector<cv::Point> polygon;
    polygon.reserve(detection.corners.size());
    for (const auto& corner : detection.corners) {
      polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));
    }

    const auto COLOR = DetectionColor(detection.color);
    cv::polylines(image, polygon, true, COLOR, K_BOX_THICKNESS, cv::LINE_AA);

    const auto TEXT = fmt::format("{} {} {:.2f}", modules::ArmorColorName(detection.color),
                                  modules::ArmorLabelName(detection.label), detection.objectness);
    DrawOutlinedText(image, TEXT, LabelOrigin(image, detection, TEXT), COLOR);
  }
}

}  // namespace mv::tool
