#include "modules/armor_light_detector/armor_light_detector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <optional>

namespace mv::modules {
namespace {

using Clock = std::chrono::steady_clock;

std::optional<double> SampleLineMean(const cv::Mat& gray, cv::Point2f first, cv::Point2f second) {
  const double LENGTH = cv::norm(second - first);
  const int SAMPLES = std::max(2, static_cast<int>(std::ceil(LENGTH)) + 1);
  double sum = 0.0;
  int count = 0;
  for (int index = 0; index < SAMPLES; ++index) {
    const float RATIO = static_cast<float>(index) / static_cast<float>(SAMPLES - 1);
    const cv::Point POINT(first + (second - first) * RATIO);
    if (POINT.x < 0 || POINT.x >= gray.cols || POINT.y < 0 || POINT.y >= gray.rows)
      continue;
    sum += gray.at<std::uint8_t>(POINT);
    ++count;
  }
  if (count < 2)
    return std::nullopt;
  return sum / static_cast<double>(count);
}

std::array<cv::Point2f, 2> LongSideEndpoints(const cv::RotatedRect& rectangle) {
  std::array<cv::Point2f, 4> points{};
  rectangle.points(points.data());
  const double FIRST_LENGTH = cv::norm(points[1] - points[0]);
  const double SECOND_LENGTH = cv::norm(points[2] - points[1]);
  std::array<cv::Point2f, 2> endpoints;
  if (FIRST_LENGTH >= SECOND_LENGTH) {
    endpoints = {(points[0] + points[3]) * 0.5F, (points[1] + points[2]) * 0.5F};
  } else {
    endpoints = {(points[0] + points[1]) * 0.5F, (points[2] + points[3]) * 0.5F};
  }
  if (endpoints[0].y > endpoints[1].y)
    std::swap(endpoints[0], endpoints[1]);
  return endpoints;
}

double ContourColorDifference(const cv::Mat& bgr, const std::vector<cv::Point>& contour,
                              ArmorColor enemy_color) {
  const cv::Rect BOX = cv::boundingRect(contour) & cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (BOX.empty())
    return 0.0;
  std::vector<cv::Point> local;
  local.reserve(contour.size());
  for (const auto& point : contour)
    local.emplace_back(point.x - BOX.x, point.y - BOX.y);
  cv::Mat mask = cv::Mat::zeros(BOX.size(), CV_8UC1);
  const std::vector<std::vector<cv::Point>> LOCAL_CONTOURS{std::move(local)};
  cv::drawContours(mask, LOCAL_CONTOURS, 0, cv::Scalar(255), cv::FILLED);
  const cv::Scalar MEAN = cv::mean(bgr(BOX), mask);
  return enemy_color == ArmorColor::RED ? MEAN[2] - MEAN[0] : MEAN[0] - MEAN[2];
}

}  // namespace

const char* LightbarThresholdSourceName(LightbarThresholdSource source) noexcept {
  switch (source) {
    case LightbarThresholdSource::FIXED:
      return "fixed";
    case LightbarThresholdSource::NETWORK_REFERENCE:
      return "network_reference";
  }
  return "unknown";
}

ArmorLightDetector::ArmorLightDetector(ArmorLightDetectorConfig config, ArmorColor enemy_color)
    : config_(config), enemy_color_(enemy_color) {}

LightbarDetectionResult ArmorLightDetector::Detect(
    const cv::Mat& bgr_image, const cv::Mat& gray_image, std::span<const ArmorDetection> detections,
    std::span<const CornerRefinementResult> refinements) const noexcept {
  const auto START = Clock::now();
  LightbarDetectionResult result;
  result.stats.enabled = config_.enabled;
  const auto FINISH = [&]() {
    result.stats.kept_candidates = result.detections.size();
    result.stats.elapsed_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - START).count();
    return result;
  };
  try {
    if (!config_.enabled) {
      result.stats.binary_threshold = config_.fixed_binary_threshold;
      result.stats.rejection_reason = "disabled";
      return FINISH();
    }
    if (bgr_image.empty() || gray_image.empty() || bgr_image.type() != CV_8UC3 ||
        gray_image.type() != CV_8UC1 || bgr_image.size() != gray_image.size()) {
      result.stats.valid_input = false;
      result.stats.rejection_reason = "invalid_image";
      return FINISH();
    }

    std::vector<double> reference_brightness;
    if (detections.size() == refinements.size()) {
      reference_brightness.reserve(detections.size() * 2);
      for (std::size_t index = 0; index < detections.size(); ++index) {
        const auto& corners = refinements[index].success ? refinements[index].refined_corners
                                                         : detections[index].corners;
        for (const auto& endpoints :
             {std::pair{corners[0], corners[3]}, std::pair{corners[1], corners[2]}}) {
          if (const auto MEAN = SampleLineMean(gray_image, endpoints.first, endpoints.second))
            reference_brightness.push_back(*MEAN);
        }
      }
    }
    result.stats.reference_lightbars = reference_brightness.size();
    int threshold = config_.fixed_binary_threshold;
    if (!reference_brightness.empty()) {
      const double MEAN =
          std::accumulate(reference_brightness.begin(), reference_brightness.end(), 0.0) /
          static_cast<double>(reference_brightness.size());
      threshold = std::clamp(static_cast<int>(std::lround(MEAN)) - config_.network_reference_offset,
                             config_.minimum_binary_threshold, config_.maximum_binary_threshold);
      result.stats.threshold_source = LightbarThresholdSource::NETWORK_REFERENCE;
    }
    result.stats.binary_threshold = threshold;

    cv::Mat binary;
    cv::threshold(gray_image, binary, threshold, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    // 点数门限针对原始轮廓采样；CHAIN_APPROX_SIMPLE 会把理想矩形压成四点而误拒绝。
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    result.stats.contours = contours.size();

    std::size_t input_index = 0;
    for (const auto& contour : contours) {
      if (static_cast<int>(contour.size()) < config_.minimum_contour_points ||
          std::abs(cv::contourArea(contour)) < config_.minimum_contour_area_px2) {
        continue;
      }
      const cv::RotatedRect RECTANGLE = cv::minAreaRect(contour);
      const double LENGTH = std::max(RECTANGLE.size.width, RECTANGLE.size.height);
      const double WIDTH = std::min(RECTANGLE.size.width, RECTANGLE.size.height);
      if (LENGTH < config_.minimum_length_px || LENGTH <= 1.0e-6)
        continue;
      const double RATIO = WIDTH / LENGTH;
      const auto ENDPOINTS = LongSideEndpoints(RECTANGLE);
      const cv::Point2f DELTA = ENDPOINTS[1] - ENDPOINTS[0];
      const double TILT = std::atan2(std::abs(static_cast<double>(DELTA.x)),
                                     std::abs(static_cast<double>(DELTA.y)));
      if (RATIO < config_.minimum_width_length_ratio ||
          RATIO > config_.maximum_width_length_ratio || TILT > config_.maximum_tilt_rad) {
        continue;
      }
      ++result.stats.geometry_candidates;
      const double COLOR_DIFFERENCE = ContourColorDifference(bgr_image, contour, enemy_color_);
      if (COLOR_DIFFERENCE < config_.minimum_color_difference)
        continue;
      ++result.stats.color_candidates;
      const double AREA = std::abs(cv::contourArea(contour));
      result.detections.push_back(
          {.input_index = input_index++,
           .color = enemy_color_,
           .top = ENDPOINTS[0],
           .bottom = ENDPOINTS[1],
           .center = (ENDPOINTS[0] + ENDPOINTS[1]) * 0.5F,
           .length_px = LENGTH,
           .width_px = WIDTH,
           .angle_rad = std::atan2(static_cast<double>(DELTA.x), static_cast<double>(DELTA.y)),
           .color_difference = COLOR_DIFFERENCE,
           .score = COLOR_DIFFERENCE * AREA});
    }
    std::sort(result.detections.begin(), result.detections.end(),
              [](const auto& left, const auto& right) { return left.score > right.score; });
    if (result.detections.size() > static_cast<std::size_t>(config_.maximum_candidates))
      result.detections.resize(static_cast<std::size_t>(config_.maximum_candidates));
    for (std::size_t index = 0; index < result.detections.size(); ++index)
      result.detections[index].input_index = index;
    return FINISH();
  } catch (...) {
    result.detections.clear();
    result.stats.valid_input = false;
    result.stats.rejection_reason = "processing_error";
    return FINISH();
  }
}

}  // namespace mv::modules
