#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <string>

#include <opencv2/imgproc.hpp>
#include <optional>

namespace mv::modules {
namespace {

using Clock = std::chrono::steady_clock;

struct Lightbar {
  cv::Point2f top{};
  cv::Point2f bottom{};
  cv::Point2f center{};
  cv::Point2f axis{};
  double length{0.0};
  double width{0.0};
  cv::RotatedRect rectangle{};
};

struct SymmetryAxis {
  cv::Point2f centroid{};
  cv::Point2f direction{};
  double mean_brightness{0.0};
};

bool Finite(const cv::Point2f& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

Lightbar MakeLightbar(cv::Point2f top, cv::Point2f bottom) {
  Lightbar light;
  light.top = top;
  light.bottom = bottom;
  light.center = (top + bottom) * 0.5F;
  light.length = cv::norm(top - bottom);
  // 与 JLU LightCornerCorrector 保持一致，用固定灯条长宽比构造待分析区域。
  light.width = light.length / 6.7;
  if (light.length > 1.0e-6) {
    light.axis = (top - bottom) * static_cast<float>(1.0 / light.length);
    const float angle =
        static_cast<float>(std::atan2(light.axis.y, light.axis.x) * 180.0 / CV_PI + 90.0);
    light.rectangle = cv::RotatedRect(
        light.center, {static_cast<float>(light.width), static_cast<float>(light.length)}, angle);
  }
  return light;
}

std::optional<SymmetryAxis> FindSymmetryAxis(const cv::Mat& gray, const Lightbar& light,
                                             const ArmorCornerRefinerConfig& config,
                                             CornerRefinementStatus& failure) {
  cv::Rect box = light.rectangle.boundingRect();
  box.x -= static_cast<int>(box.width * config.padding_scale);
  box.y -= static_cast<int>(box.height * config.padding_scale);
  box.width += static_cast<int>(box.width * config.padding_scale * 2.0);
  box.height += static_cast<int>(box.height * config.padding_scale * 2.0);
  box &= cv::Rect(0, 0, gray.cols, gray.rows);
  if (box.empty()) {
    failure = CornerRefinementStatus::ROI_INVALID;
    return std::nullopt;
  }

  const cv::Mat source_roi = gray(box);
  // 亮度门限使用原始灰度；归一化图像仅用于降低曝光尺度对灰度矩主轴的影响。
  const double mean_brightness = cv::mean(source_roi)[0];
  if (mean_brightness <= config.lightbar_min_mean_brightness) {
    failure = CornerRefinementStatus::LIGHT_TOO_DARK;
    return std::nullopt;
  }

  cv::Mat roi;
  source_roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0.0, config.normalize_max_brightness, cv::NORM_MINMAX);
  const cv::Moments moments = cv::moments(roi, false);
  if (std::abs(moments.m00) <= 1.0e-9) {
    failure = CornerRefinementStatus::MOMENTS_ZERO;
    return std::nullopt;
  }

  const double mu20 = moments.mu20;
  const double mu11 = moments.mu11;
  const double mu02 = moments.mu02;
  if (mu20 == 0.0 && mu11 == 0.0 && mu02 == 0.0) {
    failure = CornerRefinementStatus::PCA_DEGENERATE;
    return std::nullopt;
  }

  // 二阶中心矩的解析主轴等价于二维 PCA，避免为 ROI 显式构造像素点集。
  const double theta = 0.5 * std::atan2(2.0 * mu11, mu20 - mu02);
  cv::Point2f axis(static_cast<float>(std::cos(theta)), static_cast<float>(std::sin(theta)));
  const double norm = cv::norm(axis);
  if (norm <= 1.0e-9 || !Finite(axis)) {
    failure = CornerRefinementStatus::PCA_DEGENERATE;
    return std::nullopt;
  }
  axis *= static_cast<float>(1.0 / norm);
  // 固定主轴符号，使 operation=1 始终朝图像上方搜索，结果不受特征向量符号影响。
  if (axis.y > 0.0F)
    axis = -axis;

  const cv::Point2f centroid(static_cast<float>(moments.m10 / moments.m00 + box.x),
                             static_cast<float>(moments.m01 / moments.m00 + box.y));
  return SymmetryAxis{centroid, axis, mean_brightness};
}

std::optional<cv::Point2f> FindCorner(const cv::Mat& gray, const Lightbar& light,
                                      const SymmetryAxis& axis, int operation,
                                      const ArmorCornerRefinerConfig& config,
                                      EndpointRefinementDiagnostic& diagnostic) {
  const float dx = axis.direction.x * static_cast<float>(operation);
  const float dy = axis.direction.y * static_cast<float>(operation);
  const float start_distance = static_cast<float>(light.length * config.search_start_ratio);
  const float search_distance =
      static_cast<float>(light.length * (config.search_end_ratio - config.search_start_ratio));
  diagnostic.search_start = axis.centroid + cv::Point2f(dx, dy) * start_distance;
  diagnostic.search_end = diagnostic.search_start + cv::Point2f(dx, dy) * search_distance;

  const auto in_image = [&gray](const cv::Point& point) {
    return point.x >= 0 && point.x < gray.cols && point.y >= 0 && point.y < gray.rows;
  };
  // 逐扫描线仅沿图像 x 轴平移，这是对 JLU 原实现的有意复刻，并非灯条法向偏移。
  const int scan_count_basis = static_cast<int>(light.width) - 2;
  const int half_count = static_cast<int>(std::round(scan_count_basis / 2));
  for (int offset = -half_count; offset <= half_count; ++offset) {
    const float x0 = axis.centroid.x + start_distance * dx + static_cast<float>(offset);
    const float y0 = axis.centroid.y + start_distance * dy;
    cv::Point2f previous(x0, y0);
    cv::Point2f corner(x0, y0);
    float max_brightness_difference = 0.0F;
    bool found = false;

    for (float x = x0 + dx, y = y0 + dy; std::hypot(x - x0, y - y0) < search_distance;
         x += dx, y += dy) {
      const cv::Point current_pixel(cv::Point2f(x, y));
      const cv::Point previous_pixel(previous);
      if (!in_image(current_pixel) || !in_image(previous_pixel))
        break;
      // 端点定义为灯条内部亮像素向外过渡时最大的正灰度下降。
      const float brightness_difference =
          static_cast<float>(gray.at<std::uint8_t>(previous_pixel)) -
          static_cast<float>(gray.at<std::uint8_t>(current_pixel));
      if (brightness_difference > max_brightness_difference &&
          gray.at<std::uint8_t>(previous_pixel) > axis.mean_brightness) {
        max_brightness_difference = brightness_difference;
        corner = previous;
        found = true;
      }
      previous = cv::Point2f(x, y);
    }
    if (found)
      diagnostic.scan_candidates.push_back(corner);
  }

  if (diagnostic.scan_candidates.empty())
    return std::nullopt;
  const cv::Point2f sum = std::accumulate(diagnostic.scan_candidates.begin(),
                                          diagnostic.scan_candidates.end(), cv::Point2f{});
  diagnostic.found = true;
  diagnostic.candidate = sum * static_cast<float>(1.0 / diagnostic.scan_candidates.size());
  return diagnostic.candidate;
}

}  // namespace

ArmorCornerRefinerConfig ParseArmorCornerRefinerConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "armor corner refiner config";
  ConfigLoader::RejectUnknownKeys(
      root,
      {"schema_version", "enabled", "pass_optimize_lightbar_width", "normalize_max_brightness",
       "lightbar_min_mean_brightness", "padding_scale", "search_start_ratio", "search_end_ratio"},
      CONTEXT);
  ArmorCornerRefinerConfig config{
      .enabled = ConfigLoader::Require<bool>(root, "enabled", CONTEXT),
      .pass_optimize_lightbar_width =
          ConfigLoader::Require<int>(root, "pass_optimize_lightbar_width", CONTEXT),
      .normalize_max_brightness =
          ConfigLoader::Require<double>(root, "normalize_max_brightness", CONTEXT),
      .lightbar_min_mean_brightness =
          ConfigLoader::Require<double>(root, "lightbar_min_mean_brightness", CONTEXT),
      .padding_scale = ConfigLoader::Require<double>(root, "padding_scale", CONTEXT),
      .search_start_ratio = ConfigLoader::Require<double>(root, "search_start_ratio", CONTEXT),
      .search_end_ratio = ConfigLoader::Require<double>(root, "search_end_ratio", CONTEXT),
  };
  if (config.pass_optimize_lightbar_width < 0 || config.normalize_max_brightness <= 0.0 ||
      config.lightbar_min_mean_brightness < 0.0 || config.padding_scale < 0.0 ||
      config.search_start_ratio <= 0.0 || config.search_end_ratio <= config.search_start_ratio) {
    throw ConfigError("armor corner refiner config contains invalid values");
  }
  return config;
}

const char* CornerRefinementStatusName(CornerRefinementStatus status) noexcept {
  switch (status) {
    case CornerRefinementStatus::SUCCESS:
      return "success";
    case CornerRefinementStatus::DISABLED:
      return "disabled";
    case CornerRefinementStatus::INVALID_IMAGE:
      return "invalid_image";
    case CornerRefinementStatus::INVALID_GEOMETRY:
      return "invalid_geometry";
    case CornerRefinementStatus::LIGHT_TOO_NARROW:
      return "light_too_narrow";
    case CornerRefinementStatus::ROI_INVALID:
      return "roi_invalid";
    case CornerRefinementStatus::LIGHT_TOO_DARK:
      return "light_too_dark";
    case CornerRefinementStatus::MOMENTS_ZERO:
      return "moments_zero";
    case CornerRefinementStatus::PCA_DEGENERATE:
      return "pca_degenerate";
    case CornerRefinementStatus::TOP_NOT_FOUND:
      return "top_not_found";
    case CornerRefinementStatus::BOTTOM_NOT_FOUND:
      return "bottom_not_found";
  }
  return "unknown";
}

ArmorCornerRefiner::ArmorCornerRefiner(ArmorCornerRefinerConfig config)
    : config_(std::move(config)) {}

CornerRefinementResult ArmorCornerRefiner::Refine(const cv::Mat& gray_image,
                                                  std::span<const cv::Point2f, 4> corners) const {
  const auto start = Clock::now();
  CornerRefinementResult result;
  std::copy(corners.begin(), corners.end(), result.original_corners.begin());
  result.refined_corners = result.original_corners;
  for (std::size_t index = 0; index < 4; ++index) {
    result.endpoints[index].original = corners[index];
    result.endpoints[index].candidate = corners[index];
    result.endpoints[index].final = corners[index];
  }
  const auto finish = [&](CornerRefinementStatus status, int light_index = -1) {
    result.status = status;
    result.failure_light_index = light_index;
    result.success = status == CornerRefinementStatus::SUCCESS;
    result.fallback = !result.success;
    // 四个端点原子提交：任意阶段失败都撤销已经找到的另一侧候选。
    if (result.fallback)
      result.refined_corners = result.original_corners;
    for (std::size_t index = 0; index < 4; ++index) {
      result.endpoints[index].applied = result.success && result.endpoints[index].found;
      result.endpoints[index].final = result.refined_corners[index];
      result.corner_displacements[index] =
          result.refined_corners[index] - result.original_corners[index];
    }
    result.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    return result;
  };

  if (!config_.enabled)
    return finish(CornerRefinementStatus::DISABLED);
  if (gray_image.empty() || gray_image.type() != CV_8UC1)
    return finish(CornerRefinementStatus::INVALID_IMAGE);
  if (!std::all_of(corners.begin(), corners.end(), Finite))
    return finish(CornerRefinementStatus::INVALID_GEOMETRY);

  // 输入顺序为 TL、TR、BR、BL，因此左灯条连接 0-3，右灯条连接 1-2。
  std::array<Lightbar, 2> lights{MakeLightbar(corners[0], corners[3]),
                                 MakeLightbar(corners[1], corners[2])};
  std::array<CornerRefinementStatus, 2> light_status{CornerRefinementStatus::SUCCESS,
                                                     CornerRefinementStatus::SUCCESS};
  constexpr std::array<std::array<std::size_t, 2>, 2> ENDPOINT_INDICES{{{0, 3}, {1, 2}}};
  for (std::size_t light_index = 0; light_index < lights.size(); ++light_index) {
    auto& light = lights[light_index];
    auto& light_diagnostic = result.lightbars[light_index];
    light_diagnostic.center = light.center;
    light_diagnostic.axis = light.axis;
    light_diagnostic.top = light.top;
    light_diagnostic.bottom = light.bottom;
    light_diagnostic.length_px = light.length;
    light_diagnostic.width_px = light.width;
    if (light.length <= 1.0e-6) {
      light_status[light_index] = CornerRefinementStatus::INVALID_GEOMETRY;
      continue;
    }
    if (light.width <= config_.pass_optimize_lightbar_width) {
      light_status[light_index] = CornerRefinementStatus::LIGHT_TOO_NARROW;
      continue;
    }

    CornerRefinementStatus axis_failure = CornerRefinementStatus::PCA_DEGENERATE;
    const auto axis = FindSymmetryAxis(gray_image, light, config_, axis_failure);
    if (!axis) {
      light_status[light_index] = axis_failure;
      continue;
    }
    light.center = axis->centroid;
    light.axis = axis->direction;
    light_diagnostic.center = axis->centroid;
    light_diagnostic.axis = axis->direction;
    light_diagnostic.mean_brightness = axis->mean_brightness;
    light_diagnostic.axis_valid = true;

    auto& top_diagnostic = result.endpoints[ENDPOINT_INDICES[light_index][0]];
    auto& bottom_diagnostic = result.endpoints[ENDPOINT_INDICES[light_index][1]];
    const auto top = FindCorner(gray_image, light, *axis, 1, config_, top_diagnostic);
    if (!top) {
      light_status[light_index] = CornerRefinementStatus::TOP_NOT_FOUND;
      continue;
    }
    const auto bottom = FindCorner(gray_image, light, *axis, -1, config_, bottom_diagnostic);
    if (!bottom) {
      light_status[light_index] = CornerRefinementStatus::BOTTOM_NOT_FOUND;
      continue;
    }
    light_diagnostic.top = *top;
    light_diagnostic.bottom = *bottom;
    light_diagnostic.success = true;
    result.refined_corners[ENDPOINT_INDICES[light_index][0]] = *top;
    result.refined_corners[ENDPOINT_INDICES[light_index][1]] = *bottom;
  }
  for (std::size_t light_index = 0; light_index < light_status.size(); ++light_index) {
    if (light_status[light_index] != CornerRefinementStatus::SUCCESS)
      return finish(light_status[light_index], static_cast<int>(light_index));
  }
  return finish(CornerRefinementStatus::SUCCESS);
}

}  // namespace mv::modules
