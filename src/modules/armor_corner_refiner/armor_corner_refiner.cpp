#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <numbers>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace mv::modules {
namespace {

using Clock = std::chrono::steady_clock;

bool Finite(const cv::Point2f& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double Length(const cv::Point2f& value) {
  return std::hypot(static_cast<double>(value.x), static_cast<double>(value.y));
}

cv::Point2f Normalize(cv::Point2f value) {
  const double length = Length(value);
  return length > 1.0e-6 ? value * static_cast<float>(1.0 / length) : cv::Point2f{};
}

double Quantile(std::vector<double> values, double fraction) {
  if (values.empty())
    return 0.0;
  const auto index = static_cast<std::size_t>(std::clamp(fraction, 0.0, 1.0) *
                                              static_cast<double>(values.size() - 1));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index),
                   values.end());
  return values[index];
}

double PolygonArea(const std::array<cv::Point2f, 4>& points) {
  double area = 0.0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto& current = points[index];
    const auto& next = points[(index + 1) % points.size()];
    area += static_cast<double>(current.x) * next.y - static_cast<double>(current.y) * next.x;
  }
  return std::abs(area) * 0.5;
}

bool Convex(const std::array<cv::Point2f, 4>& points) {
  return cv::isContourConvex(std::vector<cv::Point2f>(points.begin(), points.end()));
}

struct StripRoi {
  cv::Rect bounds;
  cv::Point2f predicted_center;
  cv::Point2f predicted_axis;
  double predicted_length{0.0};
};

std::optional<StripRoi> MakeRoi(const cv::Size& image_size, cv::Point2f top, cv::Point2f bottom,
                                const ArmorCornerRefinerConfig& config) {
  const cv::Point2f axis = Normalize(bottom - top);
  const double length = Length(bottom - top);
  if (length < 2.0 || Length(axis) < 0.5)
    return std::nullopt;
  const cv::Point2f lateral(-axis.y, axis.x);
  const float axial = static_cast<float>(length * config.axial_expansion);
  const float half_width = static_cast<float>(length * config.lateral_expansion * 0.5);
  const cv::Point2f extended_top = top - axis * axial;
  const cv::Point2f extended_bottom = bottom + axis * axial;
  const std::array<cv::Point2f, 4> quad{
      extended_top - lateral * half_width, extended_top + lateral * half_width,
      extended_bottom + lateral * half_width, extended_bottom - lateral * half_width};
  const cv::Rect image_bounds(0, 0, image_size.width, image_size.height);
  const cv::Rect bounds =
      cv::boundingRect(std::vector<cv::Point2f>(quad.begin(), quad.end())) & image_bounds;
  if (bounds.area() < config.min_roi_area_px)
    return std::nullopt;
  return StripRoi{bounds, (top + bottom) * 0.5F, axis, length};
}

std::optional<RefinedLightStrip> ExtractStrip(const cv::Mat& image, const StripRoi& roi,
                                              ArmorColor color,
                                              const ArmorCornerRefinerConfig& config) {
  const cv::Mat patch = image(roi.bounds);
  std::vector<cv::Mat> channels;
  cv::split(patch, channels);
  cv::Mat color_difference;
  if (color == ArmorColor::BLUE) {
    cv::subtract(channels[0], channels[2], color_difference, cv::noArray(), CV_16S);
  } else {
    cv::subtract(channels[2], channels[0], color_difference, cv::noArray(), CV_16S);
  }
  cv::Mat brightness;
  cv::max(channels[0], channels[1], brightness);
  cv::max(brightness, channels[2], brightness);

  std::vector<double> differences;
  std::vector<double> brightnesses;
  differences.reserve(patch.total());
  brightnesses.reserve(patch.total());
  for (int row = 0; row < patch.rows; ++row) {
    const auto* difference_row = color_difference.ptr<std::int16_t>(row);
    const auto* brightness_row = brightness.ptr<std::uint8_t>(row);
    for (int column = 0; column < patch.cols; ++column) {
      differences.push_back(difference_row[column]);
      brightnesses.push_back(brightness_row[column]);
    }
  }
  const double color_threshold =
      std::max(config.min_color_difference, Quantile(differences, config.color_quantile));
  const double brightness_threshold =
      std::max(config.min_brightness, Quantile(brightnesses, config.brightness_quantile));
  cv::Mat mask(patch.size(), CV_8UC1, cv::Scalar(0));
  for (int row = 0; row < patch.rows; ++row) {
    const auto* difference_row = color_difference.ptr<std::int16_t>(row);
    const auto* brightness_row = brightness.ptr<std::uint8_t>(row);
    auto* mask_row = mask.ptr<std::uint8_t>(row);
    for (int column = 0; column < patch.cols; ++column) {
      if (difference_row[column] >= color_threshold &&
          brightness_row[column] >= brightness_threshold) {
        mask_row[column] = 255;
      }
    }
  }
  if (config.morphology_kernel > 1) {
    const int size = config.morphology_kernel | 1;
    const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {size, size});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  }

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
  int best_label = -1;
  double best_score = std::numeric_limits<double>::infinity();
  for (int label = 1; label < count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area < config.min_support_pixels)
      continue;
    const cv::Point2f center(static_cast<float>(centroids.at<double>(label, 0) + roi.bounds.x),
                             static_cast<float>(centroids.at<double>(label, 1) + roi.bounds.y));
    const double offset = Length(center - roi.predicted_center) / roi.predicted_length;
    if (offset > config.max_center_offset_ratio)
      continue;
    const double score = offset - 0.002 * static_cast<double>(area);
    if (score < best_score) {
      best_score = score;
      best_label = label;
    }
  }
  if (best_label < 0)
    return std::nullopt;

  std::vector<cv::Point2f> points;
  for (int row = 0; row < labels.rows; ++row) {
    const auto* label_row = labels.ptr<int>(row);
    for (int column = 0; column < labels.cols; ++column) {
      if (label_row[column] == best_label) {
        points.emplace_back(static_cast<float>(column + roi.bounds.x),
                            static_cast<float>(row + roi.bounds.y));
      }
    }
  }
  if (points.size() < static_cast<std::size_t>(config.min_support_pixels))
    return std::nullopt;
  cv::Mat data(static_cast<int>(points.size()), 2, CV_64F);
  for (std::size_t index = 0; index < points.size(); ++index) {
    data.at<double>(static_cast<int>(index), 0) = points[index].x;
    data.at<double>(static_cast<int>(index), 1) = points[index].y;
  }
  const cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
  const double major = pca.eigenvalues.at<double>(0);
  const double minor = pca.eigenvalues.at<double>(1);
  if (!(major > 1.0e-6 && major / std::max(minor, 1.0e-6) >= config.min_axis_ratio))
    return std::nullopt;
  cv::Point2f axis(static_cast<float>(pca.eigenvectors.at<double>(0, 0)),
                   static_cast<float>(pca.eigenvectors.at<double>(0, 1)));
  if (axis.dot(roi.predicted_axis) < 0.0F)
    axis = -axis;
  axis = Normalize(axis);
  const cv::Point2f center(static_cast<float>(pca.mean.at<double>(0, 0)),
                           static_cast<float>(pca.mean.at<double>(0, 1)));
  std::vector<double> projections;
  projections.reserve(points.size());
  for (const auto& point : points)
    projections.push_back((point - center).dot(axis));
  const double low = Quantile(projections, config.endpoint_low_quantile);
  const double high = Quantile(projections, config.endpoint_high_quantile);
  if (high - low < 1.0)
    return std::nullopt;
  const double band = std::max(1.0, (high - low) * config.endpoint_band_ratio);
  cv::Point2f top_sum{};
  cv::Point2f bottom_sum{};
  std::size_t top_count = 0;
  std::size_t bottom_count = 0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (projections[index] <= low + band) {
      top_sum += points[index];
      ++top_count;
    }
    if (projections[index] >= high - band) {
      bottom_sum += points[index];
      ++bottom_count;
    }
  }
  if (top_count == 0 || bottom_count == 0)
    return std::nullopt;
  const cv::Point2f top = top_sum * static_cast<float>(1.0 / top_count);
  const cv::Point2f bottom = bottom_sum * static_cast<float>(1.0 / bottom_count);
  const double axis_ratio = major / std::max(minor, 1.0e-6);
  const double confidence = std::clamp((axis_ratio / config.min_axis_ratio - 1.0) / 4.0, 0.0, 1.0);
  return RefinedLightStrip{center, axis, top, bottom, points.size(), confidence};
}

double AngleDegrees(cv::Point2f left, cv::Point2f right) {
  const double cosine =
      std::clamp(std::abs(static_cast<double>(Normalize(left).dot(Normalize(right)))), 0.0, 1.0);
  return std::acos(cosine) * 180.0 / std::numbers::pi;
}

CornerRefinementMode ParseMode(const std::string& value) {
  if (value == "raw")
    return CornerRefinementMode::RAW;
  if (value == "percentile_pca_shadow")
    return CornerRefinementMode::PERCENTILE_PCA_SHADOW;
  if (value == "gradient_axis_shadow")
    return CornerRefinementMode::GRADIENT_AXIS_SHADOW;
  throw ConfigError("armor corner refiner mode is invalid: " + value);
}

double Bilinear(const cv::Mat& gray, cv::Point2f point) {
  if (point.x < 0.0F || point.y < 0.0F || point.x >= gray.cols - 1.0F ||
      point.y >= gray.rows - 1.0F)
    return std::numeric_limits<double>::quiet_NaN();
  const int x = static_cast<int>(std::floor(point.x));
  const int y = static_cast<int>(std::floor(point.y));
  const double dx = point.x - x;
  const double dy = point.y - y;
  const double upper =
      gray.at<std::uint8_t>(y, x) * (1.0 - dx) + gray.at<std::uint8_t>(y, x + 1) * dx;
  const double lower =
      gray.at<std::uint8_t>(y + 1, x) * (1.0 - dx) + gray.at<std::uint8_t>(y + 1, x + 1) * dx;
  return upper * (1.0 - dy) + lower * dy;
}

double Median(std::vector<double> values) {
  if (values.empty())
    return 0.0;
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle),
                   values.end());
  const double high = values[middle];
  if (values.size() % 2 != 0)
    return high;
  return 0.5 * (high + *std::max_element(values.begin(), values.begin() + middle));
}

struct GradientAxis {
  RefinedLightStrip strip;
  double contrast{0.0};
  double background{0.0};
  double half_width{0.0};
  EndpointRefinementStatus failure{EndpointRefinementStatus::PCA_DEGENERATE};
  bool valid{false};
};

GradientAxis EstimateGradientAxis(const cv::Mat& gray, cv::Point2f predicted_top,
                                  cv::Point2f predicted_bottom,
                                  const ArmorCornerRefinerConfig& config) {
  GradientAxis result;
  const cv::Point2f predicted_center = (predicted_top + predicted_bottom) * 0.5F;
  const cv::Point2f predicted_axis = Normalize(predicted_bottom - predicted_top);
  const double predicted_length = Length(predicted_bottom - predicted_top);
  result.strip.center = predicted_center;
  result.strip.axis = predicted_axis;
  result.strip.top = predicted_top;
  result.strip.bottom = predicted_bottom;
  result.strip.estimated_length_px = predicted_length;
  if (predicted_length < config.gradient_min_light_length_px) {
    result.failure = EndpointRefinementStatus::LIGHT_TOO_SHORT;
    return result;
  }
  result.half_width = std::clamp(predicted_length * config.gradient_lateral_half_width_ratio,
                                 config.gradient_lateral_half_width_min_px,
                                 config.gradient_lateral_half_width_max_px);
  const cv::Point2f normal(-predicted_axis.y, predicted_axis.x);
  const double half_length = predicted_length * (0.5 + config.gradient_axial_expansion);
  const std::array<cv::Point2f, 4> quad{
      predicted_center - predicted_axis * static_cast<float>(half_length) -
          normal * static_cast<float>(result.half_width),
      predicted_center - predicted_axis * static_cast<float>(half_length) +
          normal * static_cast<float>(result.half_width),
      predicted_center + predicted_axis * static_cast<float>(half_length) +
          normal * static_cast<float>(result.half_width),
      predicted_center + predicted_axis * static_cast<float>(half_length) -
          normal * static_cast<float>(result.half_width)};
  const cv::Rect bounds = cv::boundingRect(std::vector<cv::Point2f>(quad.begin(), quad.end())) &
                          cv::Rect(0, 0, gray.cols, gray.rows);
  if (bounds.area() < config.min_roi_area_px) {
    result.failure = EndpointRefinementStatus::ROI_TOO_SMALL;
    return result;
  }

  std::vector<double> intensities;
  struct WeightedPixel {
    cv::Point2f point;
    double intensity;
  };
  std::vector<WeightedPixel> pixels;
  pixels.reserve(bounds.area());
  for (int y = bounds.y; y < bounds.y + bounds.height; ++y) {
    for (int x = bounds.x; x < bounds.x + bounds.width; ++x) {
      const cv::Point2f point(static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F);
      const cv::Point2f delta = point - predicted_center;
      if (std::abs(delta.dot(predicted_axis)) > half_length ||
          std::abs(delta.dot(normal)) > result.half_width)
        continue;
      const double intensity = gray.at<std::uint8_t>(y, x);
      intensities.push_back(intensity);
      pixels.push_back({point, intensity});
    }
  }
  if (pixels.size() < static_cast<std::size_t>(config.min_support_pixels)) {
    result.failure = EndpointRefinementStatus::ROI_TOO_SMALL;
    return result;
  }
  const double background = Quantile(intensities, 0.50);
  result.background = background;
  result.contrast = std::max(0.0, Quantile(intensities, 0.95) - background);
  result.strip.background_brightness = result.background;
  result.strip.contrast = result.contrast;
  double weight_sum = 0.0;
  cv::Point2d center{};
  for (const auto& pixel : pixels) {
    const double weight = std::max(0.0, pixel.intensity - background);
    weight_sum += weight;
    center += cv::Point2d(pixel.point.x, pixel.point.y) * weight;
  }
  if (weight_sum <= 1.0e-6) {
    result.failure = EndpointRefinementStatus::NO_BRIGHTNESS_SUPPORT;
    return result;
  }
  center *= 1.0 / weight_sum;
  double xx = 0.0, xy = 0.0, yy = 0.0;
  for (const auto& pixel : pixels) {
    const double weight = std::max(0.0, pixel.intensity - background);
    const double dx = pixel.point.x - center.x;
    const double dy = pixel.point.y - center.y;
    xx += weight * dx * dx;
    xy += weight * dx * dy;
    yy += weight * dy * dy;
  }
  cv::Mat covariance = (cv::Mat_<double>(2, 2) << xx / weight_sum, xy / weight_sum, xy / weight_sum,
                        yy / weight_sum);
  cv::Mat eigenvalues, eigenvectors;
  if (!cv::eigen(covariance, eigenvalues, eigenvectors)) {
    result.failure = EndpointRefinementStatus::PCA_DEGENERATE;
    return result;
  }
  const double major = eigenvalues.at<double>(0);
  const double minor = eigenvalues.at<double>(1);
  if (!(major > 1.0e-6 && minor >= 0.0)) {
    result.failure = EndpointRefinementStatus::PCA_DEGENERATE;
    return result;
  }
  cv::Point2f axis(static_cast<float>(eigenvectors.at<double>(0, 0)),
                   static_cast<float>(eigenvectors.at<double>(0, 1)));
  if (axis.dot(predicted_axis) < 0.0F)
    axis = -axis;
  axis = Normalize(axis);
  result.strip.center = cv::Point2f(static_cast<float>(center.x), static_cast<float>(center.y));
  result.strip.axis = axis;
  result.strip.estimated_length_px = std::sqrt(12.0 * major);
  result.strip.estimated_width_px = std::sqrt(12.0 * minor);
  result.strip.axis_ratio = major / std::max(minor, 1.0e-6);
  result.strip.axis_deviation_deg = AngleDegrees(axis, predicted_axis);
  result.strip.center_offset_px = Length(result.strip.center - predicted_center);
  result.strip.confidence =
      std::clamp(result.strip.axis_ratio / (4.0 * config.min_axis_ratio), 0.0, 1.0);
  if (result.strip.estimated_width_px <= config.gradient_min_estimated_width_px) {
    result.failure = EndpointRefinementStatus::LIGHT_TOO_NARROW;
    return result;
  }
  if (result.strip.axis_ratio < config.min_axis_ratio) {
    result.failure = EndpointRefinementStatus::AXIS_RATIO_TOO_LOW;
    return result;
  }
  if (result.strip.axis_deviation_deg > config.gradient_max_axis_angle_deg) {
    result.failure = EndpointRefinementStatus::AXIS_DEVIATION_TOO_LARGE;
    return result;
  }
  if (result.strip.center_offset_px > predicted_length * config.gradient_max_center_offset_ratio) {
    result.failure = EndpointRefinementStatus::CENTER_OFFSET_TOO_LARGE;
    return result;
  }
  result.valid = true;
  return result;
}

std::vector<double> GaussianSmooth(const std::vector<double>& input, double sigma_samples) {
  const int radius = std::max(1, static_cast<int>(std::ceil(3.0 * sigma_samples)));
  std::vector<double> kernel(static_cast<std::size_t>(2 * radius + 1));
  double sum = 0.0;
  for (int offset = -radius; offset <= radius; ++offset) {
    const double value = std::exp(-0.5 * offset * offset / (sigma_samples * sigma_samples));
    kernel[static_cast<std::size_t>(offset + radius)] = value;
    sum += value;
  }
  for (auto& value : kernel)
    value /= sum;
  std::vector<double> output(input.size());
  for (std::size_t index = 0; index < input.size(); ++index) {
    double value = 0.0;
    for (int offset = -radius; offset <= radius; ++offset) {
      const auto source = static_cast<std::size_t>(
          std::clamp(static_cast<int>(index) + offset, 0, static_cast<int>(input.size()) - 1));
      value += input[source] * kernel[static_cast<std::size_t>(offset + radius)];
    }
    output[index] = value;
  }
  return output;
}

struct ProfilePeak {
  bool valid{false};
  double sample_index{0.0};
  double strength{0.0};
  double secondary_strength{0.0};
  double inner_brightness{0.0};
};

ProfilePeak FindProfilePeak(const std::vector<double>& samples, double sigma_samples,
                            double sample_step_px) {
  ProfilePeak result;
  if (samples.size() < 5)
    return result;
  const auto smoothed = GaussianSmooth(samples, sigma_samples);
  const int gradient_radius = std::max(1, static_cast<int>(std::lround(0.5 / sample_step_px)));
  std::vector<double> gradients(smoothed.size(), 0.0);
  std::size_t peak = 0;
  for (std::size_t index = static_cast<std::size_t>(gradient_radius);
       index + static_cast<std::size_t>(gradient_radius) < smoothed.size(); ++index) {
    gradients[index] = smoothed[index - gradient_radius] - smoothed[index + gradient_radius];
    if (gradients[index] > result.strength) {
      result.strength = gradients[index];
      peak = index;
    }
  }
  if (peak == 0 || peak + 1 >= gradients.size())
    return result;
  const int exclusion_radius =
      std::max(gradient_radius, static_cast<int>(std::ceil(1.0 / sample_step_px)));
  for (std::size_t index = static_cast<std::size_t>(gradient_radius);
       index + static_cast<std::size_t>(gradient_radius) < gradients.size(); ++index) {
    if (std::abs(static_cast<int>(index) - static_cast<int>(peak)) <= exclusion_radius)
      continue;
    result.secondary_strength = std::max(result.secondary_strength, gradients[index]);
  }
  double subpixel = 0.0;
  const double denominator = gradients[peak - 1] - 2.0 * gradients[peak] + gradients[peak + 1];
  if (std::abs(denominator) > 1.0e-9) {
    subpixel =
        std::clamp(0.5 * (gradients[peak - 1] - gradients[peak + 1]) / denominator, -0.5, 0.5);
  }
  result.valid = true;
  result.sample_index = static_cast<double>(peak) + subpixel;
  result.inner_brightness = smoothed[peak - static_cast<std::size_t>(gradient_radius)];
  return result;
}

}  // namespace

const char* CornerRefinementModeName(CornerRefinementMode mode) noexcept {
  switch (mode) {
    case CornerRefinementMode::RAW:
      return "raw";
    case CornerRefinementMode::PERCENTILE_PCA_SHADOW:
      return "percentile_pca_shadow";
    case CornerRefinementMode::GRADIENT_AXIS_SHADOW:
      return "gradient_axis_shadow";
  }
  return "unknown";
}

ArmorCornerRefinerConfig ParseArmorCornerRefinerConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "armor corner refiner config";
  ConfigLoader::RejectUnknownKeys(root,
                                  {"schema_version",
                                   "mode",
                                   "axial_expansion",
                                   "lateral_expansion",
                                   "min_roi_area_px",
                                   "min_support_pixels",
                                   "color_quantile",
                                   "brightness_quantile",
                                   "min_color_difference",
                                   "min_brightness",
                                   "morphology_kernel",
                                   "min_axis_ratio",
                                   "max_center_offset_ratio",
                                   "max_parallel_angle_deg",
                                   "max_length_ratio",
                                   "endpoint_low_quantile",
                                   "endpoint_high_quantile",
                                   "endpoint_band_ratio",
                                   "max_corner_move_height_ratio",
                                   "min_polygon_area_px",
                                   "gradient_axial_expansion",
                                   "gradient_lateral_half_width_ratio",
                                   "gradient_lateral_half_width_min_px",
                                   "gradient_lateral_half_width_max_px",
                                   "gradient_min_light_length_px",
                                   "gradient_short_light_threshold_px",
                                   "gradient_min_estimated_width_px",
                                   "gradient_max_axis_angle_deg",
                                   "gradient_max_center_offset_ratio",
                                   "gradient_search_start_ratio",
                                   "gradient_search_end_ratio",
                                   "gradient_short_scan_lines",
                                   "gradient_long_scan_lines",
                                   "gradient_sample_step_px",
                                   "gradient_smoothing_sigma_px",
                                   "gradient_min_strength_gray",
                                   "gradient_min_contrast_ratio",
                                   "gradient_min_inner_brightness_ratio",
                                   "gradient_max_secondary_peak_ratio",
                                   "gradient_max_profile_peak_spread_px",
                                   "gradient_max_corner_move_px"},
                                  CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 4)
    throw ConfigError("armor corner refiner config schema_version must be 4");
  ArmorCornerRefinerConfig config{
      .mode = ParseMode(ConfigLoader::Require<std::string>(root, "mode", CONTEXT)),
      .axial_expansion = ConfigLoader::Require<double>(root, "axial_expansion", CONTEXT),
      .lateral_expansion = ConfigLoader::Require<double>(root, "lateral_expansion", CONTEXT),
      .min_roi_area_px = ConfigLoader::Require<int>(root, "min_roi_area_px", CONTEXT),
      .min_support_pixels = ConfigLoader::Require<int>(root, "min_support_pixels", CONTEXT),
      .color_quantile = ConfigLoader::Require<double>(root, "color_quantile", CONTEXT),
      .brightness_quantile = ConfigLoader::Require<double>(root, "brightness_quantile", CONTEXT),
      .min_color_difference = ConfigLoader::Require<double>(root, "min_color_difference", CONTEXT),
      .min_brightness = ConfigLoader::Require<double>(root, "min_brightness", CONTEXT),
      .morphology_kernel = ConfigLoader::Require<int>(root, "morphology_kernel", CONTEXT),
      .min_axis_ratio = ConfigLoader::Require<double>(root, "min_axis_ratio", CONTEXT),
      .max_center_offset_ratio =
          ConfigLoader::Require<double>(root, "max_center_offset_ratio", CONTEXT),
      .max_parallel_angle_deg =
          ConfigLoader::Require<double>(root, "max_parallel_angle_deg", CONTEXT),
      .max_length_ratio = ConfigLoader::Require<double>(root, "max_length_ratio", CONTEXT),
      .endpoint_low_quantile =
          ConfigLoader::Require<double>(root, "endpoint_low_quantile", CONTEXT),
      .endpoint_high_quantile =
          ConfigLoader::Require<double>(root, "endpoint_high_quantile", CONTEXT),
      .endpoint_band_ratio = ConfigLoader::Require<double>(root, "endpoint_band_ratio", CONTEXT),
      .max_corner_move_height_ratio =
          ConfigLoader::Require<double>(root, "max_corner_move_height_ratio", CONTEXT),
      .min_polygon_area_px = ConfigLoader::Require<double>(root, "min_polygon_area_px", CONTEXT),
      .gradient_axial_expansion =
          ConfigLoader::Require<double>(root, "gradient_axial_expansion", CONTEXT),
      .gradient_lateral_half_width_ratio =
          ConfigLoader::Require<double>(root, "gradient_lateral_half_width_ratio", CONTEXT),
      .gradient_lateral_half_width_min_px =
          ConfigLoader::Require<double>(root, "gradient_lateral_half_width_min_px", CONTEXT),
      .gradient_lateral_half_width_max_px =
          ConfigLoader::Require<double>(root, "gradient_lateral_half_width_max_px", CONTEXT),
      .gradient_min_light_length_px =
          ConfigLoader::Require<double>(root, "gradient_min_light_length_px", CONTEXT),
      .gradient_short_light_threshold_px =
          ConfigLoader::Require<double>(root, "gradient_short_light_threshold_px", CONTEXT),
      .gradient_min_estimated_width_px =
          ConfigLoader::Require<double>(root, "gradient_min_estimated_width_px", CONTEXT),
      .gradient_max_axis_angle_deg =
          ConfigLoader::Require<double>(root, "gradient_max_axis_angle_deg", CONTEXT),
      .gradient_max_center_offset_ratio =
          ConfigLoader::Require<double>(root, "gradient_max_center_offset_ratio", CONTEXT),
      .gradient_search_start_ratio =
          ConfigLoader::Require<double>(root, "gradient_search_start_ratio", CONTEXT),
      .gradient_search_end_ratio =
          ConfigLoader::Require<double>(root, "gradient_search_end_ratio", CONTEXT),
      .gradient_short_scan_lines =
          ConfigLoader::Require<int>(root, "gradient_short_scan_lines", CONTEXT),
      .gradient_long_scan_lines =
          ConfigLoader::Require<int>(root, "gradient_long_scan_lines", CONTEXT),
      .gradient_sample_step_px =
          ConfigLoader::Require<double>(root, "gradient_sample_step_px", CONTEXT),
      .gradient_smoothing_sigma_px =
          ConfigLoader::Require<double>(root, "gradient_smoothing_sigma_px", CONTEXT),
      .gradient_min_strength_gray =
          ConfigLoader::Require<double>(root, "gradient_min_strength_gray", CONTEXT),
      .gradient_min_contrast_ratio =
          ConfigLoader::Require<double>(root, "gradient_min_contrast_ratio", CONTEXT),
      .gradient_min_inner_brightness_ratio =
          ConfigLoader::Require<double>(root, "gradient_min_inner_brightness_ratio", CONTEXT),
      .gradient_max_secondary_peak_ratio =
          ConfigLoader::Require<double>(root, "gradient_max_secondary_peak_ratio", CONTEXT),
      .gradient_max_profile_peak_spread_px =
          ConfigLoader::Require<double>(root, "gradient_max_profile_peak_spread_px", CONTEXT),
      .gradient_max_corner_move_px =
          ConfigLoader::Require<double>(root, "gradient_max_corner_move_px", CONTEXT)};
  if (!(config.axial_expansion >= 0.0 && config.lateral_expansion > 0.0 &&
        config.min_roi_area_px > 0 && config.min_support_pixels >= 3 &&
        config.color_quantile >= 0.0 && config.color_quantile <= 1.0 &&
        config.brightness_quantile >= 0.0 && config.brightness_quantile <= 1.0 &&
        config.min_axis_ratio > 1.0 && config.max_center_offset_ratio > 0.0 &&
        config.max_parallel_angle_deg > 0.0 && config.max_parallel_angle_deg < 90.0 &&
        config.max_length_ratio >= 1.0 && config.endpoint_low_quantile >= 0.0 &&
        config.endpoint_low_quantile < config.endpoint_high_quantile &&
        config.endpoint_high_quantile <= 1.0 && config.endpoint_band_ratio > 0.0 &&
        config.endpoint_band_ratio < 0.5 && config.max_corner_move_height_ratio > 0.0 &&
        config.min_polygon_area_px > 0.0 && config.gradient_axial_expansion >= 0.0 &&
        config.gradient_axial_expansion <= 0.5 && config.gradient_lateral_half_width_ratio > 0.0 &&
        config.gradient_lateral_half_width_min_px > 0.0 &&
        config.gradient_lateral_half_width_max_px >= config.gradient_lateral_half_width_min_px &&
        config.gradient_min_light_length_px > 0.0 &&
        config.gradient_short_light_threshold_px > config.gradient_min_light_length_px &&
        config.gradient_min_estimated_width_px > 0.0 && config.gradient_max_axis_angle_deg > 0.0 &&
        config.gradient_max_axis_angle_deg < 90.0 &&
        config.gradient_max_center_offset_ratio > 0.0 && config.gradient_search_start_ratio > 0.0 &&
        config.gradient_search_end_ratio > config.gradient_search_start_ratio &&
        config.gradient_search_end_ratio <= 1.0 && config.gradient_short_scan_lines == 3 &&
        config.gradient_long_scan_lines == 5 && config.gradient_sample_step_px > 0.0 &&
        config.gradient_sample_step_px <= 2.0 && config.gradient_smoothing_sigma_px > 0.0 &&
        config.gradient_smoothing_sigma_px <= 5.0 && config.gradient_min_strength_gray > 0.0 &&
        config.gradient_min_contrast_ratio > 0.0 && config.gradient_min_contrast_ratio <= 1.0 &&
        config.gradient_min_inner_brightness_ratio >= 0.0 &&
        config.gradient_min_inner_brightness_ratio <= 1.0 &&
        config.gradient_max_secondary_peak_ratio > 0.0 &&
        config.gradient_max_secondary_peak_ratio < 1.0 &&
        config.gradient_max_profile_peak_spread_px > 0.0 &&
        config.gradient_max_corner_move_px > 0.0)) {
    throw ConfigError("armor corner refiner thresholds are invalid");
  }
  return config;
}

const char* CornerRefinementStatusName(CornerRefinementStatus status) noexcept {
  switch (status) {
    case CornerRefinementStatus::SUCCESS:
      return "success";
    case CornerRefinementStatus::INVALID_IMAGE:
      return "invalid_image";
    case CornerRefinementStatus::INVALID_GEOMETRY:
      return "invalid_geometry";
    case CornerRefinementStatus::ROI_TOO_SMALL:
      return "roi_too_small";
    case CornerRefinementStatus::LEFT_STRIP_NOT_FOUND:
      return "left_strip_not_found";
    case CornerRefinementStatus::RIGHT_STRIP_NOT_FOUND:
      return "right_strip_not_found";
    case CornerRefinementStatus::PCA_DEGENERATE:
      return "pca_degenerate";
    case CornerRefinementStatus::STRIP_PAIR_INVALID:
      return "strip_pair_invalid";
    case CornerRefinementStatus::CORNER_MOVE_TOO_LARGE:
      return "corner_move_too_large";
    case CornerRefinementStatus::PNP_FALLBACK:
      return "pnp_fallback";
    case CornerRefinementStatus::MODE_RAW:
      return "mode_raw";
    case CornerRefinementStatus::INCOMPLETE_ENDPOINTS:
      return "incomplete_endpoints";
  }
  return "unknown";
}

const char* EndpointRefinementStatusName(EndpointRefinementStatus status) noexcept {
  switch (status) {
    case EndpointRefinementStatus::APPLIED:
      return "applied";
    case EndpointRefinementStatus::MODE_RAW:
      return "mode_raw";
    case EndpointRefinementStatus::LIGHT_TOO_SHORT:
      return "light_too_short";
    case EndpointRefinementStatus::ROI_TOO_SMALL:
      return "roi_too_small";
    case EndpointRefinementStatus::NO_BRIGHTNESS_SUPPORT:
      return "no_brightness_support";
    case EndpointRefinementStatus::PCA_DEGENERATE:
      return "pca_degenerate";
    case EndpointRefinementStatus::LIGHT_TOO_NARROW:
      return "light_too_narrow";
    case EndpointRefinementStatus::AXIS_RATIO_TOO_LOW:
      return "axis_ratio_too_low";
    case EndpointRefinementStatus::AXIS_DEVIATION_TOO_LARGE:
      return "axis_deviation_too_large";
    case EndpointRefinementStatus::CENTER_OFFSET_TOO_LARGE:
      return "center_offset_too_large";
    case EndpointRefinementStatus::GRADIENT_TOO_WEAK:
      return "gradient_too_weak";
    case EndpointRefinementStatus::BRIGHT_SIDE_TOO_DARK:
      return "bright_side_too_dark";
    case EndpointRefinementStatus::INSUFFICIENT_SCAN_LINES:
      return "insufficient_scan_lines";
    case EndpointRefinementStatus::PEAK_AMBIGUOUS:
      return "peak_ambiguous";
    case EndpointRefinementStatus::PROFILE_UNSTABLE:
      return "profile_unstable";
    case EndpointRefinementStatus::MOVE_TOO_LARGE:
      return "move_too_large";
  }
  return "unknown";
}

const char* EndpointRevertedByName(EndpointRevertedBy reason) noexcept {
  switch (reason) {
    case EndpointRevertedBy::NONE:
      return "none";
    case EndpointRevertedBy::ARMOR_ATOMIC:
      return "armor_atomic";
    case EndpointRevertedBy::GEOMETRY:
      return "geometry";
    case EndpointRevertedBy::PNP:
      return "pnp";
  }
  return "unknown";
}

ArmorCornerRefiner::ArmorCornerRefiner(ArmorCornerRefinerConfig config)
    : config_(std::move(config)) {}

CornerRefinementResult ArmorCornerRefiner::Refine(
    const cv::Mat& bgr_image, std::span<const cv::Point2f, 4> corners, ArmorColor enemy_color,
    hal::CameraFrame::ArmorType /*armor_type*/) const {
  if (config_.mode == CornerRefinementMode::PERCENTILE_PCA_SHADOW)
    return RefinePercentile(bgr_image, corners, enemy_color);
  if (config_.mode == CornerRefinementMode::GRADIENT_AXIS_SHADOW)
    return RefineGradient(bgr_image, corners);

  CornerRefinementResult result;
  result.mode = CornerRefinementMode::RAW;
  std::copy(corners.begin(), corners.end(), result.original_corners.begin());
  result.refined_corners = result.original_corners;
  result.status = CornerRefinementStatus::MODE_RAW;
  for (std::size_t index = 0; index < result.endpoints.size(); ++index) {
    result.endpoints[index].original = result.original_corners[index];
    result.endpoints[index].candidate = result.original_corners[index];
    result.endpoints[index].final = result.original_corners[index];
  }
  return result;
}

CornerRefinementResult ArmorCornerRefiner::RefinePercentile(const cv::Mat& bgr_image,
                                                            std::span<const cv::Point2f, 4> corners,
                                                            ArmorColor enemy_color) const {
  const auto start = Clock::now();
  CornerRefinementResult result;
  result.mode = CornerRefinementMode::PERCENTILE_PCA_SHADOW;
  std::copy(corners.begin(), corners.end(), result.original_corners.begin());
  result.refined_corners = result.original_corners;
  const auto finish = [&](CornerRefinementStatus status) {
    result.status = status;
    result.success = status == CornerRefinementStatus::SUCCESS;
    result.fallback = !result.success;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    if (!result.success)
      result.refined_corners = result.original_corners;
    for (std::size_t index = 0; index < 4; ++index)
      result.corner_displacements[index] =
          result.refined_corners[index] - result.original_corners[index];
    return result;
  };
  if (bgr_image.empty() || bgr_image.type() != CV_8UC3)
    return finish(CornerRefinementStatus::INVALID_IMAGE);
  if (!std::all_of(corners.begin(), corners.end(), Finite) ||
      PolygonArea(result.original_corners) < config_.min_polygon_area_px ||
      !Convex(result.original_corners))
    return finish(CornerRefinementStatus::INVALID_GEOMETRY);

  const auto left_roi = MakeRoi(bgr_image.size(), corners[0], corners[3], config_);
  const auto right_roi = MakeRoi(bgr_image.size(), corners[1], corners[2], config_);
  if (!left_roi || !right_roi)
    return finish(CornerRefinementStatus::ROI_TOO_SMALL);
  const auto left = ExtractStrip(bgr_image, *left_roi, enemy_color, config_);
  if (!left)
    return finish(CornerRefinementStatus::LEFT_STRIP_NOT_FOUND);
  const auto right = ExtractStrip(bgr_image, *right_roi, enemy_color, config_);
  if (!right)
    return finish(CornerRefinementStatus::RIGHT_STRIP_NOT_FOUND);
  result.strips = {*left, *right};

  const double left_length = Length(left->bottom - left->top);
  const double right_length = Length(right->bottom - right->top);
  const double ratio =
      std::max(left_length, right_length) / std::max(1.0, std::min(left_length, right_length));
  const double own_identity_cost = Length(left->center - left_roi->predicted_center) +
                                   Length(right->center - right_roi->predicted_center);
  const double swapped_identity_cost = Length(left->center - right_roi->predicted_center) +
                                       Length(right->center - left_roi->predicted_center);
  if (AngleDegrees(left->axis, right->axis) > config_.max_parallel_angle_deg ||
      ratio > config_.max_length_ratio || own_identity_cost >= swapped_identity_cost)
    return finish(CornerRefinementStatus::STRIP_PAIR_INVALID);

  const auto assign = [](const RefinedLightStrip& strip, cv::Point2f predicted_top) {
    return cv::norm(strip.top - predicted_top) <= cv::norm(strip.bottom - predicted_top)
               ? std::pair(strip.top, strip.bottom)
               : std::pair(strip.bottom, strip.top);
  };
  const auto left_endpoints = assign(*left, corners[0]);
  const auto right_endpoints = assign(*right, corners[1]);
  result.refined_corners = {left_endpoints.first, right_endpoints.first, right_endpoints.second,
                            left_endpoints.second};
  if (!Convex(result.refined_corners) ||
      PolygonArea(result.refined_corners) < config_.min_polygon_area_px)
    return finish(CornerRefinementStatus::INVALID_GEOMETRY);
  const double detected_height =
      0.5 * (Length(corners[3] - corners[0]) + Length(corners[2] - corners[1]));
  const double max_move = detected_height * config_.max_corner_move_height_ratio;
  for (std::size_t index = 0; index < 4; ++index) {
    if (Length(result.refined_corners[index] - corners[index]) > max_move)
      return finish(CornerRefinementStatus::CORNER_MOVE_TOO_LARGE);
  }
  result.confidence = std::min(left->confidence, right->confidence);
  return finish(CornerRefinementStatus::SUCCESS);
}

CornerRefinementResult ArmorCornerRefiner::RefineGradient(
    const cv::Mat& bgr_image, std::span<const cv::Point2f, 4> corners) const {
  const auto start = Clock::now();
  CornerRefinementResult result;
  result.mode = CornerRefinementMode::GRADIENT_AXIS_SHADOW;
  std::copy(corners.begin(), corners.end(), result.original_corners.begin());
  result.refined_corners = result.original_corners;
  for (std::size_t index = 0; index < result.endpoints.size(); ++index) {
    result.endpoints[index].original = result.original_corners[index];
    result.endpoints[index].candidate = result.original_corners[index];
    result.endpoints[index].final = result.original_corners[index];
  }
  const auto finish = [&](CornerRefinementStatus status) {
    result.status = status;
    result.success = status == CornerRefinementStatus::SUCCESS;
    result.fallback = !result.success;
    if (result.fallback)
      result.refined_corners = result.original_corners;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    for (std::size_t index = 0; index < result.endpoints.size(); ++index) {
      result.endpoints[index].final = result.refined_corners[index];
      result.endpoints[index].movement_px =
          Length(result.refined_corners[index] - result.original_corners[index]);
      result.corner_displacements[index] =
          result.refined_corners[index] - result.original_corners[index];
    }
    return result;
  };
  if (bgr_image.empty() || bgr_image.type() != CV_8UC3)
    return finish(CornerRefinementStatus::INVALID_IMAGE);
  if (!std::all_of(corners.begin(), corners.end(), Finite) ||
      PolygonArea(result.original_corners) < config_.min_polygon_area_px ||
      !Convex(result.original_corners))
    return finish(CornerRefinementStatus::INVALID_GEOMETRY);

  cv::Mat gray;
  cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
  std::array<GradientAxis, 2> axes{EstimateGradientAxis(gray, corners[0], corners[3], config_),
                                   EstimateGradientAxis(gray, corners[1], corners[2], config_)};
  result.strips = {axes[0].strip, axes[1].strip};

  const auto refine_endpoint = [&](std::size_t corner_index, std::size_t strip_index, int sign) {
    auto& diagnostic = result.endpoints[corner_index];
    const auto& analysis = axes[strip_index];
    if (!analysis.valid) {
      diagnostic.status = analysis.failure;
      return;
    }
    const auto& strip = analysis.strip;
    const cv::Point2f direction = strip.axis * static_cast<float>(sign);
    const cv::Point2f normal(-strip.axis.y, strip.axis.x);
    const double predicted_length =
        Length(strip_index == 0 ? corners[3] - corners[0] : corners[2] - corners[1]);
    const double start_distance = predicted_length * config_.gradient_search_start_ratio;
    const double end_distance = predicted_length * config_.gradient_search_end_ratio;
    diagnostic.search_start = strip.center + direction * static_cast<float>(start_distance);
    diagnostic.search_end = strip.center + direction * static_cast<float>(end_distance);
    const std::size_t sample_count =
        static_cast<std::size_t>(
            std::floor((end_distance - start_distance) / config_.gradient_sample_step_px)) +
        1;
    if (sample_count < 5) {
      diagnostic.status = EndpointRefinementStatus::ROI_TOO_SMALL;
      return;
    }
    const double scan_span =
        std::min(analysis.half_width * 0.7, std::max(0.5, strip.estimated_width_px * 0.35));
    const double threshold = std::max(config_.gradient_min_strength_gray,
                                      analysis.contrast * config_.gradient_min_contrast_ratio);
    diagnostic.bright_side_threshold =
        analysis.background + analysis.contrast * config_.gradient_min_inner_brightness_ratio;
    const int scan_line_count = predicted_length < config_.gradient_short_light_threshold_px
                                    ? config_.gradient_short_scan_lines
                                    : config_.gradient_long_scan_lines;
    const std::size_t minimum_scan_lines = static_cast<std::size_t>((scan_line_count + 1) / 2);
    std::vector<std::vector<double>> scan_profiles;
    scan_profiles.reserve(static_cast<std::size_t>(scan_line_count));
    for (int scan = 0; scan < scan_line_count; ++scan) {
      const double lateral =
          scan_line_count == 1 ? 0.0 : -scan_span + 2.0 * scan_span * scan / (scan_line_count - 1);
      std::vector<double> samples;
      samples.reserve(sample_count);
      bool inside = true;
      for (std::size_t index = 0; index < sample_count; ++index) {
        const double distance = start_distance + index * config_.gradient_sample_step_px;
        const cv::Point2f point = strip.center + direction * static_cast<float>(distance) +
                                  normal * static_cast<float>(lateral);
        const double value = Bilinear(gray, point);
        if (!std::isfinite(value)) {
          inside = false;
          break;
        }
        samples.push_back(value);
      }
      if (!inside)
        continue;
      const auto line_peak = FindProfilePeak(
          samples, config_.gradient_smoothing_sigma_px / config_.gradient_sample_step_px,
          config_.gradient_sample_step_px);
      const std::size_t diagnostic_index = scan_profiles.size();
      if (line_peak.valid && diagnostic_index < diagnostic.scan_candidates.size()) {
        const double distance =
            start_distance + line_peak.sample_index * config_.gradient_sample_step_px;
        diagnostic.scan_candidates[diagnostic_index] = strip.center +
                                                       direction * static_cast<float>(distance) +
                                                       normal * static_cast<float>(lateral);
        diagnostic.scan_candidate_present[diagnostic_index] = true;
        diagnostic.scan_candidate_valid[diagnostic_index] =
            line_peak.strength >= threshold &&
            line_peak.inner_brightness >= diagnostic.bright_side_threshold;
      }
      scan_profiles.push_back(std::move(samples));
    }
    diagnostic.valid_scan_lines = scan_profiles.size();
    if (scan_profiles.size() < minimum_scan_lines) {
      diagnostic.status = EndpointRefinementStatus::INSUFFICIENT_SCAN_LINES;
      return;
    }

    const auto fuse_profiles = [&](std::optional<std::size_t> excluded) {
      std::vector<double> fused(sample_count, 0.0);
      for (std::size_t sample = 0; sample < sample_count; ++sample) {
        std::vector<double> values;
        values.reserve(scan_profiles.size());
        for (std::size_t line = 0; line < scan_profiles.size(); ++line) {
          if (!excluded || line != *excluded)
            values.push_back(scan_profiles[line][sample]);
        }
        fused[sample] = Median(std::move(values));
      }
      return fused;
    };
    const auto peak =
        FindProfilePeak(fuse_profiles(std::nullopt),
                        config_.gradient_smoothing_sigma_px / config_.gradient_sample_step_px,
                        config_.gradient_sample_step_px);
    if (!peak.valid) {
      diagnostic.status = EndpointRefinementStatus::GRADIENT_TOO_WEAK;
      return;
    }
    diagnostic.gradient_strength = peak.strength;
    diagnostic.secondary_gradient_strength = peak.secondary_strength;
    diagnostic.secondary_peak_ratio =
        peak.strength > 1.0e-9 ? peak.secondary_strength / peak.strength : 0.0;
    diagnostic.inner_brightness = peak.inner_brightness;
    const double axial = start_distance + peak.sample_index * config_.gradient_sample_step_px;
    diagnostic.candidate = strip.center + direction * static_cast<float>(axial);
    diagnostic.requested_movement_px = Length(diagnostic.candidate - diagnostic.original);
    if (peak.strength < threshold) {
      diagnostic.status = EndpointRefinementStatus::GRADIENT_TOO_WEAK;
      return;
    }
    if (peak.inner_brightness < diagnostic.bright_side_threshold) {
      diagnostic.status = EndpointRefinementStatus::BRIGHT_SIDE_TOO_DARK;
      return;
    }
    if (diagnostic.secondary_peak_ratio > config_.gradient_max_secondary_peak_ratio) {
      diagnostic.status = EndpointRefinementStatus::PEAK_AMBIGUOUS;
      return;
    }

    std::vector<double> bootstrap_peaks;
    bootstrap_peaks.reserve(scan_profiles.size());
    for (std::size_t excluded = 0; excluded < scan_profiles.size(); ++excluded) {
      const auto bootstrap =
          FindProfilePeak(fuse_profiles(excluded),
                          config_.gradient_smoothing_sigma_px / config_.gradient_sample_step_px,
                          config_.gradient_sample_step_px);
      if (bootstrap.valid) {
        bootstrap_peaks.push_back(start_distance +
                                  bootstrap.sample_index * config_.gradient_sample_step_px);
      }
    }
    if (bootstrap_peaks.size() < minimum_scan_lines) {
      diagnostic.status = EndpointRefinementStatus::INSUFFICIENT_SCAN_LINES;
      return;
    }
    const double bootstrap_center = Median(bootstrap_peaks);
    std::vector<double> deviations;
    deviations.reserve(bootstrap_peaks.size());
    for (const double value : bootstrap_peaks)
      deviations.push_back(std::abs(value - bootstrap_center));
    diagnostic.profile_peak_spread_px = Median(std::move(deviations));
    if (diagnostic.profile_peak_spread_px > config_.gradient_max_profile_peak_spread_px) {
      diagnostic.status = EndpointRefinementStatus::PROFILE_UNSTABLE;
      return;
    }
    if (diagnostic.requested_movement_px > config_.gradient_max_corner_move_px) {
      diagnostic.status = EndpointRefinementStatus::MOVE_TOO_LARGE;
      return;
    }
    diagnostic.candidate_valid = true;
    diagnostic.applied = true;
    diagnostic.fallback = false;
    diagnostic.status = EndpointRefinementStatus::APPLIED;
    result.refined_corners[corner_index] = diagnostic.candidate;
  };

  refine_endpoint(0, 0, -1);
  refine_endpoint(3, 0, 1);
  refine_endpoint(1, 1, -1);
  refine_endpoint(2, 1, 1);
  const std::size_t applied = static_cast<std::size_t>(
      std::count_if(result.endpoints.begin(), result.endpoints.end(),
                    [](const EndpointRefinementDiagnostic& endpoint) { return endpoint.applied; }));
  if (applied != result.endpoints.size()) {
    result.refined_corners = result.original_corners;
    for (auto& endpoint : result.endpoints) {
      if (!endpoint.applied)
        continue;
      endpoint.applied = false;
      endpoint.fallback = true;
      endpoint.reverted_by = EndpointRevertedBy::ARMOR_ATOMIC;
    }
    return finish(CornerRefinementStatus::INCOMPLETE_ENDPOINTS);
  }
  if (!Convex(result.refined_corners) ||
      PolygonArea(result.refined_corners) < config_.min_polygon_area_px) {
    result.refined_corners = result.original_corners;
    for (auto& endpoint : result.endpoints) {
      if (endpoint.applied) {
        endpoint.applied = false;
        endpoint.fallback = true;
        endpoint.reverted_by = EndpointRevertedBy::GEOMETRY;
      }
    }
    return finish(CornerRefinementStatus::INVALID_GEOMETRY);
  }
  result.strips[0].top = result.refined_corners[0];
  result.strips[0].bottom = result.refined_corners[3];
  result.strips[1].top = result.refined_corners[1];
  result.strips[1].bottom = result.refined_corners[2];
  result.confidence = std::min(result.strips[0].confidence, result.strips[1].confidence);
  return finish(CornerRefinementStatus::SUCCESS);
}

}  // namespace mv::modules
