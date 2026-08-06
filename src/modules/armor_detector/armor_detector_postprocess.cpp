#include "modules/armor_detector/armor_detector_postprocess.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>

namespace mv::modules {

const char* ArmorColorName(ArmorColor color) noexcept {
  switch (color) {
    case ArmorColor::RED:
      return "red";
    case ArmorColor::BLUE:
      return "blue";
  }
  return "unknown";
}

const char* ArmorLabelName(ArmorLabel label) noexcept {
  switch (label) {
    case ArmorLabel::SENTRY:
      return "sentry";
    case ArmorLabel::ONE:
      return "one";
    case ArmorLabel::TWO:
      return "two";
    case ArmorLabel::THREE:
      return "three";
    case ArmorLabel::FOUR:
      return "four";
    case ArmorLabel::FIVE:
      return "five";
    case ArmorLabel::OUTPOST:
      return "outpost";
    case ArmorLabel::BASE_SMALL:
      return "base_small";
    case ArmorLabel::BASE_BIG:
      return "base_big";
  }
  return "unknown";
}

}  // namespace mv::modules

namespace mv::modules::detail {
namespace {

constexpr float K_MIN_GEOMETRY_SIZE = 1.0e-3F;
constexpr float K_MIN_POLYGON_AREA = 1.0e-3F;

// 模型类别通道与 ArmorLabel 枚举同序，转换前先保护数组契约。
ArmorLabel ToArmorLabel(int id) {
  if (id < 0 || id > 8) {
    throw std::out_of_range("armor label id is outside [0, 8]");
  }
  return static_cast<ArmorLabel>(id);
}

// 0526 权重的实际颜色通道依次为 blue、red、gray、purple。
std::optional<ArmorColor> ToArmorColor(int id) noexcept {
  switch (id) {
    case 0:
      return ArmorColor::BLUE;
    case 1:
      return ArmorColor::RED;
    default:
      return std::nullopt;
  }
}

// 使用鞋带公式计算四边形面积，过滤重合或近似共线的角点。
float PolygonArea(const std::array<cv::Point2f, 4>& points) noexcept {
  float twice_area = 0.0F;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& current = points[i];
    const auto& next = points[(i + 1) % points.size()];
    twice_area += current.x * next.y - next.x * current.y;
  }
  return std::abs(twice_area) * 0.5F;
}

// 任一 NaN 或 Inf 都会污染分类、坐标和 NMS，整行候选直接丢弃。
bool IsFiniteRow(const float* row) noexcept {
  for (std::size_t i = 0; i < K_OUTPUT_COLUMNS; ++i) {
    if (!std::isfinite(row[i])) {
      return false;
    }
  }
  return true;
}

// 返回分类通道最大值的首个索引，保持相同分数下的确定性。
int ArgMax(const float* values, int count) noexcept {
  return static_cast<int>(std::max_element(values, values + count) - values);
}

}  // namespace

LetterboxTransform MakeLetterboxTransform(int source_width, int source_height) {
  if (source_width <= 0 || source_height <= 0) {
    throw std::invalid_argument("letterbox source dimensions must be positive");
  }
  // 缩放后内容贴在画布左上角，右侧或下侧剩余区域由调用方填黑。
  const float SCALE =
      std::min(static_cast<float>(K_MODEL_WIDTH) / static_cast<float>(source_width),
               static_cast<float>(K_MODEL_HEIGHT) / static_cast<float>(source_height));
  const int CONTENT_WIDTH =
      std::clamp(static_cast<int>(std::lround(static_cast<float>(source_width) * SCALE)), 1,
                 K_MODEL_WIDTH);
  const int CONTENT_HEIGHT =
      std::clamp(static_cast<int>(std::lround(static_cast<float>(source_height) * SCALE)), 1,
                 K_MODEL_HEIGHT);
  return {.scale = SCALE,
          .content_width = CONTENT_WIDTH,
          .content_height = CONTENT_HEIGHT,
          .source_width = source_width,
          .source_height = source_height};
}

float StableSigmoid(float value) noexcept {
  // 按正负分支计算，避免大幅负数取反后 exp() 上溢。
  if (value >= 0.0F) {
    return 1.0F / (1.0F + std::exp(-value));
  }
  const float EXPONENTIAL = std::exp(value);
  return EXPONENTIAL / (1.0F + EXPONENTIAL);
}

float IntersectionOverUnion(const cv::Rect2f& lhs, const cv::Rect2f& rhs) noexcept {
  const float LEFT = std::max(lhs.x, rhs.x);
  const float TOP = std::max(lhs.y, rhs.y);
  const float RIGHT = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
  const float BOTTOM = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
  const float INTERSECTION_WIDTH = std::max(0.0F, RIGHT - LEFT);
  const float INTERSECTION_HEIGHT = std::max(0.0F, BOTTOM - TOP);
  const float INTERSECTION = INTERSECTION_WIDTH * INTERSECTION_HEIGHT;
  const float UNION_AREA = lhs.area() + rhs.area() - INTERSECTION;
  return UNION_AREA > 0.0F ? INTERSECTION / UNION_AREA : 0.0F;
}

std::vector<std::size_t> ClassAgnosticNms(const std::vector<ArmorDetection>& candidates,
                                          float iou_threshold) {
  if (!(iou_threshold >= 0.0F && iou_threshold <= 1.0F)) {
    throw std::invalid_argument("NMS IoU threshold must be in [0, 1]");
  }

  // 先按 objectness 稳定降序排列，确保高置信度候选优先保留。
  std::vector<std::size_t> order(candidates.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&candidates](std::size_t lhs, std::size_t rhs) {
    return candidates[lhs].objectness > candidates[rhs].objectness;
  });

  std::vector<std::size_t> kept;
  for (const auto CANDIDATE_INDEX : order) {
    bool suppressed = false;
    // NMS 不区分装甲类别；与任一已保留框过度重叠即被抑制。
    for (const auto KEPT_INDEX : kept) {
      if (IntersectionOverUnion(candidates[CANDIDATE_INDEX].bounding_box,
                                candidates[KEPT_INDEX].bounding_box) > iou_threshold) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      kept.push_back(CANDIDATE_INDEX);
    }
  }
  return kept;
}

DecodeResult DecodeYolo0526(const float* output, std::size_t rows, std::size_t columns,
                            const LetterboxTransform& transform, ArmorColor enemy_color,
                            const DecodeThresholds& thresholds) {
  if (output == nullptr) {
    throw std::invalid_argument("YOLO output pointer must not be null");
  }
  if (rows != K_OUTPUT_ROWS || columns != K_OUTPUT_COLUMNS) {
    throw std::invalid_argument("YOLO output must have shape [25200,22]");
  }
  if (!(thresholds.confidence > 0.0F && thresholds.confidence < 1.0F)) {
    throw std::invalid_argument("confidence threshold must be in (0, 1)");
  }

  DecodeResult result;
  std::vector<ArmorDetection> candidates;
  candidates.reserve(64);

  for (std::size_t row_index = 0; row_index < rows; ++row_index) {
    const float* row = output + row_index * columns;
    if (!IsFiniteRow(row)) {
      continue;
    }

    // 置信度只取第 8 通道的 sigmoid，不与颜色或类别分数相乘。
    const float OBJECTNESS = StableSigmoid(row[8]);
    if (OBJECTNESS < thresholds.confidence) {
      continue;
    }
    ++result.threshold_candidates;

    // 先把模型颜色通道转换为业务语义，再过滤灰、紫及非配置敌方颜色。
    const int COLOR_ID = ArgMax(row + 9, 4);
    const auto COLOR = ToArmorColor(COLOR_ID);
    if (!COLOR.has_value() || COLOR.value() != enemy_color) {
      continue;
    }
    const int LABEL_ID = ArgMax(row + 13, 9);

    // 候选中心落在 Letterbox 黑色填充区时，说明它不对应原图中的有效目标。
    const float CENTER_X = (row[0] + row[2] + row[4] + row[6]) * 0.25F;
    const float CENTER_Y = (row[1] + row[3] + row[5] + row[7]) * 0.25F;
    if (CENTER_X < 0.0F || CENTER_Y < 0.0F ||
        CENTER_X >= static_cast<float>(transform.content_width) ||
        CENTER_Y >= static_cast<float>(transform.content_height)) {
      continue;
    }

    // 左上对齐 Letterbox 没有平移量，只需除以缩放比例并裁剪到原图边界。
    const auto MAP_POINT = [&transform](float x, float y) {
      return cv::Point2f{
          std::clamp(x / transform.scale, 0.0F, static_cast<float>(transform.source_width - 1)),
          std::clamp(y / transform.scale, 0.0F, static_cast<float>(transform.source_height - 1))};
    };

    ArmorDetection detection;
    detection.color = COLOR.value();
    detection.label = ToArmorLabel(LABEL_ID);
    detection.objectness = OBJECTNESS;
    detection.corners = {
        MAP_POINT(row[0], row[1]),  // 模型 LT -> 对外 TL
        MAP_POINT(row[6], row[7]),  // 模型 RT -> 对外 TR
        MAP_POINT(row[4], row[5]),  // 模型 RB -> 对外 BR
        MAP_POINT(row[2], row[3]),  // 模型 LB -> 对外 BL
    };

    // 外接矩形服务于 NMS 和上层快速定位，四角点保留精确的装甲几何信息。
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& point : detection.corners) {
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
    }
    detection.bounding_box = {min_x, min_y, max_x - min_x, max_y - min_y};
    if (detection.bounding_box.width <= K_MIN_GEOMETRY_SIZE ||
        detection.bounding_box.height <= K_MIN_GEOMETRY_SIZE ||
        PolygonArea(detection.corners) <= K_MIN_POLYGON_AREA) {
      continue;
    }
    candidates.push_back(detection);
  }

  // 最终结果按 objectness 降序输出，便于调用方直接选择第一候选。
  const auto KEPT = ClassAgnosticNms(candidates, thresholds.nms_iou);
  result.detections.reserve(KEPT.size());
  for (const auto INDEX : KEPT) {
    result.detections.push_back(candidates[INDEX]);
  }
  return result;
}

}  // namespace mv::modules::detail
