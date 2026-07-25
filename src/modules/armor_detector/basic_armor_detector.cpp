/**
 * @file basic_armor_detector.cpp
 * @brief 传统视觉装甲板检测器实现
 *
 * 【Pimpl 布局】
 *   Impl 结构体集中存放：参数、调试开关、调试数据、LightBar 结构体、以及所有
 *   内部算法函数（BuildChannelDiff / MakeBinary / FindLightBars /
 *   IsValidArmor / MakeDetection）。
 *
 * 【lights_vis 可视化移出说明】
 *   过滤原因色码可视化（绿/橙/紫）已移至 mv::tool::PaintLightBarsVis()
 *   （tool/debug/light_vis_painter.hpp），Detect() 不再生成该图。
 */
#include "basic_armor_detector.hpp"

#include "core/logger.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace mv::modules {

// ── 注册到工厂 ────────────────────────────────────────────────────────────

namespace {
// NOLINTNEXTLINE(cert-err58-cpp)
const bool BASIC_ARMOR_DETECTOR_REGISTERED = [] {
  ::mv::Factory<::mv::IDetector>::Register("basic",
                                           [] { return std::make_unique<BasicArmorDetector>(); });
  return true;
}();
}  // namespace

// ── Impl：私有实现细节（struct → 所有成员 public，命名 lower_case 无后缀）────

struct BasicArmorDetector::Impl {
  // ── 数据成员 ──────────────────────────────────────────────────────────────

  Params params;
  DebugData debug_data;
  bool debug_enabled{false};
  bool initialized{false};

  // ── 内部数据结构 ──────────────────────────────────────────────────────────

  /** 灯条描述符：归一化后的几何属性 */
  struct LightBar {
    cv::RotatedRect rect;  ///< 最小外接矩形（height >= width）
    cv::Point2f top;       ///< 灯条顶端中点（y 较小）
    cv::Point2f bottom;    ///< 灯条底端中点（y 较大）
    float length{0.0F};    ///< 长轴（height）
    float width{0.0F};     ///< 短轴（width）
    float tilt{0.0F};      ///< 相对垂直方向的倾斜角（°，0=垂直，90=水平）
  };

  // ── 算法函数 ──────────────────────────────────────────────────────────────

  /**
   * @brief 双通道差值 AND 颜色掩码（仅彩色边缘，白色过曝中心在此为空）
   *
   * red:  (R-B > green_thresh) AND (R-G > green_thresh)
   * blue: (B-R > green_thresh) AND (B-G > green_thresh)
   */
  [[nodiscard]] cv::Mat BuildChannelDiff(const cv::Mat& bgr, ArmorColor color) const {
    std::vector<cv::Mat> channels;  // [0]=B [1]=G [2]=R
    cv::split(bgr, channels);

    cv::Mat mask_main;
    cv::Mat mask_secondary;

    if (color == ArmorColor::RED) {
      cv::subtract(channels[2], channels[0], mask_main);       // R - B
      cv::subtract(channels[2], channels[1], mask_secondary);  // R - G
    } else if (color == ArmorColor::BLUE) {
      cv::subtract(channels[0], channels[2], mask_main);       // B - R
      cv::subtract(channels[0], channels[1], mask_secondary);  // B - G
    } else {
      cv::Mat gray_img;
      cv::cvtColor(bgr, gray_img, cv::COLOR_BGR2GRAY);
      return gray_img;
    }

    cv::threshold(mask_main, mask_main, params.green_thresh, 255, cv::THRESH_BINARY);
    cv::threshold(mask_secondary, mask_secondary, params.green_thresh, 255, cv::THRESH_BINARY);

    cv::Mat result;
    cv::bitwise_and(mask_main, mask_secondary, result);
    return result;
  }

  /**
   * @brief 三路融合：gray_bin AND dilated_color_bin → 形态学精修 → 二值图
   *
   * debug_enabled=true 时将中间图写入 debug_data。
   */
  [[nodiscard]] cv::Mat MakeBinary(const cv::Mat& frame, ArmorColor color) {
    // 路径 1：灰度亮度掩码（含白色过曝中心）
    cv::Mat gray_img;
    cv::cvtColor(frame, gray_img, cv::COLOR_BGR2GRAY);
    cv::Mat gray_bin;
    cv::threshold(gray_img, gray_bin, params.light_thresh, 255, cv::THRESH_BINARY);

    // 路径 2：双通道差值颜色掩码（仅彩色边缘）
    cv::Mat color_bin = BuildChannelDiff(frame, color);
    if (debug_enabled) {
      debug_data.diff = color_bin.clone();
    }

    // 大核膨胀：让彩色边缘扩散覆盖过曝中心
    // 核 11px ≈ 过曝扩散半径上界；更大的核会把细长灯条胀圆，影响 fitEllipse 精度
    const cv::Mat KERNEL_EXPAND = cv::getStructuringElement(cv::MORPH_ELLIPSE, {11, 11});
    cv::Mat color_expanded;
    cv::dilate(color_bin, color_expanded, KERNEL_EXPAND);

    // 交集：完整实心灯条
    cv::Mat binary;
    cv::bitwise_and(color_expanded, gray_bin, binary);

    // 闭运算补空洞 + 轻度膨胀连通 + 中值滤波去单像素抖动
    const cv::Mat KERNEL_CLOSE = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
    const cv::Mat KERNEL_DILATE = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, KERNEL_CLOSE);
    cv::dilate(binary, binary, KERNEL_DILATE);
    cv::medianBlur(binary, binary, 3);

    if (debug_enabled) {
      debug_data.binary = binary.clone();
    }
    return binary;
  }

  /**
   * @brief 从二值图中提取所有通过过滤的灯条
   *
   * 过滤规则：
   *   1. 轮廓面积 >= min_area，轮廓点数 >= 5（fitEllipse 要求）
   *   2. WID/LEN（短/长轴比）∈ [min_light_ratio, max_light_ratio]
   *   3. TILT（fitEllipse 长轴相对垂直方向的偏角）<= max_light_angle
   */
  [[nodiscard]] std::vector<LightBar> FindLightBars(const cv::Mat& binary) const {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> lights;
    lights.reserve(contours.size());

    for (const auto& contour : contours) {
      const auto AREA = static_cast<float>(cv::contourArea(contour));
      if (AREA < params.min_area || contour.size() < 5) {
        continue;
      }

      // 归一化 RotatedRect：使 height >= width
      cv::RotatedRect rect = cv::minAreaRect(contour);
      if (rect.size.width > rect.size.height) {
        std::swap(rect.size.width, rect.size.height);
        rect.angle += 90.0F;
      }

      const float LEN = rect.size.height;
      const float WID = rect.size.width;
      const float RATIO = WID / LEN;
      if (RATIO < params.min_light_ratio || RATIO > params.max_light_ratio) {
        continue;
      }

      // fitEllipse 对过曝空洞和形态学膨胀更鲁棒；angle ∈ [0,180)，90° 对应垂直
      const float TILT = std::abs(cv::fitEllipse(contour).angle - 90.0F);
      if (TILT > params.max_light_angle) {
        continue;
      }

      // 沿长轴方向计算顶/底端中点（rect.angle+90° 为长轴在图像坐标系中的方向）
      const float LONG_AX_RAD = (rect.angle + 90.0F) * static_cast<float>(CV_PI) / 180.0F;
      const cv::Point2f HALF_AXIS(std::cos(LONG_AX_RAD) * LEN * 0.5F,
                                  std::sin(LONG_AX_RAD) * LEN * 0.5F);
      const cv::Point2f POINT_A = rect.center - HALF_AXIS;
      const cv::Point2f POINT_B = rect.center + HALF_AXIS;
      const cv::Point2f TOP_PT = (POINT_A.y < POINT_B.y) ? POINT_A : POINT_B;
      const cv::Point2f BOTTOM_PT = (POINT_A.y < POINT_B.y) ? POINT_B : POINT_A;

      lights.push_back({rect, TOP_PT, BOTTOM_PT, LEN, WID, TILT});
    }

    return lights;
  }

  /**
   * @brief 判断两灯条是否构成合法装甲板（left.center.x < right.center.x）
   *
   * 过滤规则：
   *   1. 倾斜角差 <= max_angle_diff
   *   2. 高度比（短/长）>= 0.6
   *   3. 中心 Y 差 <= 0.5 × avg_len
   *   4. 水平间距 / avg_len ∈ [min_armor_ratio, max_armor_ratio]
   */
  [[nodiscard]] bool IsValidArmor(const LightBar& left, const LightBar& right) const {
    if (std::abs(left.tilt - right.tilt) > params.max_angle_diff) {
      return false;
    }
    const float MAX_LEN = std::max(left.length, right.length);
    const float MIN_LEN = std::min(left.length, right.length);
    if (MIN_LEN / MAX_LEN < 0.6F) {
      return false;
    }
    const float AVG_LEN = (left.length + right.length) * 0.5F;
    if (std::abs(left.rect.center.y - right.rect.center.y) > AVG_LEN * 0.5F) {
      return false;
    }
    const float DELTA_X = right.rect.center.x - left.rect.center.x;
    if (DELTA_X <= 0.0F) {
      return false;
    }
    const float ARMOR_RATIO = DELTA_X / AVG_LEN;
    return ARMOR_RATIO >= params.min_armor_ratio && ARMOR_RATIO <= params.max_armor_ratio;
  }

  /**
   * @brief 由一对灯条构造 Detection（不填充 3D 信息）
   *
   * 四角点顺序：BL, BR, TR, TL（顺时针从左下开始）。
   * 装甲类型：水平跨度 / avg_len > 3.5 → BIG，否则 SMALL。
   */
  [[nodiscard]] static Detection MakeDetection(const LightBar& left, const LightBar& right) {
    Detection det;

    det.points[0] = left.bottom;   // BL
    det.points[1] = right.bottom;  // BR
    det.points[2] = right.top;     // TR
    det.points[3] = left.top;      // TL

    const float MIN_X =
        std::min({det.points[0].x, det.points[1].x, det.points[2].x, det.points[3].x});
    const float MAX_X =
        std::max({det.points[0].x, det.points[1].x, det.points[2].x, det.points[3].x});
    const float MIN_Y =
        std::min({det.points[0].y, det.points[1].y, det.points[2].y, det.points[3].y});
    const float MAX_Y =
        std::max({det.points[0].y, det.points[1].y, det.points[2].y, det.points[3].y});
    det.box = {MIN_X, MIN_Y, MAX_X - MIN_X, MAX_Y - MIN_Y};

    det.confidence = 1.0F;
    det.color = ArmorColor::UNKNOWN;
    det.number = ArmorNumber::UNKNOWN;

    const float AVG_LEN = (left.length + right.length) * 0.5F;
    det.type = (det.box.width / AVG_LEN > 3.5F) ? ArmorType::BIG : ArmorType::SMALL;

    const cv::Point2f CENTER = det.Center();
    const cv::Point2f IMG_CTR = {640.0F, 512.0F};
    det.distance_to_center = static_cast<double>(cv::norm(CENTER - IMG_CTR));

    det.is_solved = false;
    return det;
  }
};

// ── 构造 / 析构 ────────────────────────────────────────────────────────────

BasicArmorDetector::BasicArmorDetector() : impl_(std::make_unique<Impl>()) {}
BasicArmorDetector::~BasicArmorDetector() = default;

BasicArmorDetector::BasicArmorDetector(BasicArmorDetector&&) noexcept = default;
BasicArmorDetector& BasicArmorDetector::operator=(BasicArmorDetector&&) noexcept = default;

// ── IDetector 接口实现 ────────────────────────────────────────────────────

bool BasicArmorDetector::Init(const YAML::Node& config) {
  if (config && config["detector"]) {
    const auto& det = config["detector"];
    // clang-format off
    if (det["light_thresh"])    { impl_->params.light_thresh    = det["light_thresh"].as<int>();    }
    if (det["green_thresh"])    { impl_->params.green_thresh    = det["green_thresh"].as<int>();    }
    if (det["white_thresh"])    { impl_->params.white_thresh    = det["white_thresh"].as<int>();    }
    if (det["min_light_ratio"]) { impl_->params.min_light_ratio = det["min_light_ratio"].as<float>(); }
    if (det["max_light_ratio"]) { impl_->params.max_light_ratio = det["max_light_ratio"].as<float>(); }
    if (det["max_light_angle"]) { impl_->params.max_light_angle = det["max_light_angle"].as<float>(); }
    if (det["min_armor_ratio"]) { impl_->params.min_armor_ratio = det["min_armor_ratio"].as<float>(); }
    if (det["max_armor_ratio"]) { impl_->params.max_armor_ratio = det["max_armor_ratio"].as<float>(); }
    if (det["max_angle_diff"])  { impl_->params.max_angle_diff  = det["max_angle_diff"].as<float>();  }
    if (det["min_area"])        { impl_->params.min_area        = det["min_area"].as<float>();        }
    // clang-format on
  }
  impl_->initialized = true;
  MV_LOG_INFO("BasicArmorDetector",
              "Init OK — thresh={} green={} angle_lim={:.1f}° armor_ratio=[{:.1f},{:.1f}]",
              impl_->params.light_thresh, impl_->params.green_thresh, impl_->params.max_light_angle,
              impl_->params.min_armor_ratio, impl_->params.max_armor_ratio);
  return true;
}

std::vector<Detection> BasicArmorDetector::Detect(const cv::Mat& frame, ArmorColor enemy_color) {
  if (frame.empty()) {
    MV_LOG_WARN("BasicArmorDetector", "Detect() called with empty frame");
    return {};
  }

  // ── 预处理：生成二值图（三路融合 + 形态学）────────────────────────────────
  cv::Mat binary = impl_->MakeBinary(frame, enemy_color);

  // ── 提取灯条并按 x 排序 ─────────────────────────────────────────────────
  std::vector<Impl::LightBar> lights = impl_->FindLightBars(binary);
  std::sort(lights.begin(), lights.end(), [](const Impl::LightBar& lhs, const Impl::LightBar& rhs) {
    return lhs.rect.center.x < rhs.rect.center.x;
  });

  if (lights.size() < 2) {
    return {};
  }

  // ── 两两配对 ────────────────────────────────────────────────────────────
  std::vector<Detection> detections;
  for (std::size_t idx = 0; idx + 1 < lights.size(); ++idx) {
    for (std::size_t jdx = idx + 1; jdx < lights.size(); ++jdx) {
      if (impl_->IsValidArmor(lights[idx], lights[jdx])) {
        detections.emplace_back(Impl::MakeDetection(lights[idx], lights[jdx]));
      }
    }
  }

  if (!detections.empty()) {
    // 补全颜色字段：Detect() 按 enemy_color 滤波，所有结果均属于 enemy_color
    // EkfTracker::Track() 依赖 d.color == enemy_color 过滤，UNKNOWN 会导致永远 LOST
    const cv::Point2f IMG_CTR{
        static_cast<float>(frame.cols) * 0.5F,
        static_cast<float>(frame.rows) * 0.5F,
    };
    for (auto& det : detections) {
      det.color = enemy_color;
      det.distance_to_center = static_cast<double>(cv::norm(det.Center() - IMG_CTR));
    }
  }

  MV_LOG_DEBUG("BasicArmorDetector", "Frame: {} lights, {} armors", lights.size(),
               detections.size());
  return detections;
}

bool BasicArmorDetector::IsInitialized() const noexcept {
  return impl_->initialized;
}

// ── 参数访问 ──────────────────────────────────────────────────────────────

void BasicArmorDetector::SetParams(const Params& params) noexcept {
  impl_->params = params;
}

const BasicArmorDetector::Params& BasicArmorDetector::GetParams() const noexcept {
  return impl_->params;
}

// ── 调试开关 ──────────────────────────────────────────────────────────────

void BasicArmorDetector::EnableDebug(bool enabled) noexcept {
  impl_->debug_enabled = enabled;
}

const BasicArmorDetector::DebugData& BasicArmorDetector::GetDebugData() const noexcept {
  return impl_->debug_data;
}

}  // namespace mv::modules
