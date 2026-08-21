#pragma once

#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_light_detector/armor_light_detector_config.hpp"

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <span>

namespace mv::modules {

enum class LightbarThresholdSource { FIXED, NETWORK_REFERENCE };

[[nodiscard]] const char* LightbarThresholdSourceName(LightbarThresholdSource source) noexcept;

/** @brief 单根全图传统 CV 灯条检测。端点按图像 y 从小到大固定为 top/bottom。 */
struct LightbarDetection {
  std::size_t input_index{0};
  ArmorColor color{ArmorColor::RED};
  cv::Point2f top{};
  cv::Point2f bottom{};
  cv::Point2f center{};
  double length_px{0.0};
  double width_px{0.0};
  double angle_rad{0.0};
  double color_difference{0.0};
  double score{0.0};
};

/** @brief 最近一帧独立灯条检测的筛选计数和耗时。 */
struct LightbarDetectorStats {
  bool enabled{true};
  bool valid_input{true};
  int binary_threshold{0};
  LightbarThresholdSource threshold_source{LightbarThresholdSource::FIXED};
  std::size_t reference_lightbars{0};
  std::size_t contours{0};
  std::size_t geometry_candidates{0};
  std::size_t color_candidates{0};
  std::size_t kept_candidates{0};
  double elapsed_ms{0.0};
  std::string rejection_reason;
};

struct LightbarDetectionResult {
  std::vector<LightbarDetection> detections;
  LightbarDetectorStats stats;
};

/** @brief 在全图以亮度、轮廓几何和敌方颜色检测未成对的独立灯条。 */
class ArmorLightDetector final {
 public:
  ArmorLightDetector(ArmorLightDetectorConfig config, ArmorColor enemy_color);

  /**
   * @brief 检测同帧独立灯条，并用网络装甲灯条亮度自适应二值阈值。
   * @return 输入无效时返回空结果和诊断，不抛出逐帧运行异常。
   */
  [[nodiscard]] LightbarDetectionResult Detect(
      const cv::Mat& bgr_image, const cv::Mat& gray_image,
      std::span<const ArmorDetection> detections,
      std::span<const CornerRefinementResult> refinements) const noexcept;

 private:
  ArmorLightDetectorConfig config_;
  ArmorColor enemy_color_{ArmorColor::RED};
};

}  // namespace mv::modules
