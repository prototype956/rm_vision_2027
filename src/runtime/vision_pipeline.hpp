#pragma once

#include "frame/frame_packet.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_light_detector/armor_light_detector.hpp"
#include "modules/armor_pnp/armor_pnp.hpp"
#include "modules/armor_predictor/armor_predictor.hpp"

#include <vector>

namespace mv::runtime {

/** @brief 单帧视觉流水线所需的全部已校验算法配置。 */
struct VisionPipelineConfig {
  modules::ArmorDetectorConfig detector;
  modules::ArmorPnpConfig pnp;
  modules::ArmorPredictorConfig predictor;
  modules::ArmorCornerRefinerConfig corner_refiner;
  modules::ArmorLightDetectorConfig light_detector;
};

/** @brief 同一相机帧经过完整感知链后产生的正式结果和诊断。 */
struct VisionFrameResult {
  std::vector<modules::ArmorDetection> detections;
  modules::DetectorStats detector_stats;
  std::vector<modules::CornerRefinementResult> refinements;
  modules::LightbarDetectionResult lightbars;
  modules::ArmorPnpFrameResult pnp;
  modules::ArmorPredictionResult prediction;
};

/** @brief 串行执行检测、角点精修、独立灯条、PnP 和目标预测的单帧流水线。 */
class VisionPipeline final {
 public:
  /** @brief 根据已解析配置创建并初始化全部算法模块。 */
  explicit VisionPipeline(const VisionPipelineConfig& config);

  VisionPipeline(const VisionPipeline&) = delete;
  VisionPipeline& operator=(const VisionPipeline&) = delete;
  VisionPipeline(VisionPipeline&&) = delete;
  VisionPipeline& operator=(VisionPipeline&&) = delete;

  /**
   * @brief 同步处理一帧图像，并保持所有中间结果来自同一帧。
   * @param frame 相机 HAL 返回的完整帧。
   * @return 检测、PnP、预测和诊断结果。
   */
  [[nodiscard]] VisionFrameResult Process(const frame::FramePacket& packet);

 private:
  modules::YoloArmorDetector detector_;
  modules::ArmorPnp pnp_;
  modules::ArmorPredictor predictor_;
  modules::ArmorCornerRefiner corner_refiner_;
  modules::ArmorLightDetector light_detector_;
};

}  // namespace mv::runtime
