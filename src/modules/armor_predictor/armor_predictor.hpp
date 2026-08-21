#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_light_detector/armor_light_detector.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"
#include "modules/armor_predictor/armor_predictor_config.hpp"

#include <memory>

#include <span>

namespace mv::modules {

/**
 * @brief 使用四装甲13维图像观测 ESEKF 跟踪单个机器人目标。
 *
 * PnP 仅用于 LOST 初始化、单装甲深度差和诊断；常规更新消费同帧二维角点。
 */
class ArmorPredictor final {
 public:
  /** @brief 保存已校验配置并创建空跟踪状态。 */
  explicit ArmorPredictor(ArmorPredictorConfig config);
  ~ArmorPredictor();

  ArmorPredictor(const ArmorPredictor& other);
  ArmorPredictor& operator=(const ArmorPredictor& other);
  ArmorPredictor(ArmorPredictor&& other) noexcept;
  ArmorPredictor& operator=(ArmorPredictor&& other) noexcept;

  /**
   * @brief 处理同帧检测、角点精修和 PnP 结果，推进跟踪器并生成多时域预测。
   * @param frame 提供帧序号、单调时间戳和 world/gimbal/camera 同帧变换。
   * @param detections 当前帧网络四角和标签。
   * @param refinements 同顺序角点精修；成功整块采用，否则整块回退网络四角。
   * @param pnp_result PnP 结果，仅承担初始化、单装甲深度差和诊断职责。
   * @param lightbar_result 独立灯条候选；不得用于 LOST 初始化。
   * @return 状态、关联、滤波诊断及各配置时域的四装甲位姿。
   */
  [[nodiscard]] ArmorPredictionResult ProcessFrame(
      const hal::CameraFrame& frame, std::span<const ArmorDetection> detections,
      std::span<const CornerRefinementResult> refinements, const ArmorPnpFrameResult& pnp_result,
      const LightbarDetectionResult& lightbar_result);

 private:
  struct Impl;                  ///< 隔离 Eigen、关联器与状态机实现。
  std::unique_ptr<Impl> impl_;  ///< 当前预测器唯一拥有的可复制实现状态。
};

}  // namespace mv::modules
