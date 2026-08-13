#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"
#include "modules/armor_predictor/armor_predictor_config.hpp"

#include <memory>

namespace mv::modules {

/**
 * @brief 使用四装甲扩展卡尔曼滤波器跟踪单个机器人目标。
 *
 * 输入仅消费正式检测链的 PnP 结果。预测器按标签优先级选择目标，以四个相差 90° 的
 * 装甲槽位进行一对一关联，并维护 LOST、DETECTING、TRACKING 和 TEMP_LOST 状态机。
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
   * @brief 处理同帧相机元数据和 PnP 结果，推进跟踪器并生成多时域预测。
   * @param frame 提供帧序号、单调时间戳和 world/gimbal/camera 同帧变换。
   * @param pnp_result 当前帧真值与检测 PnP 尝试；仅 DETECTION 成功结果参与跟踪。
   * @return 状态、关联、滤波诊断及各配置时域的四装甲位姿。
   */
  [[nodiscard]] ArmorPredictionResult ProcessFrame(const hal::CameraFrame& frame,
                                                   const ArmorPnpFrameResult& pnp_result);

 private:
  struct Impl;                  ///< 隔离 Eigen、关联器与状态机实现。
  std::unique_ptr<Impl> impl_;  ///< 当前预测器唯一拥有的可复制实现状态。
};

}  // namespace mv::modules
