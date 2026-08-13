#pragma once

#include "modules/armor_predictor/armor_prediction_types.hpp"
#include "modules/armor_predictor/armor_predictor_config.hpp"
#include "modules/armor_predictor/detail/armor_predictor_internal.hpp"

#include <span>

namespace mv::modules::detail {

/** @brief 四装甲 11 状态扩展卡尔曼滤波器，不包含目标选择和跟踪状态机。 */
class ArmorEkf final {
 public:
  /** @brief 清空状态和协方差。 */
  void Reset() noexcept;
  /** @brief 使用首块装甲观测和标签对应初始半径初始化车辆中心状态。 */
  void Initialize(const Observation& observation, const ArmorPredictorConfig& config);
  /** @brief 按三轴与 yaw 匀速模型执行时间预测。 */
  void Predict(double dt, const ArmorPredictorConfig& config);
  /** @brief 将全部已关联装甲拼成联合量测，执行一次 EKF 更新并写入诊断。 */
  void Update(std::span<const Observation> observations, std::span<const int> slots,
              const hal::CameraFrame::FrameGeometry& geometry, const ArmorPredictorConfig& config,
              ArmorPredictionResult& result);

  /** @brief 检查非有限状态、协方差及两组交替装甲半径是否越界。 */
  [[nodiscard]] bool Diverged(const ArmorPredictorConfig& config) const noexcept;
  [[nodiscard]] const StateVector& State() const noexcept { return state_; }
  [[nodiscard]] const StateMatrix& Covariance() const noexcept { return covariance_; }

 private:
  StateVector state_{StateVector::Zero()};
  StateMatrix covariance_{StateMatrix::Zero()};
};

}  // namespace mv::modules::detail
