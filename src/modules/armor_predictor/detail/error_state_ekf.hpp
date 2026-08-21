#pragma once

#include "modules/armor_predictor/armor_predictor_config.hpp"
#include "modules/armor_predictor/detail/armor_motion_model.hpp"
#include "modules/armor_predictor/detail/image_observation.hpp"

#include <vector>

#include <optional>
#include <span>

namespace mv::modules::detail {

struct EsekfUpdateDiagnostic {
  std::vector<double> innovation;
  std::optional<double> nis;
  int residual_dimension{0};
  int iteration_count{0};
  double yaw_velocity_update_rad_s{0.0};
};

/** @brief 四装甲13维右扰动迭代误差状态扩展卡尔曼滤波器。 */
class ArmorEsekf {
 public:
  void Reset() noexcept;
  void Initialize(const NominalState& state, const ArmorPredictorConfig& config);
  [[nodiscard]] bool Predict(double dt, const ArmorPredictorConfig& config,
                             double yaw_acceleration_variance);
  /** @brief 为已完成的本帧预测补加一份车体系 yaw 角加速度过程噪声。 */
  [[nodiscard]] bool AddYawAccelerationNoise(double dt, double variance);
  [[nodiscard]] bool Update(std::span<const ImageObservation> observations,
                            const ArmorPredictorConfig& config, EsekfUpdateDiagnostic& diagnostic);

  [[nodiscard]] bool Initialized() const noexcept { return initialized_; }
  [[nodiscard]] bool Diverged(const ArmorPredictorConfig& config) const noexcept;
  [[nodiscard]] const NominalState& State() const noexcept { return state_; }
  [[nodiscard]] const StateMatrix& Covariance() const noexcept { return covariance_; }

 private:
  NominalState state_;
  StateMatrix covariance_{StateMatrix::Identity()};
  bool initialized_{false};
};

}  // namespace mv::modules::detail
