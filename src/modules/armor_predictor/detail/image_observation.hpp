#pragma once

#include "modules/armor_predictor/armor_predictor_config.hpp"
#include "modules/armor_predictor/detail/armor_motion_model.hpp"

#include <array>

#include <Eigen/Core>
#include <variant>

namespace mv::modules::detail {

/** @brief 一根灯条的 UVL=[方向角,中心u,中心v,长度] 图像观测。 */
struct UvlObservation {
  Eigen::Vector4d value{Eigen::Vector4d::Zero()};
  int slot{0};
  bool left{true};
  bool standalone{false};  ///< 独立轮廓灯条使用更保守的观测噪声。
  double armor_tilt_rad{0.0};
  hal::CameraFrame::ArmorType type{hal::CameraFrame::ArmorType::SMALL};
  const hal::CameraFrame::FrameGeometry* geometry{nullptr};
};

/** @brief IPPE 提供的左右灯条中心 camera optical Z 深度差观测。 */
struct DepthDifferenceObservation {
  double value_m{0.0};
  int slot{0};
  double armor_tilt_rad{0.0};
  hal::CameraFrame::ArmorType type{hal::CameraFrame::ArmorType::SMALL};
  const hal::CameraFrame::FrameGeometry* geometry{nullptr};
};

using ImageObservation = std::variant<UvlObservation, DepthDifferenceObservation>;

struct LinearizedObservation {
  Eigen::VectorXd measurement;
  Eigen::VectorXd prediction;
  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd covariance;
};

/** @brief 从 TL、TR、BR、BL 四角原子生成左右两根灯条观测。 */
[[nodiscard]] std::array<UvlObservation, 2> MakeUvlObservations(
    const std::array<cv::Point2f, 4>& corners, const hal::CameraFrame::FrameGeometry& geometry,
    double armor_tilt_rad, hal::CameraFrame::ArmorType type, int slot);

/** @brief 从按图像上下排序的单根灯条端点生成独立 UVL 观测。 */
[[nodiscard]] UvlObservation MakeStandaloneUvlObservation(
    cv::Point2f top, cv::Point2f bottom, const hal::CameraFrame::FrameGeometry& geometry,
    double armor_tilt_rad, hal::CameraFrame::ArmorType type, int slot, bool left);

/** @brief 在给定名义状态处用 Jet 对一条图像观测求值和局部误差 Jacobian。 */
[[nodiscard]] LinearizedObservation LinearizeObservation(const ImageObservation& observation,
                                                         const NominalState& state,
                                                         const ArmorPredictorConfig& config);

}  // namespace mv::modules::detail
