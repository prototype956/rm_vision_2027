#include "modules/armor_predictor/detail/image_observation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace mv::modules::detail {
namespace {

constexpr double PI = 3.14159265358979323846;
using Jet = ceres::Jet<double, K_STATE_SIZE>;

template <typename T>
Eigen::Matrix<T, 4, 1> PredictUvl(const UvlObservation& observation, const StateValue<T>& state) {
  const auto CORNERS =
      ProjectArmorCorners(state, {.slot = observation.slot, .tilt_rad = observation.armor_tilt_rad},
                          observation.type, *observation.geometry);
  const int TOP = observation.left ? 0 : 1;
  const int BOTTOM = observation.left ? 3 : 2;
  const auto DELTA = CORNERS[BOTTOM] - CORNERS[TOP];
  using ceres::atan2;
  using ceres::sqrt;
  using std::atan2;
  using std::sqrt;
  Eigen::Matrix<T, 4, 1> result;
  result << atan2(DELTA.x(), DELTA.y()), (CORNERS[TOP].x() + CORNERS[BOTTOM].x()) / T(2.0),
      (CORNERS[TOP].y() + CORNERS[BOTTOM].y()) / T(2.0), sqrt(DELTA.squaredNorm());
  return result;
}

template <typename T>
T PredictDepthDifference(const DepthDifferenceObservation& observation,
                         const StateValue<T>& state) {
  const double WIDTH = observation.type == hal::CameraFrame::ArmorType::LARGE ? 0.225 : 0.135;
  const auto WORLD_T_CAMERA =
      ComposeTransform(CastTransform<T>(observation.geometry->world_t_gimbal),
                       CastTransform<T>(observation.geometry->gimbal_t_camera_optical));
  const auto CAMERA_T_ARMOR = ComposeTransform(
      InverseTransform(WORLD_T_CAMERA),
      WorldTArmor(state, {.slot = observation.slot, .tilt_rad = observation.armor_tilt_rad}));
  const Eigen::Matrix<T, 3, 1> LEFT =
      CAMERA_T_ARMOR.translation +
      CAMERA_T_ARMOR.rotation * Eigen::Matrix<T, 3, 1>(T(-0.5 * WIDTH), T(0.0), T(0.0));
  const Eigen::Matrix<T, 3, 1> RIGHT =
      CAMERA_T_ARMOR.translation +
      CAMERA_T_ARMOR.rotation * Eigen::Matrix<T, 3, 1>(T(0.5 * WIDTH), T(0.0), T(0.0));
  return LEFT.z() - RIGHT.z();
}

template <typename Observation>
LinearizedObservation Linearize(const Observation& observation, const NominalState& state,
                                const ArmorPredictorConfig& config) {
  if (observation.geometry == nullptr)
    throw std::invalid_argument("image observation requires frame geometry");

  Eigen::Matrix<Jet, K_STATE_SIZE, 1> error;
  for (int index = 0; index < K_STATE_SIZE; ++index) {
    error[index] = Jet(0.0);
    error[index].v[index] = 1.0;
  }
  auto jet_state = CastState<Jet>(state);
  Inject(error, jet_state);

  LinearizedObservation result;
  if constexpr (std::is_same_v<Observation, UvlObservation>) {
    const auto PREDICTED = PredictUvl(observation, jet_state);
    result.measurement = observation.value;
    result.prediction.resize(4);
    result.jacobian.resize(4, K_STATE_SIZE);
    for (int row = 0; row < 4; ++row) {
      result.prediction[row] = PREDICTED[row].a;
      result.jacobian.row(row) = PREDICTED[row].v.transpose();
    }
    result.residual = result.measurement - result.prediction;
    while (result.residual[0] > PI)
      result.residual[0] -= 2.0 * PI;
    while (result.residual[0] < -PI)
      result.residual[0] += 2.0 * PI;
    const double LENGTH = std::max(1.0, observation.value[3]);
    const double CENTER_RATIO = observation.standalone
                                    ? config.standalone_uvl_center_sigma_length_ratio
                                    : config.uvl_center_sigma_length_ratio;
    const double LENGTH_RATIO = observation.standalone
                                    ? config.standalone_uvl_length_sigma_length_ratio
                                    : config.uvl_length_sigma_length_ratio;
    const double ANGLE_SIGMA =
        observation.standalone ? config.standalone_uvl_angle_sigma_rad : config.uvl_angle_sigma_rad;
    const double CENTER_SIGMA = CENTER_RATIO * LENGTH;
    const double LENGTH_SIGMA = LENGTH_RATIO * LENGTH;
    result.covariance = Eigen::Vector4d(ANGLE_SIGMA * ANGLE_SIGMA, CENTER_SIGMA * CENTER_SIGMA,
                                        CENTER_SIGMA * CENTER_SIGMA, LENGTH_SIGMA * LENGTH_SIGMA)
                            .asDiagonal();
  } else {
    const Jet PREDICTED = PredictDepthDifference(observation, jet_state);
    result.measurement = Eigen::VectorXd::Constant(1, observation.value_m);
    result.prediction = Eigen::VectorXd::Constant(1, PREDICTED.a);
    result.residual = result.measurement - result.prediction;
    result.jacobian.resize(1, K_STATE_SIZE);
    result.jacobian.row(0) = PREDICTED.v.transpose();
    result.covariance = Eigen::MatrixXd::Constant(
        1, 1, config.depth_difference_sigma_m * config.depth_difference_sigma_m);
  }
  return result;
}

}  // namespace

std::array<UvlObservation, 2> MakeUvlObservations(const std::array<cv::Point2f, 4>& corners,
                                                  const hal::CameraFrame::FrameGeometry& geometry,
                                                  double armor_tilt_rad,
                                                  hal::CameraFrame::ArmorType type, int slot) {
  const auto MAKE = [&](bool left) {
    const int TOP = left ? 0 : 1;
    const int BOTTOM = left ? 3 : 2;
    const cv::Point2f DELTA = corners[BOTTOM] - corners[TOP];
    UvlObservation observation;
    observation.value << std::atan2(DELTA.x, DELTA.y),
        0.5 * static_cast<double>(corners[TOP].x + corners[BOTTOM].x),
        0.5 * static_cast<double>(corners[TOP].y + corners[BOTTOM].y),
        std::hypot(static_cast<double>(DELTA.x), static_cast<double>(DELTA.y));
    observation.slot = slot;
    observation.left = left;
    observation.armor_tilt_rad = armor_tilt_rad;
    observation.type = type;
    observation.geometry = &geometry;
    return observation;
  };
  return {MAKE(true), MAKE(false)};
}

UvlObservation MakeStandaloneUvlObservation(cv::Point2f top, cv::Point2f bottom,
                                            const hal::CameraFrame::FrameGeometry& geometry,
                                            double armor_tilt_rad, hal::CameraFrame::ArmorType type,
                                            int slot, bool left) {
  const cv::Point2f DELTA = bottom - top;
  UvlObservation result;
  result.value << std::atan2(DELTA.x, DELTA.y), 0.5 * static_cast<double>(top.x + bottom.x),
      0.5 * static_cast<double>(top.y + bottom.y),
      std::hypot(static_cast<double>(DELTA.x), static_cast<double>(DELTA.y));
  result.slot = slot;
  result.left = left;
  result.standalone = true;
  result.armor_tilt_rad = armor_tilt_rad;
  result.type = type;
  result.geometry = &geometry;
  return result;
}

LinearizedObservation LinearizeObservation(const ImageObservation& observation,
                                           const NominalState& state,
                                           const ArmorPredictorConfig& config) {
  return std::visit([&](const auto& value) { return Linearize(value, state, config); },
                    observation);
}

}  // namespace mv::modules::detail
