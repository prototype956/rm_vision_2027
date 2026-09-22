#pragma once

#include "geometry/armor_type.hpp"
#include "geometry/rigid_transform.hpp"
#include "modules/armor_detector/armor_detector_output.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

#include <optional>

namespace mv::modules {

enum class TrackerState : std::uint8_t { LOST = 0, DETECTING, TRACKING, TEMP_LOST };

[[nodiscard]] const char* TrackerStateName(TrackerState state) noexcept;

struct PredictedArmorPose {
  int slot{0};
  geometry::RigidTransform world_t_armor;
};

struct PredictionHorizon {
  double seconds{0.0};
  geometry::Vector3 center_world{geometry::Vector3::Zero()};
  geometry::Quaternion orientation_world{geometry::Quaternion::Identity()};
  double yaw{0.0};
  std::array<PredictedArmorPose, 4> armors{};
};

/** @brief 下游控制消费的正式目标状态和预测时域。 */
struct ArmorPredictionOutput {
  std::uint64_t source_round_id{0};   ///< 原样转发来源图像回合，不参与目标估计。
  std::uint64_t track_generation{0};  ///< 每次滤波器成功初始化时递增。
  std::uint64_t sequence{0};
  std::optional<std::uint64_t> source_capture_timestamp_ns;
  std::chrono::steady_clock::time_point source_receive_steady_time{};
  std::optional<std::chrono::steady_clock::time_point> source_steady_time;
  TrackerState state{TrackerState::LOST};
  std::optional<ArmorLabel> label;
  std::optional<geometry::ArmorType> type;
  std::array<double, 13> state_vector{};
  std::array<double, 13> covariance_diagonal{};
  geometry::Vector3 center_world{geometry::Vector3::Zero()};
  geometry::Vector3 velocity_world{geometry::Vector3::Zero()};
  geometry::Quaternion orientation_world{geometry::Quaternion::Identity()};
  double yaw_velocity_rad_s{0.0};
  std::array<double, 2> radii_m{};
  double height_offset_m{0.0};
  double armor_tilt_rad{0.0};
  Eigen::Matrix3d center_covariance_world{Eigen::Matrix3d::Zero()};
  double yaw_variance_rad2{0.0};
  std::vector<PredictionHorizon> horizons;
};

[[nodiscard]] PredictionHorizon ExtrapolatePrediction(const ArmorPredictionOutput& prediction,
                                                      double seconds);

}  // namespace mv::modules
