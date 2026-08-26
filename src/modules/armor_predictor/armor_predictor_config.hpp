#pragma once

#include <array>

#include <yaml-cpp/yaml.h>

namespace mv::modules {

/** @brief 装甲预测器严格配置 Schema 版本；入口加载和模块解析必须共用此常量。 */
inline constexpr int ARMOR_PREDICTOR_CONFIG_SCHEMA_VERSION = 6;

/** @brief 13维整车 ESEKF、图像关联、观测噪声及预测时域参数。 */
struct ArmorPredictorConfig {
  int min_detect_count{5};
  int max_temp_lost_count{15};
  double max_dt_s{0.1};

  int esekf_iterations{5};
  double vehicle_initial_radius_m{0.21154};
  double hero_initial_radius_m{0.23930};
  double vehicle_armor_tilt_rad{-0.265216};
  double hero_armor_tilt_rad{-0.256525};
  double min_radius_m{0.05};
  double max_radius_m{0.5};
  double max_height_offset_m{0.5};
  double max_yaw_velocity_rad_s{20.0};

  /** @brief cx,vx,cy,vy,cz,vz,rx,ry,rz,vyaw,log_r1,log_r2,h 的初始方差。 */
  std::array<double, 13> initial_covariance_diagonal{1.0, 10.0, 1.0,   10.0, 1.0, 10.0, 1.0,
                                                     1.0, 1.0,  100.0, 1.0,  1.0, 1.0};
  std::array<double, 3> body_acceleration_variance{30.0, 30.0, 1.0};
  double yaw_acceleration_variance{30.0};
  double roll_pitch_random_walk_variance_per_s{0.1};
  double radius_random_walk_variance_m2_per_s{1.0e-7};
  double height_random_walk_variance_m2_per_s{1.0e-7};

  int visible_slot_count{3};
  double association_center_weight{5.0};
  double association_edge_angle_weight{10.0};
  double association_perimeter_ratio_weight{1.0};
  double association_initial_gate{1000.0};
  double association_gate{200.0};

  double light_dedup_center_length_ratio{0.5};
  double light_dedup_angle_gate_rad{0.20};
  double light_dedup_log_length_gate{0.35};
  double light_match_log_length_gate{0.25};
  double light_match_angle_gate_rad{0.25};
  double light_match_endpoint_distance_length_ratio{2.0};
  double light_match_position_weight{1.0};
  double light_match_angle_weight{2.0};
  double light_match_length_weight{1.0};

  double maneuver_association_cost_trigger{50.0};
  int maneuver_confirmation_frames{2};
  double maneuver_confirmation_window_s{0.15};
  double maneuver_hold_s{0.30};
  double maneuver_trigger_yaw_acceleration_variance{10000.0};
  double maneuver_active_yaw_acceleration_variance{2500.0};
  double maneuver_recovery_gate{500.0};
  double maneuver_nis_per_dof_gate{3.0};

  double uvl_center_sigma_length_ratio{0.2};
  double uvl_length_sigma_length_ratio{0.5};
  double uvl_angle_sigma_rad{0.1};
  double standalone_uvl_center_sigma_length_ratio{0.35};
  double standalone_uvl_length_sigma_length_ratio{0.70};
  double standalone_uvl_angle_sigma_rad{0.15};
  double depth_difference_sigma_m{0.1};

  std::array<int, 5> label_priorities{3, 1, 2, 0, 0};
  std::array<double, 4> prediction_horizons_s{0.0, 0.05, 0.1, 0.2};
};

/** @brief 解析并严格校验13维装甲预测器配置。 */
[[nodiscard]] ArmorPredictorConfig ParseArmorPredictorConfig(const YAML::Node& root);

}  // namespace mv::modules
