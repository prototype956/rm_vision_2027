#pragma once

#include <array>

#include <yaml-cpp/yaml.h>

namespace mv::modules {

/** @brief 四装甲目标跟踪、关联、EKF 噪声及预测时域参数。 */
struct ArmorPredictorConfig {
  int min_detect_count{5};      ///< DETECTING 转入 TRACKING 所需连续匹配帧数。
  int max_temp_lost_count{15};  ///< TEMP_LOST 状态允许的最大连续丢失帧数。
  double max_dt_s{0.1};         ///< 接受的相邻帧最大时间间隔，单位为秒。
  double vehicle_initial_radius_m{0.21154};    ///< 普通车辆中心到装甲的初始半径。
  double hero_initial_radius_m{0.23930};       ///< 英雄车辆中心到装甲的初始半径。
  double vehicle_armor_roll_rad{-0.265216};    ///< 普通车辆装甲安装滚转角。
  double hero_armor_roll_rad{-0.256525};       ///< 英雄车辆装甲安装滚转角。
  double min_radius_m{0.05};                   ///< 发散检查接受的最小装甲半径。
  double max_radius_m{0.5};                    ///< 发散检查接受的最大装甲半径。
  double association_max_position_m{0.8};      ///< 观测—槽位关联位置误差门限。
  double association_max_yaw_rad{1.2};         ///< 观测—槽位关联朝向误差门限。
  double linear_acceleration_variance{100.0};  ///< 三轴匀速模型加速度过程噪声方差。
  double angular_acceleration_variance{400.0};  ///< 航向匀速模型角加速度过程噪声方差。
  /** @brief 状态顺序 x,vx,y,vy,z,vz,yaw,vyaw,r,dr,dz 的初始协方差对角线。 */
  std::array<double, 11> initial_covariance_diagonal{1.0, 64.0,  1.0, 64.0, 1.0, 64.0,
                                                     0.4, 100.0, 1.0, 1.0,  1.0};
  /** @brief 方位角、俯仰角、距离和装甲 yaw 的基础量测噪声方差。 */
  std::array<double, 4> measurement_variance_base{0.004, 0.004, 1.0, 0.09};
  double distance_variance_angle_scale{1.0};  ///< 斜视角对距离方差的对数缩放系数。
  double armor_yaw_variance_distance_scale{0.005};  ///< 距离对装甲 yaw 方差的对数缩放系数。
  std::array<int, 5> label_priorities{3, 1, 2, 0, 0};  ///< sentry、one 至 four，值越小越优先。
  std::array<double, 4> prediction_horizons_s{0.0, 0.05, 0.1, 0.2};  ///< 输出预测时域，单位为秒。
};

/**
 * @brief 解析并严格校验四装甲预测器配置。
 * @throws ConfigError Schema 版本错误、字段缺失、未知键或参数范围无效。
 */
[[nodiscard]] ArmorPredictorConfig ParseArmorPredictorConfig(const YAML::Node& root);

}  // namespace mv::modules
