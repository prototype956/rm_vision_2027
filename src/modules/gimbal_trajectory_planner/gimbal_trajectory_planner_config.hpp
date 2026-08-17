#pragma once

#include <array>

#include <yaml-cpp/yaml.h>

namespace mv::modules {

/** @brief 双轴云台 TinyMPC 离散模型、约束、代价和 ADMM 求解参数。 */
struct GimbalTrajectoryPlannerConfig {
  double dt_s{0.01};                 ///< 相邻离散状态之间的时间间隔，单位为秒。
  int horizon_steps{50};             ///< 每轴状态参考和输出轨迹的离散点数。
  double command_lookahead_s{0.05};  ///< 从轨迹起点选取正式命令的期望前视时间。
  double normalization_angle_scale_rad{0.1};    ///< 角度状态归一化尺度。
  double max_yaw_velocity_rad_s{3.0};           ///< 偏航角速度绝对值约束。
  double max_pitch_velocity_rad_s{3.0};         ///< 俯仰角速度绝对值约束。
  double max_yaw_acceleration_rad_s2{50.0};     ///< 偏航控制输入绝对值约束。
  double max_pitch_acceleration_rad_s2{100.0};  ///< 俯仰控制输入绝对值约束。
  std::array<double, 2> q_yaw{9.0e6, 0.0};      ///< 偏航角度、角速度状态误差权重。
  std::array<double, 2> q_pitch{9.0e6, 0.0};    ///< 俯仰角度、角速度状态误差权重。
  double r_yaw{1.0};                            ///< 偏航角加速度输入代价权重。
  double r_pitch{1.0};                          ///< 俯仰角加速度输入代价权重。
  int max_iterations{50};                       ///< 每次 TinyMPC ADMM 求解迭代上限。
  double rho{1.0};                              ///< TinyMPC ADMM 罚参数。
  double absolute_primal_tolerance{5.0e-2};     ///< 归一化空间的绝对原始残差容差。
  double absolute_dual_tolerance{5.0e-2};       ///< 归一化空间的绝对对偶残差容差。
};

/**
 * @brief 解析并严格校验云台轨迹规划器 YAML 配置。
 * @param root 轨迹规划器配置根节点。
 * @return 可直接构造 GimbalTrajectoryPlanner 的类型化配置。
 * @throws ConfigError 字段缺失、类型错误、存在未知键或值域非法。
 */
[[nodiscard]] GimbalTrajectoryPlannerConfig ParseGimbalTrajectoryPlannerConfig(
    const YAML::Node& root);

}  // namespace mv::modules
