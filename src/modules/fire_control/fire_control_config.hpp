#pragma once

#include <yaml-cpp/yaml.h>

namespace mv::modules {

/** @brief 火控弹道、时延、装甲选择、不确定性和开火门控参数。 */
struct FireControlConfig {
  bool auto_fire{false};                 ///< 是否允许输出 fire=true 的自动开火脉冲。
  double bullet_speed_mps{25.0};         ///< 弹丸初速度，单位为米每秒。
  double gravity_mps2{9.81};             ///< 弹道模型使用的重力加速度绝对值。
  double command_delay_s{0.015};         ///< 从生成命令到云台开始执行的估计延迟。
  double max_prediction_age_s{0.1};      ///< 接受预测和反馈的最大数据年龄。
  double max_temp_lost_control_s{0.15};  ///< TEMP_LOST 状态仍允许控制外推的最长时间。
  int ballistic_max_iterations{10};      ///< 飞行时间—目标位置定点迭代次数上限。
  double ballistic_time_tolerance_s{1.0e-4};   ///< 判定飞行时间收敛的相邻迭代差。
  double armor_enter_angle_rad{1.0471975512};  ///< 新装甲进入候选集的最大观察角。
  double armor_leave_angle_rad{1.2217304764};  ///< 已锁装甲保持可选的最大观察角。
  double slot_switch_improvement_rad{0.1745329252};  ///< 切换槽位所需的最小观察角改善量。
  double slot_switch_confirmation_s{0.05};  ///< 更优槽位持续满足条件后的切换确认时间。
  double max_center_position_std_m{0.5};  ///< 允许开火的车辆中心位置标准差上限。
  double max_yaw_std_rad{0.5};            ///< 允许开火的目标航向标准差上限。
  double fire_window_scale{0.7};  ///< 将装甲投影角宽缩放为开火窗口的比例。
  double min_fire_yaw_rad{0.0052359878};    ///< 距离自适应偏航窗口下限。
  double max_fire_yaw_rad{0.0349065850};    ///< 距离自适应偏航窗口上限。
  double min_fire_pitch_rad{0.0034906585};  ///< 距离自适应俯仰窗口下限。
  double max_fire_pitch_rad{0.0174532925};  ///< 距离自适应俯仰窗口上限。
  int stable_cycles{3};                     ///< 连续命中开火窗口所需控制周期数。
  double fire_interval_s{0.1};              ///< 两次开火脉冲起点的最小间隔。
  double fire_pulse_width_s{0.03};          ///< 单次 fire=true 脉冲持续时间。
};

/**
 * @brief 解析并严格校验火控 YAML 配置，同时将角度字段由度转换为弧度。
 * @param root 火控配置根节点。
 * @return 可直接构造 FireControl 的类型化配置。
 * @throws ConfigError 字段缺失、类型错误、存在未知键或值域非法。
 */
[[nodiscard]] FireControlConfig ParseFireControlConfig(const YAML::Node& root);

}  // namespace mv::modules
