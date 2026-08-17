#pragma once

#include <chrono>
#include <cstdint>

namespace mv::hal {

/** @brief Talos 发布端使用的云台执行器模型。 */
enum class GimbalActuatorMode : std::uint8_t {
  LEGACY = 0,    ///< 未提供执行器模型信息的兼容值，不应作为有效遥测使用。
  PHYSICAL = 1,  ///< 模拟延迟、二阶响应、速度/加速度及机械限位。
  IDEAL = 2,     ///< 忽略动力学约束，直接跟随有效目标角。
};

/**
 * @brief Talos 云台执行器在某个仿真时刻发布的状态快照。
 *
 * 角度采用视觉内部 ROS 约定，单位为弧度；角速度和角加速度分别为 rad/s、rad/s^2。
 * saturation_flags 的 bit 0 至 bit 6 依次表示 yaw 速度、pitch 速度、yaw 加速度、
 * pitch 加速度、pitch 机械限位、命令超时和积分追赶超限。
 */
struct GimbalActuatorTelemetry {
  bool valid{false};  ///< 整个快照是否通过协议版本、模式和有限性校验。
  std::uint64_t state_timestamp_ns{0};  ///< 执行器状态采样的 Unix epoch 纳秒时间。
  std::uint64_t consumed_command_timestamp_ns{0};  ///< 最近消费命令自身携带的时间戳。
  std::uint64_t consumed_at_timestamp_ns{0};  ///< 执行器消费该命令的 Unix epoch 纳秒时间。
  GimbalActuatorMode mode{GimbalActuatorMode::LEGACY};  ///< 当前执行器模型。
  bool command_valid{false};         ///< 当前目标是否来自尚未超时的有效命令。
  std::uint8_t saturation_flags{0};  ///< 当前动力学约束和命令超时状态位。
  double target_yaw{0.0};            ///< 当前目标偏航角，单位为弧度。
  double target_pitch{0.0};          ///< 当前目标俯仰角，单位为弧度。
  double actual_yaw{0.0};            ///< 当前实际偏航角，单位为弧度。
  double actual_pitch{0.0};          ///< 当前实际俯仰角，单位为弧度。
  double yaw_velocity{0.0};          ///< 当前偏航角速度，单位为弧度每秒。
  double pitch_velocity{0.0};        ///< 当前俯仰角速度，单位为弧度每秒。
  double yaw_acceleration{0.0};      ///< 当前偏航角加速度，单位为 rad/s^2。
  double pitch_acceleration{0.0};    ///< 当前俯仰角加速度，单位为 rad/s^2。
};

/** @brief 提供给火控和轨迹规划器的本机单调时钟云台反馈。 */
struct GimbalFeedback {
  bool valid{false};                 ///< 当前估计是否可用于控制计算。
  std::uint64_t source_sequence{0};  ///< 反馈来源对应的 CameraFrame 帧序号。
  std::chrono::steady_clock::time_point timestamp{};  ///< 状态对应的本机单调时刻。
  double yaw{0.0};                                    ///< 偏航角，单位为弧度。
  double yaw_velocity{0.0};    ///< 偏航角速度，单位为弧度每秒。
  double pitch{0.0};           ///< 俯仰角，单位为弧度。
  double pitch_velocity{0.0};  ///< 俯仰角速度，单位为弧度每秒。
};

/**
 * @brief 火控向云台后端提交的一条目标状态和开火建议。
 *
 * valid=false 表示停止外部跟随；此时后端忽略其余运动字段并禁止开火。
 */
struct GimbalCommand {
  bool valid{false};              ///< 是否启用该目标；false 表示发送停止命令。
  bool fire{false};               ///< 有效目标下是否建议开火。
  std::uint64_t timestamp_ns{0};  ///< 命令生成的 Unix epoch 纳秒时间；0 由后端补当前时间。
  double yaw{0.0};                ///< 目标偏航角，单位为弧度。
  double yaw_velocity{0.0};        ///< 目标偏航角速度，单位为弧度每秒。
  double yaw_acceleration{0.0};    ///< 目标偏航角加速度，单位为 rad/s^2。
  double pitch{0.0};               ///< 目标俯仰角，单位为弧度，正方向朝上。
  double pitch_velocity{0.0};      ///< 目标俯仰角速度，单位为弧度每秒。
  double pitch_acceleration{0.0};  ///< 目标俯仰角加速度，单位为 rad/s^2。
  double target_distance_m{-1.0};  ///< 瞄准目标距离，单位为米；有效命令必须非负。
};

}  // namespace mv::hal
