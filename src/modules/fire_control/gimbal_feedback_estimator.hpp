#pragma once

#include "geometry/rigid_transform.hpp"
#include "hal/gimbal/gimbal_types.hpp"

#include <limits>

#include <optional>

namespace mv::modules {

/** @brief 当前云台反馈状态采用的数据来源或保持策略。 */
enum class GimbalFeedbackSource {
  NONE,                     ///< 尚无可用反馈。
  MEASUREMENT_INIT,         ///< 使用首个相机位姿量测初始化。
  PUBLISHED_COMMAND,        ///< 无物理遥测时投影最近成功发布的运动命令。
  HELD_COMMAND,             ///< 无物理遥测时保持上一条命令角度并将速度置零。
  CAMERA_FALLBACK,          ///< 物理遥测失效后回退到最近相机位姿量测。
  ACTUATOR_RUNTIME_DIRECT,  ///< 采用一份新的 Talos 物理执行器运行时遥测。
  ACTUATOR_RUNTIME_HOLD,    ///< Talos 尚无新遥测，保持最近的物理执行器状态。
};

/** @brief 将反馈来源转换为稳定的日志与 Foxglove 字段名称。 */
[[nodiscard]] const char* GimbalFeedbackSourceName(GimbalFeedbackSource source) noexcept;

/**
 * @brief 融合相机同帧位姿、Talos 物理执行器遥测和已发布命令的云台反馈估计器。
 *
 * 新鲜的 PHYSICAL 执行器遥测优先级最高；其不可用时依次使用命令投影或相机量测。
 * 所有输出时间戳均转换到本机 steady_clock，偏航角会相对上一状态连续展开。
 */
class GimbalFeedbackEstimator final {
 public:
  /** @brief 使用允许的偏航和俯仰角速度绝对值上限创建空估计器。 */
  GimbalFeedbackEstimator(double max_yaw_velocity_rad_s, double max_pitch_velocity_rad_s);

  /** @brief 接收异步 Talos 执行器遥测，校验新鲜度并更新最高优先级反馈。 */
  void ObserveActuatorTelemetry(const hal::GimbalActuatorTelemetry& actuator,
                                std::chrono::steady_clock::time_point now,
                                std::uint64_t system_now_ns) noexcept;
  /** @brief 从同帧 world_t_gimbal 提取角度，并以相邻帧差分或执行器遥测估计速度。 */
  void ObserveMeasurement(std::uint64_t sequence, std::chrono::steady_clock::time_point timestamp,
                          const geometry::RigidTransform& world_t_gimbal,
                          const std::optional<hal::GimbalActuatorTelemetry>& actuator) noexcept;
  /** @brief 在没有物理执行器遥测时，将成功发布的命令投影为当前反馈。 */
  void ObservePublishedCommand(const hal::GimbalCommand& command,
                               std::chrono::steady_clock::time_point timestamp,
                               bool held_command) noexcept;
  /** @brief 返回当前反馈快照；now 为预留的外推时刻，当前实现不继续外推。 */
  [[nodiscard]] hal::GimbalFeedback Estimate(
      std::chrono::steady_clock::time_point now) const noexcept;
  /** @brief 返回最近一份相机位姿直接量测，不受命令投影和运行时遥测覆盖。 */
  [[nodiscard]] const hal::GimbalFeedback& LastMeasurement() const noexcept { return measurement_; }
  /** @brief 返回当前反馈来源。 */
  [[nodiscard]] GimbalFeedbackSource Source() const noexcept { return source_; }
  /** @brief 检查当前是否正在使用有效的 PHYSICAL 执行器遥测。 */
  [[nodiscard]] bool RuntimeActuatorActive() const noexcept { return runtime_actuator_active_; }
  /** @brief 返回最近运行时遥测相对当前系统时刻的数据年龄，单位为秒。 */
  [[nodiscard]] double RuntimeActuatorAgeS() const noexcept { return runtime_actuator_age_s_; }
  /** @brief 返回最近采用的运行时状态 Unix epoch 纳秒时间戳。 */
  [[nodiscard]] std::uint64_t RuntimeStateTimestampNs() const noexcept {
    return runtime_state_timestamp_ns_;
  }
  /** @brief 返回命令投影时间；当前估计器不执行连续外推，恒为 0。 */
  [[nodiscard]] double ProjectionDtS() const noexcept { return 0.0; }
  /** @brief 停止使用已发布命令投影，并回退到运行时遥测或相机量测。 */
  void ClearCommandProjection() noexcept;
  /** @brief 丢弃运行时执行器状态，并回退到最近相机量测。 */
  void ClearRuntimeActuator() noexcept;
  /** @brief 清空全部量测、投影、运行时状态和来源标记。 */
  void Reset() noexcept;

 private:
  double max_yaw_velocity_rad_s_;    ///< 差分和遥测偏航速度钳位绝对值。
  double max_pitch_velocity_rad_s_;  ///< 差分和遥测俯仰速度钳位绝对值。
  hal::GimbalFeedback state_;        ///< Estimate() 返回的当前融合状态。
  hal::GimbalFeedback measurement_;  ///< 最近相机位姿直接量测。
  std::chrono::steady_clock::time_point last_observation_time_{};  ///< 最近相机量测时刻。
  double last_measured_yaw_{0.0};    ///< 用于连续展开和差分的上一偏航角。
  double last_measured_pitch_{0.0};  ///< 用于差分的上一俯仰角。
  GimbalFeedbackSource source_{GimbalFeedbackSource::NONE};  ///< 当前融合状态来源。
  bool command_projection_active_{false};        ///< state_ 是否来自已发布命令。
  bool runtime_actuator_active_{false};          ///< state_ 是否由物理执行器遥测主导。
  std::uint64_t runtime_state_timestamp_ns_{0};  ///< 最近采用的执行器状态时间戳。
  double runtime_actuator_age_s_{std::numeric_limits<double>::infinity()};  ///< 遥测年龄。
};

}  // namespace mv::modules
