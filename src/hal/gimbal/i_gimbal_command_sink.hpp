#pragma once

#include "hal/gimbal/gimbal_types.hpp"

namespace mv::hal {

/**
 * @brief 云台控制命令下沉与执行器状态查询的统一接口。
 *
 * 后端不拥有调用方传入的命令。实现应在 Send() 返回前完成复制或发布，并保证所有
 * noexcept 接口将协议、连接和输入错误转换为 false 或无效遥测。
 */
class IGimbalCommandSink {
 public:
  virtual ~IGimbalCommandSink() = default;

  /**
   * @brief 发布一条云台目标或停止命令。
   * @param command 待发布的不可变命令；valid=false 表示停止外部跟随。
   * @return 命令成功交给后端时返回 true，未连接或协议状态无效时返回 false。
   */
  virtual bool Send(const GimbalCommand& command) noexcept = 0;

  /** @brief 检查数据源是否已启用并正在执行外部云台控制。 */
  [[nodiscard]] virtual bool ExternalControlEnabled() const noexcept = 0;

  /** @brief 检查命令通道已连接且数据源心跳未超时。 */
  [[nodiscard]] virtual bool IsHealthy() const noexcept = 0;

  /** @brief 返回数据源最近一次心跳的 Unix epoch 纳秒时间；不可用时返回 0。 */
  [[nodiscard]] virtual std::uint64_t HeartbeatTimestampNs() const noexcept = 0;

  /** @brief 返回最近的执行器状态快照；不可用或校验失败时 valid=false。 */
  [[nodiscard]] virtual GimbalActuatorTelemetry ActuatorTelemetry() const noexcept = 0;
};

}  // namespace mv::hal
