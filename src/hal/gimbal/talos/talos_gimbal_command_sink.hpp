#pragma once

#include "hal/gimbal/i_gimbal_command_sink.hpp"

#include <memory>

#include <yaml-cpp/yaml.h>

namespace mv::hal {

/**
 * @brief 通过 Talos v5 共享内存三缓冲发布云台命令并读取执行器遥测。
 *
 * Open() 与 Close() 管理元数据文件映射；析构时会先发布停止命令，再释放映射资源。
 * 命令发布和状态查询由实例内部互斥量串行化。
 */
class TalosGimbalCommandSink final : public IGimbalCommandSink {
 public:
  /** @brief 创建尚未映射 Talos 元数据文件的命令后端。 */
  TalosGimbalCommandSink();
  ~TalosGimbalCommandSink() override;

  TalosGimbalCommandSink(const TalosGimbalCommandSink&) = delete;
  TalosGimbalCommandSink& operator=(const TalosGimbalCommandSink&) = delete;

  /**
   * @brief 从 Talos 相机配置中读取共享内存路径和心跳超时并建立映射。
   * @param camera_config 包含 shared_memory.meta_path 和 timeouts.heartbeat_ms 的配置节点。
   * @return 协议头有效且映射成功时返回 true；重复打开已连接实例时也返回 true。
   */
  bool Open(const YAML::Node& camera_config) noexcept;

  /** @brief 发布停止命令并释放共享内存映射；允许重复调用。 */
  void Close() noexcept;
  bool Send(const GimbalCommand& command) noexcept override;
  [[nodiscard]] bool ExternalControlEnabled() const noexcept override;
  [[nodiscard]] bool IsHealthy() const noexcept override;
  [[nodiscard]] std::uint64_t HeartbeatTimestampNs() const noexcept override;
  [[nodiscard]] GimbalActuatorTelemetry ActuatorTelemetry() const noexcept override;

 private:
  struct Impl;                  ///< 隔离 POSIX 映射、协议视图和并发状态。
  std::unique_ptr<Impl> impl_;  ///< 当前后端唯一拥有的映射资源。
};

}  // namespace mv::hal
