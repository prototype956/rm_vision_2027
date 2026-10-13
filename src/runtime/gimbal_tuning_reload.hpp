#pragma once

#include "hal/serial/serial_gimbal_command_sink.hpp"
#include "modules/gimbal_control/gimbal_control_config.hpp"

#include <mutex>
#include <thread>

#include <filesystem>
#include <optional>

namespace mv::runtime {

/** @brief 一组完整的实机控制与传输参数，不包含任何串口资源。 */
struct GimbalTuningSnapshot {
  bool operator==(const GimbalTuningSnapshot&) const = default;
  modules::GimbalControlConfig control;
  hal::SerialGimbalCommandSinkConfig transport;
};

/** @brief 后台读取稳定文件快照，校验不可热更新字段后发布 latest-wins 参数。 */
class GimbalTuningReload final {
 public:
  GimbalTuningReload(std::filesystem::path config_root, GimbalTuningSnapshot startup);
  [[nodiscard]] std::optional<GimbalTuningSnapshot> TakePending();

 private:
  void Run(const std::stop_token& stop);
  std::filesystem::path root_;
  GimbalTuningSnapshot startup_;
  std::mutex mutex_;
  std::optional<GimbalTuningSnapshot> pending_;
  std::jthread worker_;  ///< 最先销毁，先停止读取再释放邮箱。
};

}  // namespace mv::runtime
