#pragma once

#include "tool/calibration/calibration.hpp"

namespace mv::tool::calibration {
/** @brief 单 yaw 外参交互流程，采集与离线求解严格分离。 */
class HandeyeApplication final {
 public:
  /** @brief 读取选中车辆配置，手动保存同帧图像—姿态；退出返回 0，中断会话返回 4。 */
  static int Capture(const std::filesystem::path& config_root,
                     const CalibrationSettings& settings, const YAML::Node& camera_config);
  /** @brief 仅依赖会话快照；0 为数值求解成功，5 为失败，6 为用户取消审核。 */
  static int SolveOffline(const std::filesystem::path& session_dir);
};
}  // namespace mv::tool::calibration
