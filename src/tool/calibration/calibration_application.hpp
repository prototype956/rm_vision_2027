#pragma once

#include "tool/calibration/calibration.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace mv::tool::calibration {

/**
 * @brief 驱动 MindVision 相机、交互窗口和标定会话持久化的前端应用。
 */
class CalibrationApplication final {
 public:
  /** @brief 保存已经校验的标定参数及相机 YAML 配置。 */
  CalibrationApplication(CalibrationSettings settings, const YAML::Node& camera_config);

  /**
   * @brief 运行相机采集与键盘交互循环，退出前保存最终会话状态。
   * @return 0 表示正常退出，非零值表示初始化或采集失败。
   */
  int Run();

 private:
  CalibrationSettings settings_;  ///< 棋盘、采集阈值、验收和输出参数。
  YAML::Node camera_config_;      ///< 传递给 MindVisionCamera::Open() 的相机配置。
};

}  // namespace mv::tool::calibration
