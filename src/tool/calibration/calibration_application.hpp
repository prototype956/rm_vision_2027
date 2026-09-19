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
   * @brief 仅采集原图与相机元数据，不执行棋盘检测或内参解算。
   * @return 0 表示正常退出，非零值表示初始化或采集失败。
   */
  int Run();

  /**
   * @brief 离线逐张检测 PNG 并弹窗供人工查看，按键继续；全部查看后解算，不打开相机。
   * @param session_dir 含 session.yaml 和 images/ 的采集目录。
   * @return 0 表示验收通过，5 表示验收失败，6 表示取消查看；读取错误抛出异常。
   */
  int SolveOffline(const std::filesystem::path& session_dir);

 private:
  CalibrationSettings settings_;  ///< 棋盘、采集阈值、验收和输出参数。
  YAML::Node camera_config_;      ///< 传递给 MindVisionCamera::Open() 的相机配置。
};

}  // namespace mv::tool::calibration
