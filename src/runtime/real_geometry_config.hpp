#pragma once

#include "geometry/gimbal_geometry.hpp"
#include "hal/imu/serial_imu.hpp"

#include <string>

#include <filesystem>

namespace mv::runtime {

/** @brief 启动时选定的车辆安装配置；路径相对配置根目录，运行中不切换坐标系。 */
struct RealGeometryConfig {
  std::string profile;
  hal::SerialImuConfig imu;
  std::filesystem::path imu_config_path;  ///< 已选择的串口配置，供采集会话冻结。
  std::filesystem::path intrinsics_path;
  std::filesystem::path extrinsics_path;
  geometry::GimbalExtrinsics extrinsics;
  YAML::Node extrinsics_snapshot;  ///< 原始已校验外参，供标定会话冻结及保留枪口字段。
  bool placeholder{true};  ///< 任一安装外参仍为占位时保持 true。
};

/** @brief 解析配置或会话快照中的固定安装外参，校验三个旋转与平移。 */
[[nodiscard]] geometry::GimbalExtrinsics ParseGimbalExtrinsics(const YAML::Node& root);

/** @brief 选择部署配置并校验安装矩阵；失败时抛出 ConfigError，不回退到其他车辆。 */
[[nodiscard]] RealGeometryConfig LoadRealGeometryConfig(const std::filesystem::path& config_root);

}  // namespace mv::runtime
