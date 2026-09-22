#pragma once

#include "geometry/gimbal_geometry.hpp"
#include "hal/imu/serial_imu.hpp"

#include <filesystem>
#include <string>

namespace mv::runtime {

/** @brief 启动时选定的车辆安装配置；路径相对配置根目录，运行中不切换坐标系。 */
struct RealGeometryConfig {
  std::string profile;
  hal::SerialImuConfig imu;
  std::filesystem::path intrinsics_path;
  std::filesystem::path extrinsics_path;
  geometry::GimbalExtrinsics extrinsics;
  bool placeholder{true};  ///< 任一安装外参仍为占位时保持 true。
};

/** @brief 选择部署配置并校验安装矩阵；失败时抛出 ConfigError，不回退到其他车辆。 */
[[nodiscard]] RealGeometryConfig LoadRealGeometryConfig(const std::filesystem::path& config_root);

}  // namespace mv::runtime
