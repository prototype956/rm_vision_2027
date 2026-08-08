#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

namespace mv::hal::detail {

/**
 * @brief 经过完整校验的 Talos 共享内存相机配置。
 */
struct TalosConfig {
  std::string meta_path;        ///< 控制信息和帧元数据文件。
  std::string image_pool_path;  ///< 三缓冲图像像素文件。
  int expected_width{0};        ///< 启动时要求发布端提供的宽度。
  int expected_height{0};       ///< 启动时要求发布端提供的高度。
  int connect_timeout_ms{0};    ///< Open() 等待发布端就绪的最长时间。
  int grab_timeout_ms{0};       ///< Grab() 等待新帧的最长时间。
  int heartbeat_timeout_ms{0};  ///< 判定发布端失联的心跳最大间隔。
};

/**
 * @brief 解析并校验 Talos 相机 YAML 配置。
 *
 * @param root 相机配置根节点。
 * @return 可直接交给 Talos 设备层的类型化配置。
 * @throws ConfigError 配置字段缺失、类型错误、版本不支持或值域非法。
 */
TalosConfig ParseTalosConfig(const YAML::Node& root);

}  // namespace mv::hal::detail
