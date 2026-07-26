#pragma once

#include <yaml-cpp/yaml.h>

namespace mv::hal::detail {

/**
 * @brief 经过完整校验的 MindVision 相机配置。
 */
struct MindVisionConfig {
  int device_index{0};
  int width{1280};
  int height{720};
  bool centered_roi{true};
  bool auto_exposure{false};
  int exposure_us{5000};
  int grab_timeout_ms{100};
};

/**
 * @brief 解析并校验 MindVision 相机 YAML 配置。
 *
 * @param root 相机配置根节点。
 * @return 可直接交给 SDK 设备层的类型化配置。
 * @throws ConfigError 配置字段缺失、类型错误或值域非法。
 */
MindVisionConfig ParseMindVisionConfig(const YAML::Node& root);

}  // namespace mv::hal::detail
