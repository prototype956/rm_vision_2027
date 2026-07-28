#pragma once

#include "modules/armor_detector/armor_detector.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace mv::modules {

/**
 * @brief 解析并校验装甲检测器 YAML 配置。
 *
 * 相对模型路径始终基于 project_root 解析，不受进程当前工作目录或配置文件
 * 所在目录影响。未知字段会被拒绝，避免拼写错误静默使用默认值。
 *
 * @param root 检测器配置文件的根节点。
 * @param project_root 项目根目录，用于解析相对模型路径。
 * @return 已校验且可直接传递给 YoloArmorDetector::Init() 的配置。
 * @throws ConfigError 配置缺少字段、包含未知字段或参数超出允许范围。
 */
[[nodiscard]] ArmorDetectorConfig ParseArmorDetectorConfig(
    const YAML::Node& root, const std::filesystem::path& project_root);

}  // namespace mv::modules
