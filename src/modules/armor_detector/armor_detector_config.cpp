#include "modules/armor_detector/armor_detector_config.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <string>
#include <string_view>

namespace mv::modules {
namespace {

// 只接受 GPU 或 GPU.<非负整数>，显式禁止 CPU、AUTO 和 MULTI 回退。
bool IsExplicitGpuDevice(const std::string& device) {
  if (device == "GPU") {
    return true;
  }
  constexpr std::string_view PREFIX = "GPU.";
  if (!device.starts_with(PREFIX) || device.size() == PREFIX.size()) {
    return false;
  }
  return std::all_of(device.begin() + static_cast<std::ptrdiff_t>(PREFIX.size()), device.end(),
                     [](unsigned char character) { return std::isdigit(character) != 0; });
}

}  // namespace

ArmorDetectorConfig ParseArmorDetectorConfig(const YAML::Node& root,
                                             const std::filesystem::path& project_root) {
  constexpr char CONTEXT[] = "armor detector config";
  // 配置采用严格模式，字段扩展时必须同步更新此白名单和 schema_version。
  ConfigLoader::RejectUnknownKeys(root,
                                  {"schema_version", "model_path", "device", "enemy_color",
                                   "confidence_threshold", "nms_iou_threshold"},
                                  CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1) {
    throw ConfigError("armor detector config schema_version must be 1");
  }

  // 所有字段均为必填项，默认成员值只服务于直接构造 ArmorDetectorConfig 的调用方。
  ArmorDetectorConfig config;
  const auto MODEL_PATH = ConfigLoader::Require<std::string>(root, "model_path", CONTEXT);
  config.device = ConfigLoader::Require<std::string>(root, "device", CONTEXT);
  const auto ENEMY_COLOR = ConfigLoader::Require<std::string>(root, "enemy_color", CONTEXT);
  config.confidence_threshold = ConfigLoader::Require<float>(root, "confidence_threshold", CONTEXT);
  config.nms_iou_threshold = ConfigLoader::Require<float>(root, "nms_iou_threshold", CONTEXT);

  if (MODEL_PATH.empty()) {
    throw ConfigError("armor detector config.model_path must not be empty");
  }
  // 模型路径以项目根目录为基准，保证从 build/bin 等目录启动时行为一致。
  config.model_path = ConfigLoader::ResolvePath(project_root, MODEL_PATH);

  if (!IsExplicitGpuDevice(config.device)) {
    throw ConfigError(
        "armor detector config.device must be GPU or GPU.<index> (CPU fallback is disabled)");
  }
  // YAML 使用小写稳定字符串，内部转换为强类型枚举。
  if (ENEMY_COLOR == "red") {
    config.enemy_color = ArmorColor::RED;
  } else if (ENEMY_COLOR == "blue") {
    config.enemy_color = ArmorColor::BLUE;
  } else {
    throw ConfigError("armor detector config.enemy_color must be red or blue");
  }

  // objectness 不接受 0 和 1；NMS 的 0/1 分别表示抑制任意重叠/不抑制正常重叠。
  if (!(config.confidence_threshold > 0.0F && config.confidence_threshold < 1.0F)) {
    throw ConfigError("armor detector config.confidence_threshold must be in (0, 1)");
  }
  if (!(config.nms_iou_threshold >= 0.0F && config.nms_iou_threshold <= 1.0F)) {
    throw ConfigError("armor detector config.nms_iou_threshold must be in [0, 1]");
  }
  return config;
}

}  // namespace mv::modules
