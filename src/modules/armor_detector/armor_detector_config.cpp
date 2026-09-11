#include "modules/armor_detector/armor_detector_config.hpp"

#include "core/config.hpp"
#include "modules/armor_detector/armor_inference_backend.hpp"

#include <string>

namespace mv::modules {
ArmorDetectorConfig ParseArmorDetectorConfig(const YAML::Node& root,
                                             const std::filesystem::path& project_root) {
  constexpr char CONTEXT[] = "armor detector config";
  // 配置采用严格模式，字段扩展时必须同步更新此白名单和 schema_version。
  ConfigLoader::RejectUnknownKeys(root,
                                  {"schema_version", "backend", "model_path", "device",
                                   "enemy_color", "confidence_threshold", "nms_iou_threshold"},
                                  CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) !=
      ARMOR_DETECTOR_CONFIG_SCHEMA_VERSION) {
    throw ConfigError("armor detector config schema_version must be 2");
  }

  // 所有字段均为必填项，默认成员值只服务于直接构造 ArmorDetectorConfig 的调用方。
  ArmorDetectorConfig config;
  const auto BACKEND = ConfigLoader::Require<std::string>(root, "backend", CONTEXT);
  if (BACKEND == "auto")
    config.backend = ArmorInferenceBackend::AUTO;
  else if (BACKEND == "openvino")
    config.backend = ArmorInferenceBackend::OPENVINO;
  else if (BACKEND == "tensorrt")
    config.backend = ArmorInferenceBackend::TENSORRT;
  else
    throw ConfigError("armor detector config.backend must be auto, openvino or tensorrt");
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

  try {
    (void)detail::ParseGpuDeviceIndex(config.device);
  } catch (const ArmorDetectorInitError& error) {
    throw ConfigError(std::string("armor detector config: ") + error.what());
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
  if (!(config.confidence_threshold > 0.0F) || !(config.confidence_threshold < 1.0F)) {
    throw ConfigError("armor detector config.confidence_threshold must be in (0, 1)");
  }
  if (!(config.nms_iou_threshold >= 0.0F) || !(config.nms_iou_threshold <= 1.0F)) {
    throw ConfigError("armor detector config.nms_iou_threshold must be in [0, 1]");
  }
  return config;
}

}  // namespace mv::modules
