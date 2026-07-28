#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/i_camera.hpp"
#include "hal/camera/mindvision/mindvision_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_detector/armor_detector_config.hpp"
#include "test/armor_detector/armor_detector_test_application.hpp"

#include <cstdio>
#include <exception>
#include <memory>
#include <string>

#include <filesystem>

namespace mv::test {
namespace {

/**
 * @brief 将装甲检测实机测试 YAML 转换为运行参数，并完成字段和值域校验。
 *
 * output_dir 相对于测试配置文件所在目录解析，避免程序启动目录影响产物位置。
 *
 * @param root 测试配置文件的根节点。
 * @param config_path 测试配置文件路径，用于解析相对输出目录。
 * @return 已校验且可直接交给 ArmorDetectorTestApplication 的运行参数。
 * @throws ConfigError 配置缺少字段、包含未知字段或参数超出允许范围。
 */
ArmorDetectorTestSettings ParseTestSettings(const YAML::Node& root,
                                            const std::filesystem::path& config_path) {
  constexpr char CONTEXT[] = "armor detector test config";
  ConfigLoader::RejectUnknownKeys(root,
                                  {"schema_version", "duration_sec", "warmup_sec", "preview",
                                   "report_interval_sec", "save_sample_interval_sec", "output_dir"},
                                  CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1) {
    throw ConfigError("armor detector test config schema_version must be 1");
  }

  ArmorDetectorTestSettings settings;
  settings.duration_sec = ConfigLoader::Require<int>(root, "duration_sec", CONTEXT);
  settings.warmup_sec = ConfigLoader::Require<int>(root, "warmup_sec", CONTEXT);
  settings.preview = ConfigLoader::Require<bool>(root, "preview", CONTEXT);
  settings.report_interval_sec = ConfigLoader::Require<int>(root, "report_interval_sec", CONTEXT);
  settings.save_sample_interval_sec =
      ConfigLoader::Require<int>(root, "save_sample_interval_sec", CONTEXT);
  const auto OUTPUT_DIR = ConfigLoader::Require<std::string>(root, "output_dir", CONTEXT);
  settings.output_dir = ConfigLoader::ResolvePath(config_path.parent_path(), OUTPUT_DIR);

  if (settings.duration_sec <= 0 || settings.warmup_sec < 0 ||
      settings.warmup_sec >= settings.duration_sec || settings.report_interval_sec <= 0 ||
      settings.save_sample_interval_sec < 0 || OUTPUT_DIR.empty()) {
    throw ConfigError("armor detector test config contains an invalid setting");
  }
  return settings;
}

/**
 * @brief 初始化检测器和相机，并执行装甲检测实机长时验收。
 *
 * 启动函数只负责配置加载和依赖装配；连续抓帧、检测、指标统计及报告输出均由
 * ArmorDetectorTestApplication::Run() 负责。
 *
 * @return 可直接作为进程退出状态使用的结果码。
 * @retval 0 测试通过。
 * @retval 1 配置加载或运行期间抛出标准异常。
 * @retval 2 检测器初始化失败。
 * @retval 3 相机打开失败。
 * @retval 4 相机输出格式不符合 1280x720 BGR8 要求。
 * @retval 5 长时验收未达到指标或被提前终止。
 */
int RunArmorDetectorTest() {
  try {
    // 四类配置分别控制日志、检测器、相机 HAL 和实机测试策略。
    const auto CONFIG_ROOT = std::filesystem::path(CONFIG_FILE_PATH);
    const auto PROJECT_ROOT = std::filesystem::path(PROJECT_ROOT_PATH);
    const auto DETECTOR_PATH = CONFIG_ROOT / "modules/armor_detector.yaml";
    const auto CAMERA_PATH = CONFIG_ROOT / "hal/camera/mindvision.yaml";
    const auto TEST_PATH = CONFIG_ROOT / "test/armor_detector_test.yaml";

    Logger::Instance().InitFromFile(CONFIG_ROOT / "core/logger.yaml");

    // 检测器必须先完成模型契约和 GPU 设备校验，再交给测试应用使用。
    std::unique_ptr<modules::YoloArmorDetector> detector =
        std::make_unique<modules::YoloArmorDetector>();
    try {
      const auto DETECTOR_YAML = ConfigLoader::LoadFile(DETECTOR_PATH);
      detector->Init(modules::ParseArmorDetectorConfig(DETECTOR_YAML, PROJECT_ROOT));
    } catch (const std::exception& error) {
      MV_LOG_ERROR("ArmorDetectorTest", "detector initialization failed: {}", error.what());
      return 2;
    }

    const auto CAMERA_CONFIG = ConfigLoader::LoadFile(CAMERA_PATH);
    const auto TEST_CONFIG = ConfigLoader::LoadFile(TEST_PATH);
    MV_LOG_INFO("Config", "armor detector config: {}", DETECTOR_PATH.string());
    MV_LOG_INFO("Config", "camera config: {}", CAMERA_PATH.string());
    MV_LOG_INFO("Config", "armor detector test config: {}", TEST_PATH.string());

    // 通过 ICamera 注入 MindVision 后端，使测试流程只依赖统一相机接口。
    std::unique_ptr<hal::ICamera> camera = std::make_unique<hal::MindVisionCamera>();
    ArmorDetectorTestApplication application(std::move(camera), std::move(detector), CAMERA_CONFIG,
                                             ParseTestSettings(TEST_CONFIG, TEST_PATH));
    return application.Run();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[ArmorDetectorTest] FATAL: %s\n", error.what());
    return 1;
  }
}

}  // namespace
}  // namespace mv::test

int main() {
  return mv::test::RunArmorDetectorTest();
}
