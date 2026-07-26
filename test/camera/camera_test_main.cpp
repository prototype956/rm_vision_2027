#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/i_camera.hpp"
#include "hal/camera/mindvision/mindvision_camera.hpp"
#include "test/camera/camera_test_application.hpp"

#include <cstdio>
#include <exception>
#include <memory>
#include <string>

#include <filesystem>

namespace mv::test {
namespace {

/**
 * @brief 将相机测试 YAML 转换为运行参数，并完成字段和值域校验。
 *
 * output_dir 相对于测试配置文件所在目录解析，避免程序启动目录影响产物位置。
 *
 * @param root 测试配置文件的根节点。
 * @param config_path 测试配置文件路径，用于解析相对输出目录。
 * @return 已校验且可直接交给 CameraTestApplication 的运行参数。
 * @throws ConfigError 配置缺少字段、包含未知字段或参数超出允许范围。
 */
CameraTestSettings ParseTestSettings(const YAML::Node& root,
                                     const std::filesystem::path& config_path) {
  ConfigLoader::RejectUnknownKeys(
      root,
      {"schema_version", "duration_sec", "warmup_sec", "preview", "report_interval_sec",
       "save_sample_interval_sec", "restart_cycles", "frames_per_restart_cycle", "output_dir"},
      "camera test config");

  CameraTestSettings settings;
  settings.duration_sec = ConfigLoader::Require<int>(root, "duration_sec", "camera test config");
  settings.warmup_sec = ConfigLoader::Require<int>(root, "warmup_sec", "camera test config");
  settings.preview = ConfigLoader::Require<bool>(root, "preview", "camera test config");
  settings.report_interval_sec =
      ConfigLoader::Require<int>(root, "report_interval_sec", "camera test config");
  settings.save_sample_interval_sec =
      ConfigLoader::Require<int>(root, "save_sample_interval_sec", "camera test config");
  settings.restart_cycles =
      ConfigLoader::Require<int>(root, "restart_cycles", "camera test config");
  settings.frames_per_restart_cycle =
      ConfigLoader::Require<int>(root, "frames_per_restart_cycle", "camera test config");

  const auto OUTPUT_DIR =
      ConfigLoader::Require<std::string>(root, "output_dir", "camera test config");
  settings.output_dir = ConfigLoader::ResolvePath(config_path.parent_path(), OUTPUT_DIR);

  if (settings.duration_sec <= 0 || settings.warmup_sec < 0 ||
      settings.warmup_sec >= settings.duration_sec || settings.report_interval_sec <= 0 ||
      settings.save_sample_interval_sec < 0 || settings.restart_cycles < 0 ||
      settings.frames_per_restart_cycle <= 0 || OUTPUT_DIR.empty()) {
    throw ConfigError("camera test config contains an invalid setting");
  }
  return settings;
}

/**
 * @brief 初始化相机测试所需依赖并执行完整测试流程。
 *
 * 启动函数只负责配置和对象装配；重复启停、连续抓帧、指标统计及报告输出均由
 * CameraTestApplication::Run() 负责。
 *
 * @return 可直接作为进程退出状态使用的结果码。
 * @retval 0 测试通过。
 * @retval 1 配置加载或运行期间抛出标准异常。
 * @retval 2 相机打开失败。
 * @retval 3 相机输出格式不符合 1280x720 BGR8 要求。
 * @retval 4 长时稳定性测试未达到验收指标。
 * @retval 5 重复启停测试未达到验收指标。
 */
int RunCameraTest() {
  try {
    // 三类配置分别控制日志、相机 HAL 和测试策略。
    const auto CONFIG_ROOT = std::filesystem::path(CONFIG_FILE_PATH);
    const auto LOGGER_PATH = CONFIG_ROOT / "core/logger.yaml";
    const auto CAMERA_PATH = CONFIG_ROOT / "hal/camera/mindvision.yaml";
    const auto TEST_PATH = CONFIG_ROOT / "test/camera_test.yaml";

    Logger::Instance().InitFromFile(LOGGER_PATH);
    const auto CAMERA_CONFIG = ConfigLoader::LoadFile(CAMERA_PATH);
    const auto TEST_CONFIG = ConfigLoader::LoadFile(TEST_PATH);
    MV_LOG_INFO("Config", "camera config: {}",
                std::filesystem::absolute(CAMERA_PATH).lexically_normal().string());
    MV_LOG_INFO("Config", "camera test config: {}",
                std::filesystem::absolute(TEST_PATH).lexically_normal().string());

    // 通过 ICamera 注入具体后端，使测试流程只依赖统一的相机接口。
    std::unique_ptr<hal::ICamera> camera = std::make_unique<hal::MindVisionCamera>();
    CameraTestApplication application(std::move(camera), CAMERA_CONFIG,
                                      ParseTestSettings(TEST_CONFIG, TEST_PATH));
    return application.Run();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[CameraTest] FATAL: %s\n", error.what());
    return 1;
  }
}

}  // namespace
}  // namespace mv::test

int main() {
  return mv::test::RunCameraTest();
}
