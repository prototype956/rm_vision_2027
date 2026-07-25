#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/i_camera.hpp"
#include "hal/camera/mindvision_camera.hpp"
#include "test/camera_test_application.hpp"

#include <cstdio>
#include <exception>
#include <memory>
#include <string>

#include <filesystem>

namespace mv::test {
namespace {

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

  const auto output_dir =
      ConfigLoader::Require<std::string>(root, "output_dir", "camera test config");
  settings.output_dir = ConfigLoader::ResolvePath(config_path.parent_path(), output_dir);

  if (settings.duration_sec <= 0 || settings.warmup_sec < 0 ||
      settings.warmup_sec >= settings.duration_sec || settings.report_interval_sec <= 0 ||
      settings.save_sample_interval_sec < 0 || settings.restart_cycles < 0 ||
      settings.frames_per_restart_cycle <= 0 || output_dir.empty()) {
    throw ConfigError("camera test config contains an invalid setting");
  }
  return settings;
}

int RunCameraTest() {
  try {
    const auto config_root = std::filesystem::path(CONFIG_FILE_PATH);
    const auto logger_path = config_root / "core/logger.yaml";
    const auto camera_path = config_root / "hal/camera/mindvision.yaml";
    const auto test_path = config_root / "apps/camera_test.yaml";

    Logger::Instance().InitFromFile(logger_path);
    const auto camera_config = ConfigLoader::LoadFile(camera_path);
    const auto test_config = ConfigLoader::LoadFile(test_path);
    MV_LOG_INFO("Config", "camera config: {}",
                std::filesystem::absolute(camera_path).lexically_normal().string());
    MV_LOG_INFO("Config", "camera test config: {}",
                std::filesystem::absolute(test_path).lexically_normal().string());

    std::unique_ptr<hal::ICamera> camera = std::make_unique<hal::MindVisionCamera>();
    CameraTestApplication application(std::move(camera), camera_config,
                                      ParseTestSettings(test_config, test_path));
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
