#include "core/config.hpp"
#include "core/logger.hpp"
#include "tool/calibration/calibration.hpp"
#include "tool/calibration/calibration_application.hpp"
#include "tool/calibration/handeye_application.hpp"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string_view>

#include <filesystem>

namespace mv::tool::calibration {
namespace {

constexpr char USAGE[] =
    "Usage: mv-camera-calibration <capture | solve <directory> | "
    "extrinsics-capture | extrinsics-solve <directory>>";

int RunCalibration(int argc, char* argv[]) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::puts(USAGE);
    return 0;
  }
  const bool SOLVE = argc == 3 && std::string_view(argv[1]) == "solve";
  const bool EXTRINSICS_CAPTURE = argc == 2 && std::string_view(argv[1]) == "extrinsics-capture";
  const bool EXTRINSICS_SOLVE = argc == 3 && std::string_view(argv[1]) == "extrinsics-solve";
  if (!SOLVE && !EXTRINSICS_CAPTURE && !EXTRINSICS_SOLVE &&
      !(argc == 2 && std::string_view(argv[1]) == "capture")) {
    std::fprintf(stderr, "%s\n", USAGE);
    return 1;
  }
  try {
    const std::filesystem::path PROJECT_ROOT = PROJECT_ROOT_PATH;
    // 外参离线模式只依赖采集会话，不加载当前机器的配置或硬件。
    if (EXTRINSICS_SOLVE)
      return HandeyeApplication::SolveOffline(ConfigLoader::ResolvePath(PROJECT_ROOT, argv[2]));
    const char* config_override = std::getenv("RM_VISION_CONFIG_ROOT");
    const auto CONFIG_ROOT = ConfigLoader::ResolvePath(
        PROJECT_ROOT,
        config_override && config_override[0] != '\0' ? config_override : CONFIG_FILE_PATH);
    Logger::Instance().InitFromFile(CONFIG_ROOT / "core/logger.yaml");
    const auto CAMERA_CONFIG =
        SOLVE ? YAML::Node{} : ConfigLoader::LoadFile(CONFIG_ROOT / "hal/camera/mindvision.yaml");
    const auto CALIBRATION_CONFIG_PATH = CONFIG_ROOT / "tool/camera_calibration.yaml";
    const auto CALIBRATION_CONFIG = ConfigLoader::LoadFile(CALIBRATION_CONFIG_PATH);
    MV_LOG_INFO("Config", "camera calibration config: {}",
                std::filesystem::absolute(CALIBRATION_CONFIG_PATH).lexically_normal().string());
    const auto SETTINGS = ParseCalibrationSettings(CALIBRATION_CONFIG, PROJECT_ROOT);
    if (EXTRINSICS_CAPTURE) {
      return HandeyeApplication::Capture(CONFIG_ROOT, SETTINGS, CAMERA_CONFIG);
    }
    CalibrationApplication application(SETTINGS, CAMERA_CONFIG);
    return SOLVE ? application.SolveOffline(ConfigLoader::ResolvePath(PROJECT_ROOT, argv[2]))
                 : application.Run();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[CameraCalibration] FATAL: %s\n", error.what());
    return 1;
  }
}

}  // namespace
}  // namespace mv::tool::calibration

int main(int argc, char* argv[]) {
  return mv::tool::calibration::RunCalibration(argc, argv);
}
