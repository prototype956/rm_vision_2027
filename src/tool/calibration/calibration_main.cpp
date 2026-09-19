#include "core/config.hpp"
#include "core/logger.hpp"
#include "tool/calibration/calibration.hpp"
#include "tool/calibration/calibration_application.hpp"

#include <cstdio>
#include <exception>
#include <filesystem>
#include <string_view>

namespace mv::tool::calibration {
namespace {

int RunCalibration(int argc, char* argv[]) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::puts("Usage: mv-camera-calibration [capture | solve <capture-directory>]");
    return 0;
  }
  const bool SOLVE = argc == 3 && std::string_view(argv[1]) == "solve";
  if (!SOLVE && !(argc == 1 || (argc == 2 && std::string_view(argv[1]) == "capture"))) {
    std::fprintf(stderr, "Usage: mv-camera-calibration [capture | solve <capture-directory>]\n");
    return 1;
  }
  try {
    const std::filesystem::path CONFIG_ROOT = CONFIG_FILE_PATH;
    const std::filesystem::path PROJECT_ROOT = PROJECT_ROOT_PATH;
    Logger::Instance().InitFromFile(CONFIG_ROOT / "core/logger.yaml");
    const auto CAMERA_CONFIG =
        SOLVE ? YAML::Node{} : ConfigLoader::LoadFile(CONFIG_ROOT / "hal/camera/mindvision.yaml");
    const auto CALIBRATION_CONFIG_PATH = CONFIG_ROOT / "tool/camera_calibration.yaml";
    const auto CALIBRATION_CONFIG = ConfigLoader::LoadFile(CALIBRATION_CONFIG_PATH);
    MV_LOG_INFO("Config", "camera calibration config: {}",
                std::filesystem::absolute(CALIBRATION_CONFIG_PATH).lexically_normal().string());
    CalibrationApplication application(ParseCalibrationSettings(CALIBRATION_CONFIG, PROJECT_ROOT),
                                       CAMERA_CONFIG);
    return SOLVE ? application.SolveOffline(argv[2]) : application.Run();
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
