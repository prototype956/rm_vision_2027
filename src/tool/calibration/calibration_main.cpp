#include "core/config.hpp"
#include "core/logger.hpp"
#include "tool/calibration/calibration.hpp"
#include "tool/calibration/calibration_application.hpp"

#include <cstdio>
#include <exception>

#include <filesystem>

namespace mv::tool::calibration {
namespace {

int RunCalibration() {
  try {
    const std::filesystem::path CONFIG_ROOT = CONFIG_FILE_PATH;
    const std::filesystem::path PROJECT_ROOT = PROJECT_ROOT_PATH;
    Logger::Instance().InitFromFile(CONFIG_ROOT / "core/logger.yaml");
    const auto CAMERA_CONFIG = ConfigLoader::LoadFile(CONFIG_ROOT / "hal/camera/mindvision.yaml");
    const auto CALIBRATION_CONFIG_PATH = CONFIG_ROOT / "tool/camera_calibration.yaml";
    const auto CALIBRATION_CONFIG = ConfigLoader::LoadFile(CALIBRATION_CONFIG_PATH);
    MV_LOG_INFO("Config", "camera calibration config: {}",
                std::filesystem::absolute(CALIBRATION_CONFIG_PATH).lexically_normal().string());
    CalibrationApplication application(ParseCalibrationSettings(CALIBRATION_CONFIG, PROJECT_ROOT),
                                       CAMERA_CONFIG);
    return application.Run();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[CameraCalibration] FATAL: %s\n", error.what());
    return 1;
  }
}

}  // namespace
}  // namespace mv::tool::calibration

int main() {
  return mv::tool::calibration::RunCalibration();
}
