#include "core/config.hpp"
#include "core/logger.hpp"
#include "tool/calibration/extrinsic_capture_application.hpp"
#include "tool/calibration/extrinsic_solve_application.hpp"
#include "tool/calibration/extrinsic_visualize_application.hpp"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string_view>

#include <filesystem>

namespace mv::tool::calibration {
namespace {

int RunExtrinsicCalibration(int argc, char* argv[]) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::puts("Usage: mv-extrinsic-calibration capture");
    std::puts("       mv-extrinsic-calibration solve <session-directory>");
    std::puts("       mv-extrinsic-calibration visualize <solution-directory> [--once]");
    return 0;
  }
  const bool SOLVE = argc == 3 && std::string_view(argv[1]) == "solve";
  const bool VISUALIZE = (argc == 3 || argc == 4) && std::string_view(argv[1]) == "visualize" &&
                         (argc == 3 || std::string_view(argv[3]) == "--once");
  if (!((argc == 2 && std::string_view(argv[1]) == "capture") || SOLVE || VISUALIZE)) {
    std::fprintf(stderr,
                 "Usage: mv-extrinsic-calibration capture\n"
                 "       mv-extrinsic-calibration solve <session-directory>\n"
                 "       mv-extrinsic-calibration visualize <solution-directory> [--once]\n");
    return 1;
  }

  try {
    const std::filesystem::path PROJECT_ROOT = PROJECT_ROOT_PATH;
    const char* config_override = std::getenv("RM_VISION_CONFIG_ROOT");
    const auto CONFIG_ROOT = ConfigLoader::ResolvePath(
        PROJECT_ROOT,
        config_override && config_override[0] != '\0' ? config_override : CONFIG_FILE_PATH);
    Logger::Instance().InitFromFile(CONFIG_ROOT / "core/logger.yaml");

    const auto TOOL_CONFIG_PATH = CONFIG_ROOT / "tool/extrinsic_calibration.yaml";
    const auto TOOL_CONFIG = ConfigLoader::LoadFile(TOOL_CONFIG_PATH);
    MV_LOG_INFO("Config", "extrinsic calibration config: {}",
                std::filesystem::absolute(TOOL_CONFIG_PATH).lexically_normal().string());
    const auto SETTINGS = ParseExtrinsicCaptureSettings(TOOL_CONFIG, PROJECT_ROOT);
    if (SOLVE) {
      ExtrinsicSolveApplication application(SETTINGS);
      return application.Run(ConfigLoader::ResolvePath(PROJECT_ROOT, argv[2]));
    }
    if (VISUALIZE) {
      ExtrinsicVisualizeApplication application(SETTINGS);
      return application.Run(ConfigLoader::ResolvePath(PROJECT_ROOT, argv[2]), argc == 4);
    }
    const auto TALOS_CONFIG = ConfigLoader::LoadFile(CONFIG_ROOT / "hal/camera/talos.yaml");
    ExtrinsicCaptureApplication application(SETTINGS, TALOS_CONFIG);
    return application.Run();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[ExtrinsicCalibration] FATAL: %s\n", error.what());
    return 1;
  }
}

}  // namespace
}  // namespace mv::tool::calibration

int main(int argc, char* argv[]) {
  return mv::tool::calibration::RunExtrinsicCalibration(argc, argv);
}
