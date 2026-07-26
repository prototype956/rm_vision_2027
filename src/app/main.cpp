#include "app/main.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/mindvision_camera.hpp"
#include "tool/debug/debug_window.hpp"

#include <cstdio>
#include <exception>
#include <filesystem>

namespace mv::app {
namespace {

constexpr char K_WINDOW_NAME[] = "MiracleVision Camera Preview";

}  // namespace

int Run() {
  try {
    const std::filesystem::path CONFIG_ROOT = CONFIG_FILE_PATH;
    Logger::Instance().InitFromFile(CONFIG_ROOT / "core/logger.yaml");
    const auto CAMERA_CONFIG =
        ConfigLoader::LoadFile(CONFIG_ROOT / "hal/camera/mindvision.yaml");

    hal::MindVisionCamera camera;
    if (!camera.Open(CAMERA_CONFIG)) {
      MV_LOG_ERROR("App", "camera open failed");
      return 2;
    }

    tool::DebugWindow window(K_WINDOW_NAME);
    while (true) {
      hal::CameraFrame frame;
      const auto STATUS = camera.Grab(frame);

      if (STATUS == hal::GrabStatus::OK) {
        window.Show(frame.image);
      } else if (STATUS == hal::GrabStatus::DISCONNECTED ||
                 STATUS == hal::GrabStatus::FATAL) {
        MV_LOG_ERROR("App", "camera grab failed: {}", hal::GrabStatusName(STATUS));
        return 3;
      }

      if (window.Poll().exit_requested) {
        return 0;
      }
    }
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[App] FATAL: %s\n", error.what());
    return 1;
  }
}

}  // namespace mv::app

int main() {
  return mv::app::Run();
}
