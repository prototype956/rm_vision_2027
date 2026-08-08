#include "hal/camera/camera_factory.hpp"

#include "core/config.hpp"
#include "hal/camera/mindvision/mindvision_camera.hpp"
#include "hal/camera/talos/talos_camera.hpp"

#include <string>

namespace mv::hal {

std::unique_ptr<ICamera> CreateCamera(std::string_view backend) {
  // 名称与 app/main.yaml 的 camera.backend 保持一致，避免在工厂中引入配置依赖。
  if (backend == "mindvision") {
    return std::make_unique<MindVisionCamera>();
  }
  if (backend == "talos") {
    return std::make_unique<TalosCamera>();
  }
  throw ConfigError("unsupported camera backend '" + std::string(backend) +
                    "' (expected mindvision or talos)");
}

}  // namespace mv::hal
