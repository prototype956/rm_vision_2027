#pragma once

#include "hal/camera/i_camera.hpp"

#include <memory>
#include <string_view>

namespace mv::hal {

/**
 * @brief 根据配置名称创建相机后端。
 *
 * 工厂只负责实例化后端，不会打开设备或加载 YAML；调用者仍需使用返回对象的
 * ICamera::Open() 完成初始化。
 *
 * @param backend 支持的后端名称，目前为 mindvision 或 talos。
 * @return 尚未打开的相机实例，所有权交给调用者。
 * @throws ConfigError backend 不在支持列表中时抛出。
 */
[[nodiscard]] std::unique_ptr<ICamera> CreateCamera(std::string_view backend);

}  // namespace mv::hal
