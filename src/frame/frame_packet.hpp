#pragma once

#include "frame/frame_types.hpp"
#include "hal/gimbal/gimbal_types.hpp"
#include "simulation/simulation_frame_data.hpp"

#include <optional>

namespace mv::frame {

/**
 * @brief 帧源输出的图像以及可选同帧空间、执行器和仿真数据。
 *
 * 所有可选成员只能与 capture 来自同一个采集快照，禁止跨帧组合。
 */
struct FramePacket {
  CapturedFrame capture;                      ///< 必需的图像和采集标识。
  std::optional<CameraModel> camera_model;    ///< 同帧相机模型。
  std::optional<FrameKinematics> kinematics;  ///< 同帧平台运动学。
  std::optional<hal::GimbalActuatorTelemetry> gimbal_actuator;  ///< 同帧执行器状态。
  std::optional<simulation::SimulationFrameData> simulation;    ///< 同帧仿真数据。
};

/** @brief 当相机模型和平台运动学均存在时构造完整空间视图。 */
[[nodiscard]] inline std::optional<SpatialFrameView> MakeSpatialFrameView(
    const FramePacket& packet) noexcept {
  if (!packet.camera_model || !packet.kinematics)
    return std::nullopt;
  return SpatialFrameView{.calibration = *packet.camera_model,
                          .world_t_gimbal = packet.kinematics->world_t_gimbal,
                          .gimbal_t_camera_optical = packet.kinematics->gimbal_t_camera_optical,
                          .gimbal_t_muzzle = packet.kinematics->gimbal_t_muzzle};
}

}  // namespace mv::frame
