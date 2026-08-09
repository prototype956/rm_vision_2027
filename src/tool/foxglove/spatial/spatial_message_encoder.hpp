#pragma once

#include "hal/camera/i_camera.hpp"

#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::spatial {

/** @brief 编码同帧的 world -> gimbal -> camera_optical 两级 TF。 */
[[nodiscard]] ::foxglove::schemas::FrameTransforms EncodeTransforms(
    const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 编码 camera_optical 的针孔内参与 plumb_bob 畸变参数。
 *
 * 畸变数组顺序为 k1、k2、p1、p2、k3；标定尺寸必须与同帧图像一致。
 */
[[nodiscard]] ::foxglove::schemas::CameraCalibration EncodeCalibration(
    const hal::CameraFrame::Calibration& calibration,
    const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 根据针孔内参生成 camera_optical 下深度为 1 米的视锥线框。
 *
 * 视锥仅用于观察相机视场，不对图像畸变进行建模，也不参与测量。
 */
[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeFrustum(
    const hal::CameraFrame::Calibration& calibration,
    const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::spatial
