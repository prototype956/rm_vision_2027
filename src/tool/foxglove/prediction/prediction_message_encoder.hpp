#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"
#include "modules/fire_control/fire_control.hpp"

#include <string>

#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::prediction {

/** @brief 图像重投影所选择的固定预测时域。 */
enum class ImagePredictionHorizon {
  CURRENT = 0,   ///< 当前后验状态，对应 0 ms horizon。
  FUTURE_100_MS  ///< 100 ms 匀速外推状态。
};

/**
 * @brief 编码世界系车辆中心、速度、双半径轨迹、关联线和全部预测装甲线框。
 * @return LOST 或没有 horizon 时返回空 SceneUpdate。
 */
[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeScene(
    const modules::ArmorPredictionResult& result, const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 将 EKF 状态、协方差、创新、NIS、关联和重置原因编码为诊断 JSON。 */
[[nodiscard]] std::string EncodeState(const modules::ArmorPredictionResult& result,
                                      const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 在 world 坐标系绘制当前预测中心到最近同标签仿真目标中心的误差线。
 * @return 没有跟踪标签、当前预测或匹配真值时返回空 SceneUpdate。
 */
[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeTruthOverlay(
    const modules::ArmorPredictionResult& result, const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 将指定时域的四装甲位姿按相机畸变模型重投影为图像线框和槽位文字。
 *
 * 正面装甲使用不透明粗线，背面装甲使用半透明细线；图像外或相机后的装甲不输出。
 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeAnnotations(
    const modules::ArmorPredictionResult& result, const hal::CameraFrame::FrameGeometry& geometry,
    ImagePredictionHorizon horizon, const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 生成只携带时间戳的空标注，使 Foxglove 清除上一帧预测线框。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeEmptyAnnotations(
    const ::foxglove::schemas::Timestamp& timestamp);

/** 将火控锁定槽位和待切换槽位重投影到与原图严格同帧的图像坐标。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeSelectedArmorAnnotations(
    const modules::ArmorPredictionResult& result, const hal::CameraFrame::FrameGeometry& geometry,
    const modules::ArmorSelectionSnapshot& selection,
    const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::prediction
