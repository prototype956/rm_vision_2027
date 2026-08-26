#pragma once

#include "frame/frame_types.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"
#include "modules/fire_control/fire_control.hpp"
#include "tool/simulation_evaluation/simulation_evaluation_types.hpp"

#include <string>

#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::prediction {

/**
 * @brief 编码世界系当前车辆中心、速度、车体轴、双半径和当前四装甲线框。
 * @return LOST 或没有 0 s horizon 时返回空 SceneUpdate。
 */
[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeScene(
    const modules::ArmorPredictionOutput& output, const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 编码严格同源帧弹道命中时域的四装甲，并突出火控最终选中槽位。 */
[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeImpactScene(
    const modules::ArmorPredictionOutput& output, const modules::ArmorImpactSnapshot* impact,
    const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 将 EKF 状态、协方差、创新、NIS、关联和重置原因编码为诊断 JSON。 */
[[nodiscard]] std::string EncodeState(
    const modules::ArmorPredictionOutput& output,
    const modules::ArmorPredictionDiagnostics& diagnostics,
    const simulation_evaluation::PredictionEvaluationResult* evaluation,
    const ::foxglove::schemas::Timestamp& timestamp);

/**
 * @brief 在 world 坐标系绘制当前预测中心到最近同标签仿真目标中心的误差线。
 * @return 没有跟踪标签、当前预测或匹配真值时返回空 SceneUpdate。
 */
[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeTruthOverlay(
    const modules::ArmorPredictionOutput& output,
    const simulation_evaluation::PredictionEvaluationResult& evaluation,
    const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 将当前四槽位预测和通过全部关联门限的预测轮廓编码为绿色实线。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeCurrentAnnotations(
    const modules::ArmorPredictionOutput& output,
    const modules::ArmorPredictionDiagnostics& diagnostics, const frame::SpatialFrameView* spatial,
    const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 生成只携带时间戳的空标注，使 Foxglove 清除上一帧预测线框。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeEmptyAnnotations(
    const ::foxglove::schemas::Timestamp& timestamp);

/** @brief 将严格同源帧火控选中槽位在弹道命中时域的位姿编码为无文字矩形框。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeImpactAnnotations(
    const modules::ArmorPredictionOutput& output, const frame::SpatialFrameView& spatial,
    const modules::ArmorImpactSnapshot& impact, const ::foxglove::schemas::Timestamp& timestamp);

/** 将火控锁定槽位和待切换槽位重投影到与原图严格同帧的图像坐标。 */
[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeSelectedArmorAnnotations(
    const modules::ArmorPredictionOutput& output, const frame::SpatialFrameView& spatial,
    const modules::ArmorSelectionSnapshot& selection,
    const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::prediction
