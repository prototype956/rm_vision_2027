#pragma once

#include "frame/frame_packet.hpp"
#include "modules/fire_control/fire_control.hpp"
#include "runtime/vision_frame_diagnostics.hpp"
#include "runtime/vision_frame_output.hpp"
#include "tool/simulation_evaluation/simulation_evaluation_types.hpp"

#include <chrono>
#include <cstdint>

#include <optional>

namespace mv::tool::foxglove::pipeline {

using SteadyClock = std::chrono::steady_clock;

/**
 * @brief 后台编码线程消费的完整同帧调试数据。
 *
 * image 通过 cv::Mat 引用计数共享像素所有权；检测结果、统计和可选空间元数据在
 * 入队时复制，后台线程不依赖检测器或相机对象的后续状态。
 */
struct VisionDebugFrame {
  frame::FramePacket packet;  ///< 与源帧共享图像所有权的完整同帧数据包。
  ::mv::runtime::VisionFrameOutput output;            ///< 当前帧正式视觉输出副本。
  ::mv::runtime::VisionFrameDiagnostics diagnostics;  ///< 当前帧视觉诊断输出副本。
  std::optional<simulation_evaluation::SimulationEvaluationResult>
      simulation_evaluation;  ///< 同帧仿真评估；真值缺失或评估失败时为空。
  std::optional<modules::ArmorSelectionSnapshot> armor_selection;  ///< 匹配的控制选择快照。
};

}  // namespace mv::tool::foxglove::pipeline
