#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"
#include "modules/armor_predictor/armor_prediction_types.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

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
  cv::Mat image;                                ///< 与相机帧共享所有权的 BGR 原图。
  SteadyClock::time_point receive_steady_time;  ///< HAL 收帧单调时钟，用于限流和延迟统计。
  std::optional<std::uint64_t> capture_timestamp_ns;  ///< 数据源采集 Unix epoch 纳秒时间。
  std::optional<hal::CameraFrame::FrameGeometry> geometry;  ///< 与图像原子同步的空间元数据。
  std::uint64_t sequence{0};               ///< 当前相机 Open() 周期内递增的帧序号。
  std::uint64_t source_invalid_frames{0};  ///< 数据源自 Open() 以来累计拒绝的无效帧数。
  std::vector<modules::ArmorDetection> detections;   ///< 当前帧检测结果副本。
  modules::DetectorStats detector_stats;             ///< 当前帧检测性能指标副本。
  modules::ArmorPnpFrameResult pnp_result;           ///< 当前帧 PnP 基准与检测结果。
  modules::ArmorPredictionResult prediction_result;  ///< 当前帧四装甲预测与诊断。
};

}  // namespace mv::tool::foxglove::pipeline
