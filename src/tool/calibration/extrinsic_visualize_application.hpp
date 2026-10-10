#pragma once

#include "tool/calibration/extrinsic_capture_application.hpp"

#include <filesystem>

namespace mv::tool::calibration {

/**
 * @brief 将离线外参解算结果回放到 Foxglove 3D 面板。
 *
 * 查看器只读取 solution 目录，不访问 Talos，也不修改 session、report 或 extrinsics 文件。
 */
class ExtrinsicVisualizeApplication final {
 public:
  /** @brief 保存采集配置，供校验棋盘和解算结果格式。 */
  explicit ExtrinsicVisualizeApplication(ExtrinsicCaptureSettings settings);

  /**
   * @brief 启动 Foxglove WebSocket 并循环回放离线结果。
   * @param solution_dir 含 extrinsics.yaml 和 report.yaml 的解算目录。
   * @param once true 时只发布一轮后退出，默认持续回放直到 Ctrl+C。
   * @return 0 表示正常结束，5 表示输入或发布初始化失败，6 表示信号中止。
   */
  int Run(const std::filesystem::path& solution_dir, bool once);

 private:
  ExtrinsicCaptureSettings settings_;
};

}  // namespace mv::tool::calibration
