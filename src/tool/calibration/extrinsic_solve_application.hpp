#pragma once

#include "tool/calibration/extrinsic_capture_application.hpp"

#include <filesystem>

namespace mv::tool::calibration {

/**
 * @brief 离线检测棋盘并复刻固定棋盘手眼标定的解算前端。
 *
 * 输入会话由 ExtrinsicCaptureApplication 生成；本类不打开 Talos，也不修改原始图像和
 * session.yaml。它只在会话下创建独立 solutions 子目录。
 */
class ExtrinsicSolveApplication final {
 public:
  /** @brief 保存棋盘和求解约束。 */
  explicit ExtrinsicSolveApplication(ExtrinsicCaptureSettings settings);

  /**
   * @brief 逐张人工确认角点后执行 Tsai 手眼求解。
   * @param session_dir 含 session.yaml 和 images/ 的采集目录。
   * @return 0 表示求解成功，4 表示用户取消，5 表示输入校验、有效观测不足或 OpenCV 求解失败。
   * @throws std::exception 结果目录不可创建或报告不可写入时抛出。
   */
  int Run(const std::filesystem::path& session_dir);

 private:
  ExtrinsicCaptureSettings settings_;
};

}  // namespace mv::tool::calibration
