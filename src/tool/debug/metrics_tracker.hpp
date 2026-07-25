/**
 * @file metrics_tracker.hpp
 * @brief 帧级性能指标统计（header-only，无重型依赖）
 *
 * 职责：
 *   - 实时 FPS（滑动窗口）
 *   - 总帧数 / 检测帧数 / 总检测目标数
 *   - 程序结束时 PrintStats() 输出摘要
 *
 * 设计：header-only，无 OpenCV / yaml-cpp 依赖，可单独被任意测试程序使用。
 */
#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>

namespace mv::tool {

class MetricsTracker {
 public:
  explicit MetricsTracker(int fps_window = 30) : fps_window_{fps_window} {}

  // ── 每帧调用一次 ──────────────────────────────────────────────────────────

  /**
   * @param has_detection  本帧是否检测到目标
   * @param det_count      本帧检测到的目标数量
   */
  void Tick(bool has_detection, int det_count) noexcept {
    ++total_frames_;
    if (has_detection) {
      ++frames_with_detection_;
      total_detections_ += det_count;
    }
    // 滑动窗口 FPS
    if (++fps_count_ >= fps_window_) {
      const auto t_now = std::chrono::steady_clock::now();
      const double elapsed =
          std::chrono::duration<double>(t_now - t_last_fps_).count();
      current_fps_ = (elapsed > 0.0) ? static_cast<double>(fps_count_) / elapsed : 0.0;
      t_last_fps_ = t_now;
      fps_count_  = 0;
    }
  }

  // ── 查询接口 ──────────────────────────────────────────────────────────────

  [[nodiscard]] double CurrentFps()   const noexcept { return current_fps_; }
  [[nodiscard]] int    TotalFrames()  const noexcept { return total_frames_; }

  // ── 统计输出 ──────────────────────────────────────────────────────────────

  void PrintStats() const {
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start_).count();

    const double avg_fps =
        (elapsed > 0.0) ? static_cast<double>(total_frames_) / elapsed : 0.0;
    const double detect_ratio =
        (total_frames_ > 0)
            ? static_cast<double>(frames_with_detection_) /
                  static_cast<double>(total_frames_) * 100.0
            : 0.0;
    const double avg_det =
        (total_frames_ > 0)
            ? static_cast<double>(total_detections_) /
                  static_cast<double>(total_frames_)
            : 0.0;

    std::cout << "\n══════════ 测试统计 ══════════\n"
              << "  总帧数          : " << total_frames_ << "\n"
              << "  耗时（秒）      : " << std::fixed << std::setprecision(2) << elapsed << "\n"
              << "  平均 FPS        : " << std::setprecision(1) << avg_fps << "\n"
              << "  检测帧比例      : " << std::setprecision(1) << detect_ratio << "%"
              << " (" << frames_with_detection_ << "/" << total_frames_ << ")\n"
              << "  平均每帧检测数  : " << std::setprecision(2) << avg_det << "\n"
              << "══════════════════════════════\n";
  }

 private:
  int       fps_window_{30};
  int       total_frames_{0};
  int       frames_with_detection_{0};
  long long total_detections_{0};

  int    fps_count_{0};
  double current_fps_{0.0};

  std::chrono::steady_clock::time_point t_start_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point t_last_fps_{std::chrono::steady_clock::now()};
};

}  // namespace mv::tool
