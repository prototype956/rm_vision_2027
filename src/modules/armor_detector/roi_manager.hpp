/**
 * @file roi_manager.hpp
 * @brief 帧间 ROI 管理器 —— 与检测器完全解耦的独立 ROI 状态机
 *
 * 【设计原则】
 *   IDetector 只感知"输入帧中的目标"，不应知晓 ROI 的存在。
 *   RoiManager 作为独立工具类，包裹在检测器调用的外层：
 *
 * 【典型用法】
 * @code
 *   mv::modules::RoiManager roi;
 *
 *   // 主循环：
 *   auto [view, offset] = roi.Crop(frame);          // ① 裁剪
 *   auto dets = detector.Detect(view, color);        // ② 检测（局部坐标）
 *   roi.RestoreAndUpdate(dets, offset, frame.size()); // ③ 还原坐标 + 更新状态
 *   // dets 中坐标已变为全图坐标，可直接使用
 * @endcode
 *
 * 【ROI 扩展策略】
 *   成功帧：以所有检测目标包围盒扩展 ×1.5宽 / ×2高 作为下一帧 ROI。
 *   连续 kMaxLost(=5) 帧无目标：自动回退全图。
 *   外部可调用 Reset() 强制回退（如切换颜色时）。
 */
#pragma once

#include "interfaces/types.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

namespace mv::modules {

class RoiManager {
 public:
  // ── 类型 ──────────────────────────────────────────────────────────────────

  /** Crop() 的返回值：ROI 视图（浅拷贝）+ 全图左上角偏移 */
  struct CropResult {
    cv::Mat view;              ///< ROI 视图（frame 的子 Mat，不分配内存）
    cv::Point2i offset{0, 0};  ///< 本帧 ROI 左上角在原始帧中的坐标
  };

  // ── 构造 ──────────────────────────────────────────────────────────────────

  RoiManager() = default;

  // ── 核心接口 ──────────────────────────────────────────────────────────────

  /**
   * @brief 按当前历史 ROI 裁剪输入帧
   *
   * 若无有效历史 ROI（状态机尚未激活或已回退），返回完整帧 + offset={0,0}。
   * 裁剪区域与帧边界取交集，保证不越界。
   */
  [[nodiscard]] CropResult Crop(const cv::Mat& frame) const {
    if (state_.roi_rect.area() <= 0) {
      return {frame, {0, 0}};
    }
    const cv::Rect2i FRAME_RECT{0, 0, frame.cols, frame.rows};
    const cv::Rect2i SAFE = state_.roi_rect & FRAME_RECT;
    if (SAFE.area() <= 0) {
      return {frame, {0, 0}};
    }
    return {frame(SAFE), {SAFE.x, SAFE.y}};
  }

  /**
   * @brief 将检测结果从 ROI 局部坐标还原到全图坐标，并更新 ROI 状态
   *
   * @param detections  Detect() 输出（局部坐标），将被就地修改为全图坐标
   * @param offset      本帧 Crop() 返回的偏移量
   * @param frame_size  原始帧尺寸（用于按全图中心更新 distance_to_center）
   */
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  void RestoreAndUpdate(std::vector<Detection>& detections, const cv::Point2i& offset,
                        const cv::Size& frame_size) {
    if (detections.empty()) {
      if (++state_.lost_count >= kMaxLost) {
        state_ = {};
      }
      return;
    }

    // ── 坐标还原：局部 → 全图 ───────────────────────────────────────────
    const cv::Point2f OFF{static_cast<float>(offset.x), static_cast<float>(offset.y)};
    const cv::Point2f IMG_CTR{
        static_cast<float>(frame_size.width) * 0.5F,
        static_cast<float>(frame_size.height) * 0.5F,
    };
    for (auto& det : detections) {
      for (auto& point_px : det.points) {
        point_px += OFF;
      }
      det.box.x += OFF.x;
      det.box.y += OFF.y;
      det.distance_to_center = static_cast<double>(cv::norm(det.Center() - IMG_CTR));
    }

    // ── 更新 ROI：以本帧所有目标包围盒扩展后存入 state_ ────────────────
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& det : detections) {
      min_x = std::min(min_x, det.box.x);
      min_y = std::min(min_y, det.box.y);
      max_x = std::max(max_x, det.box.x + det.box.width);
      max_y = std::max(max_y, det.box.y + det.box.height);
    }

    const float BOX_W = max_x - min_x;
    const float BOX_H = max_y - min_y;
    const float EXPAND_X = BOX_W * 1.5F;
    const float EXPAND_Y = BOX_H * 2.0F;
    state_.roi_rect = cv::Rect2i{
        static_cast<int>(min_x - EXPAND_X), static_cast<int>(min_y - EXPAND_Y),
        static_cast<int>(BOX_W + EXPAND_X * 2.0F), static_cast<int>(BOX_H + EXPAND_Y * 2.0F)};
    state_.lost_count = 0;
  }

  /** 立即清空 ROI 状态，下一帧回退全图 */
  void Reset() noexcept { state_ = {}; }

  /** 获取当前活跃的 ROI 矩形（area()==0 表示使用全图）*/
  [[nodiscard]] cv::Rect2i GetRoiRect() const noexcept { return state_.roi_rect; }

  /** ROI 是否当前处于激活状态 */
  [[nodiscard]] bool IsActive() const noexcept { return state_.roi_rect.area() > 0; }

 private:
  /// 连续失败帧上限：超过此帧数无目标后自动回退全图
  // NOLINTNEXTLINE(readability-identifier-naming)
  static constexpr int kMaxLost = 5;

  /**
   * 内部状态块，封装为小类以支持高效重置（Reset() 只需 state_ = {} 即可）。
   * roi_rect.area() == 0 表示当前处于全图模式。
   */
  struct State {
    cv::Rect2i roi_rect{0, 0, 0, 0};  ///< 当前历史 ROI，{0,0,0,0} 表示使用全图
    int lost_count{0};                ///< 连续未检测到目标的帧数
  };
  State state_{};
};

}  // namespace mv::modules
