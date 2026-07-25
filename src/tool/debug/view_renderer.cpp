/**
 * @file view_renderer.cpp
 * @brief ViewRenderer 实现（Pimpl 模式）
 *
 * 【模块职责说明】
 *   - 匿名 namespace 中的辅助函数（DrawDetection / DrawHUD /
 *     DrawDebugOverlay / MakeFullMat）均为纯函数、无状态，仅本文件可见；
 *   - Impl struct 极简：仅持窗口名和当前视图枚举，无图像缓存，
 *     每帧 Render() 完整重新生成显示内容，不保留帧间状态；
 *   - Debug 窗口始终显示还原到全图尺寸的 binary 图 + 当前参数单行摘要，
 *     独立于主窗口的视图模式，便于同时观察二值化效果与最终检测结果。
 *
 * 【ROI 局部图还原（MakeFullMat）】
 *   当 roi_rect.area() > 0 时，BasicArmorDetector::DebugData 中的
 *   binary / diff 均为 ROI 裁剪片段（尺寸 < 全图）。
 *   MakeFullMat() 将其粘贴回全图尺寸的零背景 Mat，确保 DIFF / BINARY /
 *   LIGHTS 视图与原始帧尺寸一致，不发生坐标错位或显示缩放问题。
 */
#include "tool/debug/view_renderer.hpp"

#include "tool/debug/light_vis_painter.hpp"

#include <array>
#include <iomanip>
#include <sstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::tool {

// ── 内部绘制辅助（匿名 namespace，仅本文件可见）─────────────────────────────

namespace {

const char* ViewModeName(ViewMode mode) noexcept {
  switch (mode) {
    case ViewMode::DIFF:
      return "diff";
    case ViewMode::BINARY:
      return "binary";
    case ViewMode::LIGHTS:
      return "lights";
    case ViewMode::ROI:
      return "roi";
    default:
      return "result";
  }
}

/** @brief 在帧上绘制单个 Detection（四角点多边形、中心点、3D 标签）*/
void DrawDetection(cv::Mat& frame, const mv::Detection& det) {
  // 四角点连线（绿色）——显式写出避免可变下标警告
  const auto& pts = det.points;
  cv::line(frame, pts[0], pts[1], cv::Scalar(0, 255, 0), 2);
  cv::line(frame, pts[1], pts[2], cv::Scalar(0, 255, 0), 2);
  cv::line(frame, pts[2], pts[3], cv::Scalar(0, 255, 0), 2);
  cv::line(frame, pts[3], pts[0], cv::Scalar(0, 255, 0), 2);

  // 中心点（红色）
  const cv::Point2f CENTER = det.Center();
  cv::circle(frame, CENTER, 5, cv::Scalar(0, 0, 255), -1);

  // 标签
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << "conf:" << det.confidence;
  if (det.is_solved) {
    oss << " y:" << std::setprecision(1) << det.yaw_angle * 180.0 / M_PI << "d"
        << " p:" << det.pitch_angle * 180.0 / M_PI << "d";
  }
  cv::putText(frame, oss.str(),
              cv::Point(static_cast<int>(CENTER.x) - 55, static_cast<int>(CENTER.y) - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

/** @brief 在帧上叠加 HUD（左上角：帧号/FPS/视图；右上角：参数摘要）*/
void DrawHUD(cv::Mat& frame, int frame_idx, double fps, const mv::GimbalControl& ctrl,
             ViewMode view_mode, const mv::modules::BasicArmorDetector::Params& params,
             const std::string& status) {
  // 第 1 行：帧号 / FPS / 视图提示
  {
    std::ostringstream oss;
    oss << "Frame:" << frame_idx << "  FPS:" << std::fixed << std::setprecision(1) << fps
        << "  View:[" << ViewModeName(view_mode) << "] 1/2/3/4/5  [s]save [c]color";
    cv::putText(frame, oss.str(), cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.50,
                cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
  }
  // 第 2 行：跟踪状态
  if (ctrl.tracking) {
    std::ostringstream oss;
    oss << "TRACKING  yaw:" << std::fixed << std::setprecision(1) << ctrl.yaw * 180.0 / M_PI << "d"
        << "  pitch:" << ctrl.pitch * 180.0 / M_PI << "d";
    cv::putText(frame, oss.str(), cv::Point(10, 44), cv::FONT_HERSHEY_SIMPLEX, 0.46,
                cv::Scalar(0, 255, 128), 1, cv::LINE_AA);
  } else {
    cv::putText(frame, "LOST", cv::Point(10, 44), cv::FONT_HERSHEY_SIMPLEX, 0.46,
                cv::Scalar(80, 80, 255), 1, cv::LINE_AA);
  }
  // 第 3 行：状态信息（如当前识别颜色）
  if (!status.empty()) {
    cv::putText(frame, status, cv::Point(10, 64), cv::FONT_HERSHEY_SIMPLEX, 0.46,
                cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
  }
  // 右上角：参数摘要
  {
    std::ostringstream oss;
    oss << "thr:" << params.light_thresh << " ang:" << std::fixed << std::setprecision(0)
        << params.max_light_angle << " ar:[" << std::setprecision(1) << params.min_armor_ratio
        << "," << params.max_armor_ratio << "]"
        << " dff:" << std::setprecision(0) << params.max_angle_diff;
    const std::string TXT = oss.str();
    const int X = std::max(0, frame.cols - static_cast<int>(TXT.size()) * 7 - 4);
    cv::putText(frame, TXT, cv::Point(X, 18), cv::FONT_HERSHEY_SIMPLEX, 0.40,
                cv::Scalar(180, 180, 255), 1, cv::LINE_AA);
  }
}

/** @brief 在 debug 窗口的 binary 图上叠加参数单行文字 */
void DrawDebugOverlay(cv::Mat& bin_bgr, const mv::modules::BasicArmorDetector::Params& params) {
  std::ostringstream oss;
  oss << "thr:" << params.light_thresh << " ang:" << std::fixed << std::setprecision(0)
      << params.max_light_angle << " ar:[" << std::setprecision(1) << params.min_armor_ratio << ","
      << params.max_armor_ratio << "]"
      << " dff:" << std::setprecision(0) << params.max_angle_diff;
  cv::putText(bin_bgr, oss.str(), cv::Point(4, 20), cv::FONT_HERSHEY_SIMPLEX, 0.46,
              cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

}  // anonymous namespace

// ── ROI 辅助：将 ROI 局部图粘贴回全图尺寸 ────────────────────────────────

namespace {

/**
 * @brief 将 ROI 局部单通道图粘贴到全图尺寸的 Mat 中。
 *
 * @param src        ROI 局部图（全图或裁剪片段）
 * @param roi_rect   全图坐标下的 ROI 矩形（面积为 0 = 未激活）
 * @param frame_size 全图尺寸（用于创建画布）
 */
cv::Mat MakeFullMat(const cv::Mat& src, const cv::Rect2i& roi_rect, const cv::Size& frame_size) {
  if (src.empty()) {
    return {};
  }
  if (roi_rect.area() == 0 || src.size() == frame_size) {
    return src;
  }
  cv::Mat full = cv::Mat::zeros(frame_size, src.type());
  const cv::Rect PASTE_RECT{roi_rect.tl(), src.size()};
  const cv::Rect SAFE = PASTE_RECT & cv::Rect(cv::Point{0, 0}, frame_size);
  if (SAFE.area() > 0) {
    const cv::Rect SRC_RECT{cv::Point{0, 0}, SAFE.size()};
    src(SRC_RECT).copyTo(full(SAFE));
  }
  return full;
}

}  // namespace

// ── Impl（public struct，成员命名 lower_case 无后缀）─────────────────────────

struct ViewRenderer::Impl {
  std::string main_win;
  std::string debug_win;
  ViewMode view{ViewMode::RESULT};
};

// ── 构造 / 析构 ────────────────────────────────────────────────────────────

ViewRenderer::ViewRenderer() : impl_(std::make_unique<Impl>()) {}
ViewRenderer::~ViewRenderer() = default;

ViewRenderer::ViewRenderer(ViewRenderer&&) noexcept = default;
ViewRenderer& ViewRenderer::operator=(ViewRenderer&&) noexcept = default;

// ── 公开接口实现 ────────────────────────────────────────────────────────────

void ViewRenderer::Init(const std::string& main_win, const std::string& debug_win) {
  impl_->main_win = main_win;
  impl_->debug_win = debug_win;
  cv::namedWindow(main_win, cv::WINDOW_NORMAL);
  cv::namedWindow(debug_win, cv::WINDOW_NORMAL);
  cv::resizeWindow(main_win, 960, 540);
  cv::resizeWindow(debug_win, 640, 480);
}

void ViewRenderer::SetView(ViewMode mode) noexcept {
  impl_->view = mode;
}

[[nodiscard]] ViewMode ViewRenderer::GetView() const noexcept {
  return impl_->view;
}

void ViewRenderer::Render(const cv::Mat& raw, const mv::modules::BasicArmorDetector::DebugData& dbg,
                          const std::vector<mv::Detection>& detections,
                          const mv::GimbalControl& ctrl, int frame_idx, double fps,
                          const mv::modules::BasicArmorDetector::Params& params,
                          const std::string& status, const cv::Rect2i& roi_rect) {
  const cv::Size FRAME_SIZE = raw.size();
  // ── 选择主视图内容 ────────────────────────────────────────────────────
  cv::Mat display;
  switch (impl_->view) {
    case ViewMode::DIFF: {
      const cv::Mat FULL_DIFF = MakeFullMat(dbg.diff, roi_rect, FRAME_SIZE);
      cv::cvtColor(FULL_DIFF.empty() ? (cv::Mat)cv::Mat::zeros(raw.size(), CV_8UC1) : FULL_DIFF,
                   display, cv::COLOR_GRAY2BGR);
      break;
    }
    case ViewMode::BINARY: {
      const cv::Mat FULL_BIN = MakeFullMat(dbg.binary, roi_rect, FRAME_SIZE);
      cv::cvtColor(FULL_BIN.empty() ? (cv::Mat)cv::Mat::zeros(raw.size(), CV_8UC1) : FULL_BIN,
                   display, cv::COLOR_GRAY2BGR);
      break;
    }
    case ViewMode::LIGHTS:
      // 将 ROI 二值图还原到全图后再送给 PaintLightBarsVis（确保轮廓坐标正确）
      display =
          mv::tool::PaintLightBarsVis(MakeFullMat(dbg.binary, roi_rect, FRAME_SIZE), raw, params);
      break;
    case ViewMode::ROI: {
      display = raw.clone();
      if (roi_rect.area() > 0) {
        cv::rectangle(display, roi_rect, cv::Scalar(0, 255, 255), 2);
        cv::putText(display, "ROI", roi_rect.tl() + cv::Point(2, -4), cv::FONT_HERSHEY_SIMPLEX,
                    0.50, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
      } else {
        cv::putText(display, "ROI: FULL FRAME", cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.50,
                    cv::Scalar(80, 80, 255), 1, cv::LINE_AA);
      }
      for (const auto& det : detections) {
        DrawDetection(display, det);
      }
      break;
    }
    default:  // ViewMode::RESULT
      display = raw.clone();
      for (const auto& det : detections) {
        DrawDetection(display, det);
      }
      break;
  }

  // ── HUD 叠加 ──────────────────────────────────────────────────────────
  DrawHUD(display, frame_idx, fps, ctrl, impl_->view, params, status);
  cv::imshow(impl_->main_win, display);

  // ── Debug 窗口：始终显示还原到全图的 binary 图 ────────────────────────
  const cv::Mat FULL_BIN_FOR_DBG = MakeFullMat(dbg.binary, roi_rect, FRAME_SIZE);
  if (!FULL_BIN_FOR_DBG.empty()) {
    cv::Mat bin_bgr;
    cv::cvtColor(FULL_BIN_FOR_DBG, bin_bgr, cv::COLOR_GRAY2BGR);
    DrawDebugOverlay(bin_bgr, params);
    cv::imshow(impl_->debug_win, bin_bgr);
  }
}

}  // namespace mv::tool
