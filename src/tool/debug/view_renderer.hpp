/**
 * @file view_renderer.hpp
 * @brief 调试视图渲染器（Pimpl 模式）
 *
 * 职责：
 *   - 管理主视图窗口（WINDOW_NORMAL，可拖拽调整大小）
 *   - 管理 Debug 辅助窗口（始终显示 binary 图 + Trackbar 区域）
 *   - 支持 5 种视图切换：
 *       RESULT  → 原图 + 检测框/标签/HUD
 *       DIFF    → 通道差分图（灰度转 BGR）
 *       BINARY  → 二值化图（灰度转 BGR）
 *       LIGHTS  → 灯条可视化图（原图 + 灯条轮廓）
 *       ROI     → 原图 + ROI 区域高亮矩形
 *   - HUD 叠加：帧号 / FPS / 跟踪状态 / 视图提示 / 当前参数摘要
 *
 * 设计：
 *   头文件通过 Pimpl 隐藏 OpenCV 实现细节。
 *   依赖 BasicArmorDetector::DebugData / Params 及 interfaces/types.hpp，
 *   均为项目内部头文件，不引入第三方大型头文件。
 */
#pragma once

#include "interfaces/types.hpp"
#include "modules/armor_detector/basic_armor_detector.hpp"

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace mv::tool {

/** @brief 视图模式枚举（与按键 1–5 对应）*/
enum class ViewMode : int {
  RESULT = 0,  ///< 1：最终检测结果（四角点 + HUD）
  DIFF = 1,    ///< 2：通道差分图
  BINARY = 2,  ///< 3：二值化图
  LIGHTS = 3,  ///< 4：灯条可视化图
  ROI = 4,     ///< 5：ROI 区域可视化
};

/**
 * @brief 调试视图渲染器
 *
 * 使用流程：
 * @code
 *   ViewRenderer renderer;
 *   renderer.Init("mv-video-test", "mv-video-debug");
 *   renderer.SetView(ViewMode::RESULT);
 *   // 主循环中：
 *   renderer.Render(raw, dbg, detections, ctrl, frame_idx, fps, params);
 * @endcode
 */
class ViewRenderer {
 public:
  ViewRenderer();
  ~ViewRenderer();

  ViewRenderer(const ViewRenderer&) = delete;
  ViewRenderer& operator=(const ViewRenderer&) = delete;
  ViewRenderer(ViewRenderer&&) noexcept;
  ViewRenderer& operator=(ViewRenderer&&) noexcept;

  /** 创建两个 OpenCV 窗口（WINDOW_NORMAL，可调大小）*/
  void Init(const std::string& main_win, const std::string& debug_win);

  /** 切换主视图显示模式 */
  void SetView(ViewMode mode) noexcept;
  [[nodiscard]] ViewMode GetView() const noexcept;

  /**
   * @brief 渲染一帧并 imshow
   *
   * @param raw        原始图像帧
   * @param dbg        BasicArmorDetector 的调试中间数据
   * @param detections 本帧检测结果
   * @param ctrl       预测器输出的云台控制量
   * @param frame_idx  当前帧号（用于 HUD 显示）
   * @param fps        当前实时 FPS（用于 HUD 显示）
   * @param params     当前检测器参数（用于参数摘要显示）
   * @param roi_rect   当前 ROI 矩形（全图坐标；空 = 未激活）
   */
  void Render(const cv::Mat& raw, const mv::modules::BasicArmorDetector::DebugData& dbg,
              const std::vector<mv::Detection>& detections, const mv::GimbalControl& ctrl,
              int frame_idx, double fps, const mv::modules::BasicArmorDetector::Params& params,
              const std::string& status = "", const cv::Rect2i& roi_rect = {});

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::tool
