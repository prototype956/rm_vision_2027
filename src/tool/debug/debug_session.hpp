/**
 * @file debug_session.hpp
 * @brief 调试会话门面（Pimpl 模式）
 *
 * 职责：
 *   聚合 ParamTuner / ViewRenderer / MetricsTracker，
 *   为测试程序提供单一入口，屏蔽调试基础设施细节。
 *
 * 测试程序侧典型用法（main() 主要部分）：
 * @code
 *   mv::tool::DebugSession dbg;
 *   dbg.Init({"mv-video-test", "mv-video-debug",
 *             CONFIG_FILE_PATH "/debug_override.yaml"});
 *
 *   // 注册参数（lambda 捕获外部 Params 引用）
 *   dbg.AddParam({"Thresh", "light_thresh", p.light_thresh, 255,
 *       [&p](int v){ p.light_thresh = v; },
 *       [&p]{ return (double)p.light_thresh; }});
 *
 *   // 自定义按键（q/ESC/空格/1-4 已内置注册）
 *   dbg.BindKey('s', [&dbg]{ dbg.SaveParams(); });
 *
 *   // 主循环
 *   while (!dbg.Poll().quit) {
 *       if (dbg.Poll().paused) continue;
 *       dbg.ApplyParams();
 *       detector.SetParams(p);
 *       // ... 检测/解算/预测 ...
 *       dbg.TickFrame(!dets.empty(), (int)dets.size());
 *       dbg.Feed(frame, detector.GetDebugData(), dets, ctrl, p);
 *   }
 *   dbg.PrintStats();
 * @endcode
 *
 * Foxglove 扩展预留：
 *   未来可在 Impl 内添加 FoxgloveSink，
 *   接入后 Feed() 同时向 Foxglove 发布数据，test 程序代码零改动。
 */
#pragma once

#include "interfaces/types.hpp"
#include "modules/armor_detector/basic_armor_detector.hpp"
#include "tool/debug/param_tuner.hpp"
#include "tool/debug/view_renderer.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace mv::tool {

class DebugSession {
 public:
  // ── 配置 ─────────────────────────────────────────────────────────────────

  struct Config {
    std::string main_window = "mv-video-test";    ///< 主视图窗口标题
    std::string debug_window = "mv-video-debug";  ///< Debug/Trackbar 窗口标题
    std::string save_yaml;  ///< SaveParams() 写入路径（空串=不保存）
    int fps_window = 30;    ///< FPS 滑动窗口帧数
  };

  // ── 主循环交互 ────────────────────────────────────────────────────────────

  /** Poll() 的返回值，供主循环判断控制流 */
  struct PollResult {
    bool quit{false};    ///< 是否应退出主循环
    bool paused{false};  ///< 当前是否处于暂停状态
  };

  // ── 生命周期 ──────────────────────────────────────────────────────────────

  DebugSession();
  ~DebugSession();

  DebugSession(const DebugSession&) = delete;
  DebugSession& operator=(const DebugSession&) = delete;
  DebugSession(DebugSession&&) noexcept;
  DebugSession& operator=(DebugSession&&) noexcept;

  /**
   * @brief 初始化会话：创建窗口、注册内置按键（q/ESC/空格/1-4）
   * @param cfg  会话配置
   */
  void Init(const Config& cfg);

  // ── 参数调节 ──────────────────────────────────────────────────────────────

  /** 注册一个可调参数并创建对应 Trackbar（必须在 Init() 之后调用）*/
  void AddParam(ParamDesc desc);

  /** 将所有 Trackbar 当前值推送到外部 Params 结构（每帧调用）*/
  void ApplyParams();

  /** 将当前参数写入 cfg.save_yaml（键 's' 的默认行为）*/
  void SaveParams();

  // ── 按键绑定 ──────────────────────────────────────────────────────────────

  /**
   * @brief 绑定自定义按键
   *
   * 内置按键（不可覆盖）：q / ESC → quit，空格 → pause/resume，
   * 1–4 → 视图切换。
   * 其余键（如 's'）可自由绑定。
   */
  void BindKey(int key, std::function<void()> action);

  // ── 主循环 ────────────────────────────────────────────────────────────────

  /**
   * @brief 处理本帧的按键事件并返回当前控制状态
   *
   * 内部调用 cv::waitKey(paused ? 50 : 1)，无需在外部调用 waitKey。
   */
  PollResult Poll();

  /**
   * @brief 提交本帧数据，触发渲染
   *
   * @param raw        原始帧
   * @param dbg        检测器调试数据（diff / binary；lights_vis 已移至 PaintLightBarsVis）
   * @param detections 检测结果列表
   * @param ctrl       云台控制指令（含跟踪状态）
   * @param params     当前检测器参数（用于 HUD 显示摘要）
   * @param status     可选字符串状态（如当前识别颜色）
   * @param roi_rect   当前 ROI 区域（5键可视化，全零 = 未激活）
   */
  void Feed(const cv::Mat& raw, const mv::modules::BasicArmorDetector::DebugData& dbg,
            const std::vector<mv::Detection>& detections, const mv::GimbalControl& ctrl,
            const mv::modules::BasicArmorDetector::Params& params, const std::string& status = "",
            const cv::Rect2i& roi_rect = {});

  // ── 性能指标 ──────────────────────────────────────────────────────────────

  /** 统计本帧（应在 Feed() 之前调用）*/
  void TickFrame(bool has_detection, int det_count);

  /** 程序结束时打印统计摘要 */
  void PrintStats() const;

  // ── 视图 ──────────────────────────────────────────────────────────────────

  void SetView(ViewMode mode);
  [[nodiscard]] ViewMode GetView() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::tool
