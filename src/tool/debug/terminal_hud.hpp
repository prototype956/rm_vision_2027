/**
 * @file terminal_hud.hpp
 * @brief 终端 HUD 状态显示（header-only，零图像/网络开销）
 *
 * 【设计目标】
 *   在无笔电网线的赛场极端环境下，通过纯终端输出替代 Foxglove 可视化，
 *   以固定速率（默认 200 ms）刷新双行状态，不占用 Pipeline 帧率。
 *
 * 【输出格式（TTY）】
 * @code
 *   [MV] 15:42:01 | FPS: 143 | DET:  2/ARMOR | LOCK: ✓ | YAW:  -3.2° PITCH:   1.1° DIST:  4.20m
 *   [THR] Capture✓143fps  Detect✓115fps  Predict✓116fps  Serial✓999fps
 * @endcode
 *
 * 【TTY 检测】
 *   若 stdout 被重定向到文件或管道，自动禁用 ANSI 颜色和 \r 行刷新，改为换行输出。
 *
 * 【最小依赖】
 *   标准库 + interfaces/types.hpp（mv::Detection / mv::GimbalControl）
 *   零 OpenCV / Foxglove / yaml-cpp 依赖，可独立被任何目标使用。
 *
 * 【典型用法】
 * @code
 *   mv::tool::TerminalHUD hud;   // 默认 200ms 刷新
 *
 *   // 在 Pipeline 的 Update tick 中：
 *   hud.Update(fps, dets, &ctrl, &node_metrics);
 *
 *   // 或在 DebugSession 主循环：
 *   hud.Update(tracker.CurrentFps(), dets);
 * @endcode
 *
 * 【NodeMetrics 与 FoxgloveSink::ThreadMetrics 的关系】
 *   两者字段完全相同，方便从 FoxgloveSink::ThreadMetrics 复制：
 * @code
 *   std::vector<TerminalHUD::NodeMetrics> nm;
 *   for (const auto& m : foxglove_metrics) {
 *       nm.push_back({m.node_name, m.fps, m.latency_ms, m.drop_count, m.is_alive, m.error_msg});
 *   }
 * @endcode
 */
#pragma once

#include "interfaces/types.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>
#include <unistd.h>
#include <vector>

namespace mv::tool {

namespace {
/// rad 转 degree 转换系数（移到匿名 namespace 避免 clangd-14 static constexpr 成员触发 InlayHints 崩溃）
inline constexpr double kRad2Deg = 57.295779513082323;
}  // namespace

class TerminalHUD {
 public:
  // ── 节点健康指标（字段与 FoxgloveSink::ThreadMetrics 一一对应）──────────

  struct NodeMetrics {
    std::string node_name;   ///< 节点名称，如 "CaptureNode"
    double fps{0.0};         ///< 当前处理帧率
    double latency_ms{0.0};  ///< 平均处理延迟（ms）
    uint64_t drop_count{0};  ///< 累计丢帧数
    bool is_alive{true};     ///< 线程是否仍在运行
    std::string error_msg;   ///< 最近一条错误信息（空=无错）
  };

  // ── 配置 ─────────────────────────────────────────────────────────────────

  struct Config {
    int interval_ms{200};  ///< 刷新间隔（毫秒），0 = 每帧都打印
    bool use_color{true};  ///< 是否使用 ANSI 颜色，非 TTY 时自动关闭
    bool use_cr{true};     ///< 同行刷新（\r）；管道/文件输出时自动切换为换行
  };

  // ── 生命周期 ──────────────────────────────────────────────────────────────

  /// 使用默认配置（host=0.0.0.0:8765, interval_ms=200）
  TerminalHUD() : TerminalHUD(Config{}) {}

  /// 自定义配置
  explicit TerminalHUD(Config cfg)
      : cfg_(std::move(cfg)), last_print_(Clock::now()) {
    // 非交互 TTY 时自动降级：关闭颜色和行刷新
    if (::isatty(STDOUT_FILENO) == 0) {
      cfg_.use_color = false;
      cfg_.use_cr    = false;
    }
  }

  // ── 主接口 ────────────────────────────────────────────────────────────────

  /**
   * @brief 速率限制刷新（每 interval_ms 打印一次）
   *
   * @param fps            整体帧率（由调用方统计，如 MetricsTracker::CurrentFps()）
   * @param dets           本帧检测结果
   * @param ctrl           云台控制指令（nullptr = 无输出）
   * @param node_metrics   各节点线程指标（nullptr 或 empty = 不打印线程行）
   */
  void Update(double fps, const std::vector<mv::Detection>& dets,
              const mv::GimbalControl* ctrl        = nullptr,
              const std::vector<NodeMetrics>* node_metrics = nullptr) {
    const auto now = Clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_).count();
    if (cfg_.interval_ms > 0 && elapsed_ms < cfg_.interval_ms) return;
    last_print_ = now;
    Print(fps, dets, ctrl, node_metrics);
  }

  /**
   * @brief 强制立即打印一次（不受速率限制）
   */
  void Print(double fps, const std::vector<mv::Detection>& dets,
             const mv::GimbalControl* ctrl              = nullptr,
             const std::vector<NodeMetrics>* node_metrics = nullptr) {
    // ── 当前时间字符串 ──────────────────────────────────────────────────────
    const auto now_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    ::localtime_r(&now_t, &tm);
    char timebuf[12];
    std::snprintf(timebuf, sizeof(timebuf), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);

    // ── 主状态行 ─────────────────────────────────────────────────────────────
    std::string line1;
    line1.reserve(160);

    line1 += Bold(Green("[MV]"));
    line1 += ' ';
    line1 += timebuf;
    line1 += " | FPS: ";

    // 帧率：>=60 绿，>=30 黄，否则红
    const std::string fps_str = FmtFixed(fps, 3, 0);
    if (fps >= 60.0) {
      line1 += Green(fps_str);
    } else if (fps >= 30.0) {
      line1 += Yellow(fps_str);
    } else {
      line1 += Red(fps_str);
    }

    // 检测结果
    const auto det_n = static_cast<int>(dets.size());
    line1 += " | DET: ";
    if (det_n > 0) {
      line1 += Green(PadLeft(std::to_string(det_n), 2) + "/ARMOR");
    } else {
      line1 += Gray(" 0      ");
    }

    // 云台控制
    if (ctrl != nullptr) {
      const bool locked = ctrl->tracking;
      line1 += " | LOCK: ";
      line1 += locked ? Green("\xe2\x9c\x93") : Yellow("\xe2\x9c\x97");  // ✓ / ✗ (UTF-8)
      line1 += " | YAW: ";
      line1 += FmtFixed(ctrl->yaw * kRad2Deg, 7, 2);
      line1 += "\xc2\xb0 PITCH: ";  // °
      line1 += FmtFixed(ctrl->pitch * kRad2Deg, 6, 2);
      line1 += "\xc2\xb0 DIST: ";
      line1 += FmtFixed(ctrl->distance, 5, 2);
      line1 += 'm';
      if (ctrl->fire) line1 += Red(" \xe2\x96\xba FIRE");  // ► FIRE
    } else {
      line1 += Gray("  | NO CTRL");
    }

    // ── 线程健康行 ────────────────────────────────────────────────────────────
    const bool has_nodes =
        (node_metrics != nullptr) && (!node_metrics->empty());

    if (cfg_.use_cr) {
      // TTY 模式：主行用 \r 覆写当前行，线程行在下一行
      if (has_nodes) {
        // 先打印线程行（在上一次的线程行位置），再移回主行
        std::printf("\033[2K\r");  // 清除当前行（线程行残留）
        std::printf("%s\n", BuildNodeLine(*node_metrics).c_str());
        std::printf("\033[2K\r%s", line1.c_str());
        std::printf("\033[1A");  // 上移一行，光标停在线程行，下次刷新覆写
      } else {
        std::printf("\r\033[2K%s", line1.c_str());
      }
    } else {
      // 非 TTY 模式：换行追加
      std::printf("%s\n", line1.c_str());
      if (has_nodes) {
        std::printf("%s\n", BuildNodeLine(*node_metrics).c_str());
      }
    }
    std::fflush(stdout);
  }

  /**
   * @brief 打印并换行（退出时调用，保证终端光标在行首）
   */
  void Flush() {
    if (cfg_.use_cr) {
      std::printf("\n");
      std::fflush(stdout);
    }
  }

 private:
  using Clock = std::chrono::steady_clock;

  Config cfg_;
  Clock::time_point last_print_;

  // ── 线程行构造 ────────────────────────────────────────────────────────────

  std::string BuildNodeLine(const std::vector<NodeMetrics>& metrics) const {
    std::string line;
    line.reserve(200);
    line += Bold(Green("[THR]"));
    line += ' ';
    for (const auto& m : metrics) {
      const std::string sname = ShortName(m.node_name);
      if (!m.is_alive) {
        // 节点已死：标红
        line += Red(sname + "\xe2\x9c\x97");  // ✗
        if (!m.error_msg.empty()) {
          line += Red('(' + m.error_msg + ')');
        }
      } else if (m.drop_count > 0 || !m.error_msg.empty()) {
        // 有丢帧或错误：标黄
        line += Yellow(sname + "!" + FmtFixed(m.fps, 3, 0) + "fps"
                       + " drop:" + std::to_string(m.drop_count));
      } else {
        // 健康：标绿
        line += Green(sname + "\xe2\x9c\x93" + FmtFixed(m.fps, 3, 0) + "fps");  // ✓
      }
      line += "  ";
    }
    return line;
  }

  // ── ANSI 颜色辅助（颜色开关统一受 cfg_.use_color 控制）────────────────────

  /// 在字符串两端包裹 ANSI 转义：\033[{code}m ... \033[0m
  /// 若 cfg_.use_color == false（非 TTY / 管道）则直接返回原始字符串，不写入途中控制字符
  std::string Esc(const char* code, const std::string& s) const {
    if (!cfg_.use_color) return s;
    return std::string("\033[") + code + "m" + s + "\033[0m";
  }
  std::string Green(const std::string& s) const  { return Esc("32",   s); }  ///< 绿色（正常 / 通过）
  std::string Yellow(const std::string& s) const { return Esc("33",   s); }  ///< 黄色（警告 / 低帧率）
  std::string Red(const std::string& s) const    { return Esc("31",   s); }  ///< 红色（错误 / 极低帧率 / FIRE）
  std::string Gray(const std::string& s) const   { return Esc("90",   s); }  ///< 灰色（无数据 / 无关信息）
  std::string Bold(const std::string& s) const   { return Esc("1",    s); }  ///< 加粗（樇题标签 [MV] / [THR]）

  // ── 格式化辅助 ────────────────────────────────────────────────────────────

  /// 定点小数格式化：总宽度 width 列，小数保留 prec 位，到 char[32] 缓冲区再转 string
  /// 避免使用 std::to_string（无宽度控制）或 std::ostringstream（频繁分配）
  static std::string FmtFixed(double v, int width, int prec) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%*.*f", width, prec, v);
    return buf;
  }

  /// 左侧空格填充到指定宽度（已达宽度则原样返回）——用于 FPS 数字对齐
  static std::string PadLeft(const std::string& s, int w) {
    if (static_cast<int>(s.size()) >= w) return s;
    return std::string(static_cast<std::size_t>(w) - s.size(), ' ') + s;
  }

  /// 从节点名中去除 "Node" 后缀以缩短显示：
  /// "CaptureNode" → "Capture"，"DetectNode" → "Detect"，无后缀则原样返回
  static std::string ShortName(const std::string& name) {
    const auto pos = name.rfind("Node");
    return (pos != std::string::npos) ? name.substr(0, pos) : name;
  }
};

}  // namespace mv::tool
