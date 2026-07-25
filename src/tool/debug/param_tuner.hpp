/**
 * @file param_tuner.hpp
 * @brief OpenCV Trackbar 参数调节器（Pimpl 模式）
 *
 * 职责：
 *   - 向已有 OpenCV 窗口附加 Trackbar
 *   - 每帧将滑条值通过 apply() 回调推送到外部 Params 结构
 *   - 程序退出时可将当前参数写入独立 YAML 文件（不污染原始配置）
 *
 * 设计约定：
 *   - 头文件不引入 OpenCV / yaml-cpp，Pimpl 隔离重型依赖
 *   - ParamDesc 中的 apply / get_val 均为 std::function<void(int)> /
 *     std::function<double()>，与具体 Params 结构解耦
 *   - AddParam() 必须在 AttachToWindow() 之后调用，且应在主循环开始前
 *     完成全部注册（Trackbar 整数地址在注册后不可移动）
 */
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <string_view>

namespace mv::tool {

/** @brief 单个可调参数的描述符 */
struct ParamDesc {
  std::string label;     ///< Trackbar 显示标签
  std::string yaml_key;  ///< 写回 YAML 时使用的字段名（如 "light_thresh"）
  int init_val;          ///< 初始整数值（Trackbar 起始位置）
  int max_val;           ///< Trackbar 最大整数值

  /** 将 Trackbar 整数值推送到外部参数结构（每帧调用）*/
  std::function<void(int)> apply;

  /** 获取外部参数的当前值（用于 YAML 写回）*/
  std::function<double()> get_val;
};

/**
 * @brief Trackbar 参数调节器
 *
 * 使用流程：
 * @code
 *   ParamTuner tuner;
 *   tuner.AttachToWindow("my-debug-win");        // 窗口需已 namedWindow
 *   tuner.AddParam({"Thresh", "light_thresh", 160, 255,
 *       [&p](int v){ p.light_thresh = v; },
 *       [&p]{ return (double)p.light_thresh; }});
 *   // 主循环中：
 *   tuner.ApplyAll();                             // 每帧调用
 *   // 退出时：
 *   tuner.SaveTo("configs/debug_override.yaml");
 * @endcode
 */
class ParamTuner {
 public:
  ParamTuner();
  ~ParamTuner();

  ParamTuner(const ParamTuner&) = delete;
  ParamTuner& operator=(const ParamTuner&) = delete;
  ParamTuner(ParamTuner&&) noexcept;
  ParamTuner& operator=(ParamTuner&&) noexcept;

  /** 将 Trackbar 附加到指定窗口（必须先调用 namedWindow）*/
  void AttachToWindow(const std::string& win_name);

  /** 注册一个参数并在已关联窗口上创建对应 Trackbar */
  void AddParam(ParamDesc desc);

  /** 读取所有 Trackbar 当前值并调用对应 apply() 函数 */
  void ApplyAll();

  /**
   * @brief 将当前参数值写入 YAML 文件
   * @param yaml_path  目标文件路径（不存在则创建，存在则合并对应节点）
   * @param section    在 YAML 文件中的节点名（默认 "detector"）
   */
  void SaveTo(const std::string& yaml_path, std::string_view section = "detector") const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::tool
