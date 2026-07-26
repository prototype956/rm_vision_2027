/**
 * @file debug_window.hpp
 * @brief 可复用的单窗口 OpenCV HighGUI 封装。
 */
#pragma once

#include <string>

#include <opencv2/core/mat.hpp>

namespace mv::tool {

/**
 * @brief 调试窗口的尺寸管理模式。
 */
enum class WindowMode {
  AUTOSIZE,  ///< 窗口尺寸跟随显示图像，不允许用户调整。
  NORMAL,    ///< 窗口尺寸可由用户调整。
};

/**
 * @brief 一次窗口事件轮询的结果。
 */
struct WindowEvent {
  int key{-1};                 ///< cv::waitKey() 的原始返回值，-1 表示没有按键。
  bool exit_requested{false};  ///< Q、q、Esc 或关闭窗口时为 true。
};

/**
 * @brief 管理单个 OpenCV 调试窗口的生命周期和输入事件。
 *
 * 典型调用顺序为：构造窗口、每帧调用 Show() 和 Poll()，主循环结束后由析构函数
 * 自动关闭窗口。该类只销毁自己创建的窗口，不会影响进程中的其他 HighGUI 窗口。
 */
class DebugWindow final {
 public:
  /**
   * @brief 创建一个命名调试窗口。
   * @param window_name 窗口标题，必须非空。
   * @param mode 窗口尺寸管理模式，默认跟随图像尺寸。
   * @throws std::invalid_argument 当 window_name 为空时抛出。
   * @throws cv::Exception 当 HighGUI 无法创建窗口时抛出。
   */
  explicit DebugWindow(std::string window_name, WindowMode mode = WindowMode::AUTOSIZE);

  /**
   * @brief 关闭当前窗口并释放其 HighGUI 资源。
   */
  ~DebugWindow() noexcept;

  DebugWindow(const DebugWindow&) = delete;
  DebugWindow& operator=(const DebugWindow&) = delete;
  DebugWindow(DebugWindow&&) = delete;
  DebugWindow& operator=(DebugWindow&&) = delete;

  /**
   * @brief 在窗口中显示一幅图像。
   * @param image 任意非空 OpenCV 图像。
   * @throws std::invalid_argument 当 image 为空时抛出。
   * @throws std::logic_error 当窗口已经关闭时抛出。
   * @throws cv::Exception 当 HighGUI 无法显示图像时抛出。
   */
  void Show(const cv::Mat& image);

  /**
   * @brief 处理窗口事件并读取一次键盘输入。
   *
   * Q、q、Esc 和用户关闭窗口都会设置 WindowEvent::exit_requested。其他按键通过
   * WindowEvent::key 原样返回，调用方可以据此扩展暂停、保存和视图切换等操作。
   *
   * @param delay_ms 等待按键的毫秒数；0 表示一直等待，不能为负数。
   * @return 本次轮询得到的按键和退出状态。
   * @throws std::invalid_argument 当 delay_ms 小于 0 时抛出。
   * @throws cv::Exception 当 HighGUI 事件处理失败时抛出。
   */
  [[nodiscard]] WindowEvent Poll(int delay_ms = 1);

  /**
   * @brief 幂等关闭当前窗口。
   *
   * 此函数不会抛出异常；重复调用不会产生额外效果。
   */
  void Close() noexcept;

  /**
   * @brief 查询此对象管理的窗口是否仍处于打开状态。
   */
  [[nodiscard]] bool IsOpen() const noexcept;

 private:
  std::string window_name_;
  bool is_open_{false};
  bool window_was_visible_{false};
};

}  // namespace mv::tool
