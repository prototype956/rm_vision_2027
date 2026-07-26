#include "tool/debug/debug_window.hpp"

#include <stdexcept>
#include <utility>

#include <opencv2/highgui.hpp>

namespace mv::tool {
namespace {

constexpr int K_ESCAPE_KEY = 27;

int ToOpenCvWindowFlag(WindowMode mode) noexcept {
  return mode == WindowMode::NORMAL ? cv::WINDOW_NORMAL : cv::WINDOW_AUTOSIZE;
}

bool IsExitKey(int key) noexcept {
  return key == K_ESCAPE_KEY || key == 'q' || key == 'Q';
}

}  // namespace

DebugWindow::DebugWindow(std::string window_name, WindowMode mode)
    : window_name_(std::move(window_name)) {
  if (window_name_.empty()) {
    throw std::invalid_argument("debug window name must not be empty");
  }

  cv::namedWindow(window_name_, ToOpenCvWindowFlag(mode));
  is_open_ = true;
}

DebugWindow::~DebugWindow() noexcept {
  Close();
}

void DebugWindow::Show(const cv::Mat& image) {
  if (!is_open_) {
    throw std::logic_error("cannot show an image in a closed debug window");
  }
  if (image.empty()) {
    throw std::invalid_argument("debug window image must not be empty");
  }

  cv::imshow(window_name_, image);
}

WindowEvent DebugWindow::Poll(int delay_ms) {
  if (delay_ms < 0) {
    throw std::invalid_argument("debug window poll delay must not be negative");
  }
  if (!is_open_) {
    return {.key = -1, .exit_requested = true};
  }

  WindowEvent event;
  event.key = cv::waitKey(delay_ms);
  if (IsExitKey(event.key)) {
    event.exit_requested = true;
    return event;
  }

  // GTK 在窗口创建后的短暂映射阶段可能返回 0。只有窗口曾经可见，之后再变为
  // 不可见，才把它视为用户关闭，避免新窗口在第一次 Poll() 时被误判为关闭。
  try {
    const double VISIBILITY = cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE);
    if (VISIBILITY >= 1.0) {
      window_was_visible_ = true;
    } else if (window_was_visible_) {
      is_open_ = false;
      event.exit_requested = true;
    }
  } catch (const cv::Exception&) {
    // 某些后端在窗口被用户销毁后直接抛出异常。启动阶段仍需等待首次可见，
    // 已经可见过的窗口则可安全地判定为关闭。
    if (window_was_visible_) {
      is_open_ = false;
      event.exit_requested = true;
    }
  }
  return event;
}

void DebugWindow::Close() noexcept {
  if (!is_open_) {
    return;
  }

  try {
    cv::destroyWindow(window_name_);
  } catch (const cv::Exception&) {
    // 析构和显式清理都不能因 HighGUI 后端状态而传播异常。
  }
  is_open_ = false;
}

bool DebugWindow::IsOpen() const noexcept {
  return is_open_;
}

}  // namespace mv::tool
