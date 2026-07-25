/**
 * @file channel.hpp
 * @brief 带 Shutdown 机制的线程安全有界通道 (Channel<T>)
 *
 * 【为什么不直接用 ThreadSafeQueue？】
 *   原 utils/thread_safe_queue.hpp 的 pop() 在队列为空时阻塞，
 *   没有超时和关闭通知机制，导致 Pipeline 停止时工作线程永久阻塞。
 *
 *   Channel<T> 在此基础上增加：
 *   1. **超时 Pop**：Pop(value, timeout) 在超时时返回 false，
 *      让节点线程每隔一段时间检查停止标志（stop_requested_）；
 *   2. **Shutdown 语义**：Shutdown() 后 Pop 立刻返回 false，
 *      所有等待中的线程被唤醒并退出；
 *   3. **有界溢出策略**：Push 在队列满时丢弃最旧的帧（PopWhenFull），
 *      防止慢节点堆积旧帧导致延迟增大（实时性优先）。
 *
 * 【典型用法】
 * @code
 *   // 创建容量为 4 的帧通道
 *   mv::pipeline::Channel<FramePacket> frame_ch{4};
 *
 *   // 生产者线程（CaptureNode worker）
 *   FramePacket pkt = ...;
 *   frame_ch.Push(std::move(pkt));          // 队满丢最旧帧
 *
 *   // 消费者线程（DetectNode worker）
 *   FramePacket pkt;
 *   while (!stop_flag) {
 *     if (frame_ch.Pop(pkt, 10ms)) {
 *       Process(pkt);
 *     }
 *   }
 *
 *   // 关闭（Pipeline::Stop() 调用）
 *   frame_ch.Shutdown();                    // 唤醒所有 Pop 等待
 * @endcode
 *
 * 【线程安全性】
 *   Push / Pop / Shutdown 均持有 mutex，多线程安全。
 *   允许多生产者多消费者，但 Pipeline 中一般是 1:1 配对。
 *
 * 【性能说明】
 *   - 使用 std::deque + std::mutex + std::condition_variable，无锁自旋；
 *   - 帧图像（cv::Mat）靠引用计数传递，Push/Pop 代价 O(1)；
 *   - 超时精度依赖系统调度精度（通常 1~5ms 抖动），对视觉任务足够。
 */
#pragma once

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>

namespace mv::pipeline {

/**
 * @brief 有界、可关闭的线程安全通道
 *
 * @tparam T       传递的数据类型（需支持移动语义）
 */
template <typename T>
class Channel {
 public:
  /**
   * @param capacity  最大缓冲帧数。建议：
   *                  - CaptureNode → DetectNode：2~4（实时优先）
   *                  - PredictNode → SerialNode：1~2（零延迟优先）
   */
  explicit Channel(size_t capacity) : capacity_(capacity) {}

  // 禁止拷贝/移动（持有互斥量）
  Channel(const Channel&) = delete;
  Channel& operator=(const Channel&) = delete;
  Channel(Channel&&) = delete;
  Channel& operator=(Channel&&) = delete;

  ~Channel() { Shutdown(); }

  // ── Push ─────────────────────────────────────────────────────────────────

  /**
   * @brief 将数据推入通道
   *
   * 策略：队满时丢弃最旧的元素（实时性优先，允许处理掉帧）。
   * Shutdown() 之后调用 Push 视为 no-op，不写入也不报错。
   *
   * @param value  待推入的数据（移动语义，避免深拷贝）
   * @return true = 成功写入；false = 通道已关闭
   */
  bool Push(T value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutdown_) {
        return false;
      }
      if (queue_.size() >= capacity_) {
        queue_.pop_front();  // 丢弃最旧帧
        ++dropped_count_;
      }
      queue_.push_back(std::move(value));
    }
    not_empty_cv_.notify_one();
    return true;
  }

  // ── Pop ──────────────────────────────────────────────────────────────────

  /**
   * @brief 从通道取出数据，带超时
   *
   * @param[out] value    取出的数据
   * @param      timeout  等待超时时间（建议 5~20ms，让节点线程定期检查停止标志）
   * @return true = 成功取出数据；
   *         false = 超时或通道已关闭（调用方应检查停止标志后决定是否重试）
   */
  bool Pop(T& value, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool GOT_DATA = not_empty_cv_.wait_for(lock, timeout, [this] {
      return !queue_.empty() || shutdown_;
    });

    if (!GOT_DATA || queue_.empty()) {
      return false;  // 超时 or Shutdown
    }

    value = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  // ── 控制 ─────────────────────────────────────────────────────────────────

  /**
   * @brief 关闭通道，唤醒所有等待 Pop 的线程
   *
   * 关闭后：
   *   - Push() 返回 false（不再写入）；
   *   - Pop() 立刻返回 false（不再阻塞）；
   *   - 队列中剩余数据被丢弃（可选：改为 drain 模式）。
   */
  void Shutdown() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    not_empty_cv_.notify_all();
  }

  /** @brief 重置通道（清空队列 + 重新开启，用于 Pipeline::Reset()）*/
  void Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    dropped_count_ = 0;
    shutdown_ = false;
  }

  // ── 诊断 ─────────────────────────────────────────────────────────────────

  /** @return 队列当前元素数量（近似值，不加锁）*/
  [[nodiscard]] size_t Size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  /** @return 通道是否已关闭 */
  [[nodiscard]] bool IsShutdown() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shutdown_;
  }

  /** @return 由于队满被丢弃的帧数（用于性能诊断）*/
  [[nodiscard]] uint64_t DroppedCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_count_;
  }

 private:
  // NOLINTNEXTLINE(readability-identifier-naming)
  const size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_cv_;
  std::deque<T> queue_;
  bool shutdown_{false};
  uint64_t dropped_count_{0};
};

}  // namespace mv::pipeline
