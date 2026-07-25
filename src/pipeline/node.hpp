/**
 * @file node.hpp
 * @brief Pipeline 节点抽象基类 (PipelineNode)
 *
 * 【设计目标】
 *   每个 PipelineNode 代表流水线中的一个处理阶段，负责：
 *   1. 管理自身工作线程的生命周期（Start / Stop）；
 *   2. 从输入通道取数据（Pop，带超时以便检查停止标志）；
 *   3. 处理数据（派生类实现 WorkLoop 或 Process）；
 *   4. 将结果推入输出通道（Push）；
 *   5. 记录性能统计（处理帧数、掉帧数、平均延迟）。
 *
 * 【线程模型】
 *   每个节点拥有一个独立的 std::thread，由 Start() 启动、Stop() 停止。
 *   节点间通过 Channel<T> 通信，Channel 满时丢弃最旧帧（实时优先策略）。
 *
 * 【停止协议】
 *   Stop() 的执行顺序（从最末节点向前逆序调用）：
 *   1. 设置 stop_requested_ = true；
 *   2. Shutdown 对应的输入 Channel（让 Pop 立刻返回 false）；
 *   3. join 工作线程。
 *
 *   Pipeline 负责按正确顺序调用各节点的 Stop()，
 *   CaptureNode 的 Stop() 还需关闭相机。
 *
 * 【错误处理】
 *   WorkLoop 中若发生不可恢复的错误（如相机断开、串口断开），
 *   节点将 error_code_ 置为非零值，
 *   Pipeline 定期检查 HasError() 并触发状态机的 ERROR 转换（Stage 5 实现）。
 *
 * 【派生类约定】
 *   - 必须实现纯虚函数 WorkLoop()；
 *   - WorkLoop() 内用 stop_requested_.load() 判断是否退出；
 *   - WorkLoop() 不应抛出异常，异常用日志记录并设置 error_code_。
 */
#pragma once

#include <atomic>
#include <string>
#include <thread>

namespace mv::pipeline {

/**
 * @brief Pipeline 节点抽象基类
 *
 * 提供线程生命周期管理和基础诊断接口，
 * 派生类实现具体的 WorkLoop() 逻辑。
 */
class PipelineNode {
 public:
  explicit PipelineNode(std::string name) : name_(std::move(name)) {}

  // 禁止拷贝（拥有线程和原子变量）
  PipelineNode(const PipelineNode&) = delete;
  PipelineNode& operator=(const PipelineNode&) = delete;
  PipelineNode(PipelineNode&&) = delete;
  PipelineNode& operator=(PipelineNode&&) = delete;

  virtual ~PipelineNode() { Stop(); }

  // ── 生命周期 ─────────────────────────────────────────────────────────────

  /**
   * @brief 启动节点工作线程
   *
   * 若节点已在运行，此调用为 no-op。
   * 派生类可重载以在启动线程前做额外初始化（如打开相机），
   * 但必须在最后调用 `PipelineNode::Start()`（链式调用）。
   */
  virtual void Start() {
    if (running_.load()) {
      return;
    }
    stop_requested_.store(false);
    error_code_.store(0);
    running_.store(true);
    worker_ = std::thread(&PipelineNode::WorkLoop, this);
  }

  /**
   * @brief 停止节点工作线程（阻塞直到线程退出）
   *
   * 调用后节点可以重新 Start()。
   * 派生类可重载以在线程退出后做清理（如关闭相机），
   * 但必须在最前调用 `PipelineNode::Stop()`。
   */
  virtual void Stop() {
    stop_requested_.store(true);
    OnStop();  // 通知派生类（如 Shutdown 输入 Channel）
    if (worker_.joinable()) {
      worker_.join();
    }
    running_.store(false);
  }

  // ── 诊断 ─────────────────────────────────────────────────────────────────

  /** @return 节点名称（用于日志前缀）*/
  [[nodiscard]] const std::string& Name() const noexcept { return name_; }

  /** @return 节点是否正在运行 */
  [[nodiscard]] bool IsRunning() const noexcept { return running_.load(); }

  /** @return 是否发生不可恢复错误 */
  [[nodiscard]] bool HasError() const noexcept { return error_code_.load() != 0; }

  /** @return 错误码（0 = 无错误）*/
  [[nodiscard]] int ErrorCode() const noexcept { return error_code_.load(); }

  /** @return 已处理的数据包数量 */
  [[nodiscard]] uint64_t ProcessedCount() const noexcept { return processed_count_.load(); }

 protected:
  // ── 派生类实现 ────────────────────────────────────────────────────────────

  /**
   * @brief 工作线程主函数（纯虚，派生类实现）
   *
   * 典型实现模板：
   * @code
   *   void WorkLoop() override {
   *     while (!stop_requested_.load()) {
   *       InputT pkt;
   *       if (!input_ch_->Pop(pkt, kPopTimeout)) {
   *         continue;  // 超时或 Shutdown，重新检查 stop_requested_
   *       }
   *       auto result = Process(pkt);
   *       if (result && output_ch_) {
   *         output_ch_->Push(std::move(*result));
   *       }
   *       ++processed_count_;
   *     }
   *   }
   * @endcode
   */
  virtual void WorkLoop() = 0;

  /**
   * @brief Stop() 调用时的钩子（派生类用于 Shutdown Channel）
   *
   * 示例（DetectNode）：
   * @code
   *   void OnStop() override { input_ch_->Shutdown(); }
   * @endcode
   */
  virtual void OnStop() {}

  // ── 派生类便捷访问方法（避免暴露原子成员为 protected）──────────────────────

  /** @brief WorkLoop() 内检查是否应退出 */
  [[nodiscard]] bool ShouldStop() const noexcept { return stop_requested_.load(); }

  /** @brief 设置不可恢复错误码（WorkLoop 内调用，触发 HasError()）*/
  void SetError(int code) noexcept { error_code_.store(code); }

  /** @brief 递增已处理包计数 */
  void IncrementProcessed() noexcept { ++processed_count_; }

 private:
  std::string name_;
  std::thread worker_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
  std::atomic<int> error_code_{0};
  std::atomic<uint64_t> processed_count_{0};
};

}  // namespace mv::pipeline
