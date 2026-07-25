/**
 * @file state_machine.hpp
 * @brief 通用有限状态机模板（Finite State Machine）
 *
 * 【设计约束】
 *   - 仅依赖标准库，不耦合任何业务逻辑；
 *   - StateEnum 必须是 enum / enum class，Context 可以是任意类型；
 *   - 每个状态最多注册三个回调（on_enter / on_exit / on_update），
 *     均可为 nullptr（表示该事件无操作）；
 *   - Transition() 保证：先调用当前状态的 on_exit，再调用目标状态的 on_enter，
 *     中间没有任何与通道/线程相关的操作（由回调自行决定），防止死锁；
 *   - 不支持重入：Transition() 不应在 on_enter/on_exit 内部再次调用
 *     Transition()（会破坏当前状态的一致性），如需链式跳转请在 on_enter 结束后
 *     通过 Update() 驱动。
 *
 * 【典型用法】
 * @code
 *   enum class State { IDLE, RUN, ERROR };
 *
 *   StateMachine<State, MyCtx> fsm;
 *
 *   fsm.Register(State::IDLE, {
 *       .on_enter  = [](MyCtx& c) { c.pipeline->Stop(); },
 *       .on_exit   = nullptr,
 *       .on_update = [](MyCtx& c) { if (c.ready) fsm.Transition(State::RUN, c); }
 *   });
 *
 *   fsm.Init(State::IDLE, ctx);  // 进入初始状态，触发 on_enter
 *
 *   // 主循环
 *   while (running) {
 *       fsm.Update(ctx);         // 触发当前状态的 on_update
 *   }
 *
 *   fsm.Transition(State::ERROR, ctx);  // 强制转换
 * @endcode
 */

#pragma once

#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace mv::fsm {

/**
 * @brief 通用有限状态机
 *
 * @tparam StateEnum  状态枚举类型（需具备 operator== 和 std::hash 特化，
 *                    enum class 已自动满足）
 * @tparam Context    上下文类型，以引用形式传入所有回调
 */
template <typename StateEnum, typename Context>
class StateMachine {
 public:
  /**
   * @brief 单个状态的事件处理器集合
   *
   * 任意字段为 nullptr 时对应事件静默跳过（不抛异常）。
   */
  struct StateHandler {
    /** 进入此状态时调用（在前一个状态的 on_exit 之后）*/
    std::function<void(Context&)> on_enter{nullptr};
    /** 离开此状态时调用（在下一个状态的 on_enter 之前）*/
    std::function<void(Context&)> on_exit{nullptr};
    /** 每次 Update() 调用一次（仅当处于此状态时）*/
    std::function<void(Context&)> on_update{nullptr};
  };

  // ── 禁止拷贝/移动（持有 lambda 时语义模糊）
  StateMachine() = default;
  StateMachine(const StateMachine&) = delete;
  StateMachine& operator=(const StateMachine&) = delete;
  StateMachine(StateMachine&&) = delete;
  StateMachine& operator=(StateMachine&&) = delete;
  ~StateMachine() = default;

  // ── 注册 ──────────────────────────────────────────────────────────────────

  /**
   * @brief 注册一个状态的处理器
   *
   * 可以覆盖注册（后注册的覆盖先注册的）。
   * 必须在 Init() 之前完成所有注册。
   */
  void Register(StateEnum state, StateHandler handler) { handlers_[state] = std::move(handler); }

  // ── 生命周期 ──────────────────────────────────────────────────────────────

  /**
   * @brief 初始化状态机，进入初始状态并触发其 on_enter
   *
   * @throw std::logic_error 如果对应状态未注册
   */
  void Init(StateEnum initial, Context& ctx) {
    if (!handlers_.count(initial)) {
      throw std::logic_error("StateMachine::Init: 初始状态未注册");
    }
    current_ = initial;
    initialized_ = true;
    CallEnter(current_, ctx);
  }

  /**
   * @brief 请求状态转换
   *
   * 调用当前状态的 on_exit → 更新 current_ → 调用目标状态的 on_enter。
   * 如果 next == current_，仍会执行 on_exit + on_enter（重进入语义）。
   *
   * @throw std::logic_error 如果目标状态未注册，或 Init() 尚未调用
   */
  void Transition(StateEnum next, Context& ctx) {
    if (!initialized_) {
      throw std::logic_error("StateMachine::Transition: 请先调用 Init()");
    }
    if (!handlers_.count(next)) {
      throw std::logic_error("StateMachine::Transition: 目标状态未注册");
    }
    CallExit(current_, ctx);
    current_ = next;
    CallEnter(current_, ctx);
  }

  /**
   * @brief 驱动当前状态的 on_update
   *
   * 应在主循环中以固定频率调用。
   * 若 on_update 内部调用 Transition()，状态会立即切换，
   * 本次 Update() 的 on_update 仅会执行切换前状态的逻辑。
   */
  void Update(Context& ctx) {
    if (!initialized_) {
      return;
    }
    auto iter = handlers_.find(current_);
    if (iter != handlers_.end() && iter->second.on_update) {
      iter->second.on_update(ctx);
    }
  }

  // ── 查询 ──────────────────────────────────────────────────────────────────

  /** @return 当前状态（Init() 之前调用时返回值未定义）*/
  [[nodiscard]] StateEnum Current() const noexcept { return current_; }

  /** @return 是否已初始化 */
  [[nodiscard]] bool IsInitialized() const noexcept { return initialized_; }

  /** @return 已注册的状态数量 */
  [[nodiscard]] size_t RegisteredCount() const noexcept { return handlers_.size(); }

 private:
  void CallEnter(StateEnum state, Context& ctx) {
    auto iter = handlers_.find(state);
    if (iter != handlers_.end() && iter->second.on_enter) {
      iter->second.on_enter(ctx);
    }
  }

  void CallExit(StateEnum state, Context& ctx) {
    auto iter = handlers_.find(state);
    if (iter != handlers_.end() && iter->second.on_exit) {
      iter->second.on_exit(ctx);
    }
  }

  std::unordered_map<StateEnum, StateHandler> handlers_;
  StateEnum current_{};
  bool initialized_{false};
};

}  // namespace mv::fsm
