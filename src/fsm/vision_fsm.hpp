/**
 * @file vision_fsm.hpp
 * @brief 视觉系统状态机（VisionFSM）
 *
 * 【系统状态图】
 *
 * @code
 *                     ┌──────────────────────────────────┐
 *                     │              IDLE                │
 *                     │   pipeline 停止,等待 Start()     │
 *                     └──────────────┬───────────────────┘
 *                                    │ Start()
 *                                    ▼
 *                     ┌──────────────────────────────────┐
 *                     │              INIT                │
 *                     │   pipeline.Start() → 等待稳定    │
 *                     └──────┬───────────────────┬───────┘
 *               AUTO_AIM ◄───┘                   └──► ENERGY_BUFF
 *      (上行 mode=0)  │                                  │  (上行 mode=1)
 *            ◄────────┘                          ────────►
 *       (轮询 CheckErrors, 上行 mode)       (轮询 CheckErrors, 上行 mode)
 *                     │  HasError                   HasError │
 *                     └──────────────┬──────────────────────┘
 *                                    ▼
 *                     ┌──────────────────────────────────┐
 *                     │              ERROR               │
 *                     │   pipeline.Stop(), 记录错误码    │
 *                     └──────────────┬───────────────────┘
 *                                    │ Recover() / 超时重试
 *                                    ▼
 *                     ┌──────────────────────────────────┐
 *                     │            RECOVERY              │
 *                     │   pipeline.Reset() → INIT        │
 *                     └──────────────────────────────────┘
 * @endcode
 *
 * 【线程安全说明】
 *   VisionFSM 对象本身不是线程安全的。
 *   应在单一线程（主循环线程）调用 Update() 和 RequestTransition()，
 *   pipeline 的线程竞争由其内部机制保证。
 *
 * 【典型用法（main.cpp）】
 * @code
 *   auto pipeline = VisionPipeline::Builder{}
 *       .Camera(...).Detector(...).Build();
 *
 *   mv::fsm::VisionFSM fsm(std::move(pipeline));
 *   fsm.Start();    // IDLE → INIT → AUTO_AIM
 *
 *   while (true) {
 *       fsm.Update();   // 驱动一次状态机
 *       if (!fsm.IsRunning()) break;
 *       std::this_thread::sleep_for(std::chrono::milliseconds(10));
 *   }
 *   fsm.Stop();     // 任意状态 → IDLE
 * @endcode
 */
#pragma once

#include "pipeline/pipeline.hpp"
#include "state_machine.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace mv::fsm {

// ============================================================================
// 系统状态枚举
// ============================================================================

/**
 * @brief 视觉系统运行状态
 *
 * 每个值对应一个明确的系统行为阶段。
 * 当前版本不包含 CALIBRATION，可在 Stage 6 中扩展。
 */
enum class SystemState : uint8_t {
  IDLE = 0,     ///< 初始/停止状态，Pipeline 未运行
  INIT,         ///< Pipeline 正在启动中
  AUTO_AIM,     ///< 自瞄模式运行中（上行 mode = 0）
  ENERGY_BUFF,  ///< 打符模式运行中（上行 mode = 1，预留）
  ERROR,        ///< 错误状态，Pipeline 已停止，等待恢复
  RECOVERY,     ///< 恢复中（Reset + 重新进入 INIT）
};

/** @brief 状态名称字符串（日志输出用）*/
inline const char* SystemStateName(SystemState state) noexcept {
  switch (state) {
    case SystemState::IDLE:
      return "IDLE";
    case SystemState::INIT:
      return "INIT";
    case SystemState::AUTO_AIM:
      return "AUTO_AIM";
    case SystemState::ENERGY_BUFF:
      return "ENERGY_BUFF";
    case SystemState::ERROR:
      return "ERROR";
    case SystemState::RECOVERY:
      return "RECOVERY";
    default:
      return "UNKNOWN";
  }
}

// ============================================================================
// 系统上下文
// ============================================================================

/**
 * @brief 传入所有状态处理器的上下文对象
 *
 * StateMachine 持有对此结构的引用，所有状态回调通过 Context& 读写共享信息。
 */
struct SystemContext {
  // ── 持有 pipeline 所有权 ─────────────────────────────────────────────────
  std::unique_ptr<pipeline::VisionPipeline> pipeline;

  // ── 状态转换请求 ──────────────────────────────────────────────────────────
  /**
   * 外部/Update 内部通过写 requested_state 请求跳转，
   * VisionFSM::Update() 在每次迭代结束前检查并执行。
   * 与 current_ 相同时表示"无请求"。
   * 不使用 std::optional 以避免模板复杂性。
   */
  SystemState requested_state{SystemState::IDLE};
  bool has_pending_transition{false};

  // ── 诊断信息 ──────────────────────────────────────────────────────────────
  int last_error_code{0};
  int recovery_attempts{0};               ///< 当前故障的累计恢复尝试次数
  static constexpr int MAX_RECOVERY = 3;  ///< 超过此次数不再自动恢复

  // ── Init 阶段稳定等待 ─────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point init_enter_time{};
  static constexpr auto INIT_STABILIZE_MS = std::chrono::milliseconds(200);

  // ── Error 阶段等待 ────────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point error_enter_time{};
  static constexpr auto ERROR_WAIT_MS = std::chrono::milliseconds(500);
};

// ============================================================================
// 状态处理器前向声明（在 vision_fsm.cpp 中实现）
// ============================================================================

namespace handlers {
void IdleOnEnter(SystemContext& ctx);
void IdleOnUpdate(SystemContext& ctx);

void InitOnEnter(SystemContext& ctx);
void InitOnUpdate(SystemContext& ctx);

void AutoAimOnEnter(SystemContext& ctx);
void AutoAimOnUpdate(SystemContext& ctx);
void AutoAimOnExit(SystemContext& ctx);

void EnergyBuffOnEnter(SystemContext& ctx);
void EnergyBuffOnUpdate(SystemContext& ctx);
void EnergyBuffOnExit(SystemContext& ctx);

void ErrorOnEnter(SystemContext& ctx);
void ErrorOnUpdate(SystemContext& ctx);

void RecoveryOnEnter(SystemContext& ctx);
void RecoveryOnUpdate(SystemContext& ctx);
}  // namespace handlers

// ============================================================================
// VisionFSM
// ============================================================================

/**
 * @brief 视觉系统顶层状态机
 *
 * 对外封装 StateMachine<SystemState, SystemContext>，
 * 提供面向业务的简洁接口。
 *
 * 不可拷贝/移动（持有 pipeline unique_ptr）。
 */
class VisionFSM {
 public:
  /**
   * @brief 构造函数，注入 pipeline 所有权
   *
   * @param pipeline   由 VisionPipeline::Builder::Build() 生成的 pipeline。
   *                   不可为 nullptr（直接 terminate）。
   */
  explicit VisionFSM(std::unique_ptr<pipeline::VisionPipeline> pipeline);

  VisionFSM(const VisionFSM&) = delete;
  VisionFSM& operator=(const VisionFSM&) = delete;
  VisionFSM(VisionFSM&&) = delete;
  VisionFSM& operator=(VisionFSM&&) = delete;

  ~VisionFSM();

  // ── 生命周期 ──────────────────────────────────────────────────────────────

  /**
   * @brief 启动状态机：IDLE → INIT（随后自动流转至 AUTO_AIM）
   *
   * 调用前 FSM 必须处于 IDLE 状态（构造后默认如此）。
   */
  void Start();

  /**
   * @brief 停止状态机：强制转换到 IDLE（会停止 Pipeline）
   *
   * 可以在任意状态调用，幂等。
   */
  void Stop();

  /**
   * @brief 主循环驱动函数，每 10~50ms 调用一次
   *
   * 驱动当前状态的 on_update，并在 on_update 结束后处理 pending transition。
   */
  void Update();

  // ── 外部状态干预 ──────────────────────────────────────────────────────────

  /**
   * @brief 外部请求状态转换（线程不安全，只应在 Update() 同一线程调用）
   *
   * 请求会在下一次 Update() 末尾被处理。
   */
  void RequestTransition(SystemState next);

  /**
   * @brief 外部请求进入错误恢复（等价于 RequestTransition(RECOVERY)）
   */
  void Recover();

  // ── 查询 ──────────────────────────────────────────────────────────────────

  /** @return 当前状态 */
  [[nodiscard]] SystemState CurrentState() const noexcept;

  /**
   * @return 是否处于正常运行态（AUTO_AIM 或 ENERGY_BUFF）
   *
   * 可供主循环判断是否继续等待：
   * @code
   *   while (fsm.IsRunning()) { fsm.Update(); ... }
   * @endcode
   */
  [[nodiscard]] bool IsRunning() const noexcept;

  /** @return 是否处于错误态 */
  [[nodiscard]] bool HasError() const noexcept;

  /** @return 最近的错误码（0 = 无错误）*/
  [[nodiscard]] int LastErrorCode() const noexcept;

  /** @return 直接访问 Pipeline 诊断信息（只读）*/
  [[nodiscard]] const pipeline::VisionPipeline* Pipeline() const noexcept;

 private:
  /** 注册所有状态处理器，在构造函数中调用一次 */
  void RegisterHandlers();

  StateMachine<SystemState, SystemContext> sm_;
  SystemContext ctx_;
};

}  // namespace mv::fsm
