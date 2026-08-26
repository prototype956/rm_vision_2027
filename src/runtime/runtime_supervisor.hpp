#pragma once

#include "runtime/runtime_error_policy.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include <optional>

namespace mv::runtime {

/** @brief 运行时健康监督覆盖的稳定组件集合。 */
enum class RuntimeComponent : std::uint8_t {
  CAMERA = 0,
  VISION_PIPELINE,
  CONTROL_RUNTIME,
  COMMAND_SINK,
  SIMULATION_EVALUATION,
  DEBUG_WINDOW,
  DIAGNOSTICS,
  COUNT,
};

/** @brief 运行时故障的稳定机器可读原因。 */
enum class RuntimeFaultCode : std::uint8_t {
  CAMERA_TIMEOUT = 0,
  CAMERA_INVALID_FRAME,
  CAMERA_DISCONNECTED,
  CAMERA_FATAL,
  VISION_PIPELINE_EXCEPTION,
  CONTROL_UPDATE_EXCEPTION,
  CONTROL_THREAD_EXCEPTION,
  COMMAND_SINK_UNHEALTHY,
  COMMAND_SEND_FAILED,
  EVALUATION_INIT_FAILURE,
  EVALUATION_EXCEPTION,
  DEBUG_WINDOW_INIT_FAILURE,
  DEBUG_WINDOW_EXCEPTION,
  DIAGNOSTICS_INIT_FAILURE,
  DIAGNOSTICS_UNAVAILABLE,
  DIAGNOSTICS_EXCEPTION,
};

/** @brief 组件及进程的统一健康级别。 */
enum class RuntimeHealthState : std::uint8_t { HEALTHY = 0, DEGRADED, FAILED };

/** @brief 对一次错误采取的主要处置；组合动作由 RuntimeDecision 的布尔字段表达。 */
enum class RuntimeFaultAction : std::uint8_t {
  CONTINUE = 0,
  SKIP_CURRENT_WORK,
  DISABLE_COMPONENT,
  SAFE_STOP,
  TERMINATE,
  SAFE_STOP_AND_TERMINATE,
};

/** @brief 进程结束原因，由应用层映射到既有退出码。 */
enum class RuntimeTerminationReason : std::uint8_t {
  NORMAL = 0,
  CAMERA_FAILURE,
  PIPELINE_FAILURE,
  CONTROL_FAILURE,
};

/** @brief 一项活动或锁存的运行时故障。 */
struct RuntimeFault {
  RuntimeComponent component{RuntimeComponent::CAMERA};
  RuntimeFaultCode code{RuntimeFaultCode::CAMERA_TIMEOUT};
  RuntimeHealthState state{RuntimeHealthState::HEALTHY};
  RuntimeFaultAction action{RuntimeFaultAction::CONTINUE};
  std::uint64_t total_occurrences{0};
  std::uint64_t consecutive_occurrences{0};
  std::chrono::steady_clock::time_point first_seen{};
  std::chrono::steady_clock::time_point last_seen{};
  std::string detail;
};

/** @brief 监督器对调用方当前错误给出的无歧义处置。 */
struct RuntimeDecision {
  RuntimeHealthState state{RuntimeHealthState::HEALTHY};
  bool skip_current_work{false};
  bool disable_component{false};
  bool request_safe_stop{false};
  std::optional<RuntimeTerminationReason> termination;
};

/** @brief 单个组件的线程安全健康快照。 */
struct RuntimeComponentHealth {
  RuntimeComponent component{RuntimeComponent::CAMERA};
  RuntimeHealthState state{RuntimeHealthState::HEALTHY};
  std::uint64_t total_events{0};
  std::optional<RuntimeFault> active_fault;
};

constexpr std::size_t K_RUNTIME_COMPONENT_COUNT = static_cast<std::size_t>(RuntimeComponent::COUNT);

/** @brief 全部运行时组件及终止故障的只读快照。 */
struct RuntimeHealthSnapshot {
  RuntimeHealthState overall{RuntimeHealthState::HEALTHY};
  std::array<RuntimeComponentHealth, K_RUNTIME_COMPONENT_COUNT> components{};
  std::optional<RuntimeFault> terminal_fault;
};

/** @brief VisionRuntime 返回给应用层的统一运行结果。 */
struct RuntimeRunResult {
  RuntimeTerminationReason reason{RuntimeTerminationReason::NORMAL};
  std::optional<RuntimeFault> fault;
};

[[nodiscard]] const char* RuntimeComponentName(RuntimeComponent component) noexcept;
[[nodiscard]] const char* RuntimeFaultCodeName(RuntimeFaultCode code) noexcept;
[[nodiscard]] const char* RuntimeFaultActionName(RuntimeFaultAction action) noexcept;
/** @brief 将运行时终止原因映射到既有进程退出码。 */
[[nodiscard]] int RuntimeExitCode(RuntimeTerminationReason reason) noexcept;

/** @brief 线程安全地应用错误策略、跟踪恢复并锁存首个终止故障。 */
class RuntimeSupervisor final {
 public:
  explicit RuntimeSupervisor(RuntimeErrorPolicy policy);
  ~RuntimeSupervisor();

  RuntimeSupervisor(const RuntimeSupervisor&) = delete;
  RuntimeSupervisor& operator=(const RuntimeSupervisor&) = delete;
  RuntimeSupervisor(RuntimeSupervisor&&) = delete;
  RuntimeSupervisor& operator=(RuntimeSupervisor&&) = delete;

  /** @brief 使用当前单调时钟报告一次错误。 */
  [[nodiscard]] RuntimeDecision Report(RuntimeFaultCode code,
                                       std::string_view detail = {}) noexcept;
  /** @brief 使用显式时间报告错误，供运行时已有时钟和确定性验收使用。 */
  [[nodiscard]] RuntimeDecision ReportAt(RuntimeFaultCode code, std::string_view detail,
                                         std::chrono::steady_clock::time_point now) noexcept;
  /** @brief 使用当前单调时钟清除组件的可恢复故障并更新最后健康时间。 */
  void Recover(RuntimeComponent component) noexcept;
  /** @brief 使用显式时间清除可恢复故障。 */
  void RecoverAt(RuntimeComponent component, std::chrono::steady_clock::time_point now) noexcept;

  [[nodiscard]] RuntimeHealthSnapshot Snapshot() const noexcept;
  [[nodiscard]] std::optional<RuntimeRunResult> TerminalResult() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::runtime
