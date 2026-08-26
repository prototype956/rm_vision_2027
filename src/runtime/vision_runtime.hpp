#pragma once

#include "runtime/runtime_supervisor.hpp"

#include <functional>

namespace mv::hal {
class ICamera;
}

namespace mv::tool {
class DebugWindow;
}

namespace mv::tool::simulation_evaluation {
class SimulationEvaluator;
}

namespace mv::runtime {

class ControlRuntime;
class IRuntimeDiagnosticsSink;
class VisionPipeline;

/** @brief 驱动相机、单帧感知流水线、调试输出和控制快照交接。 */
class VisionRuntime final {
 public:
  VisionRuntime(hal::ICamera& camera, VisionPipeline& pipeline, ControlRuntime* control,
                tool::DebugWindow* window, IRuntimeDiagnosticsSink* diagnostics,
                tool::simulation_evaluation::SimulationEvaluator* evaluator,
                RuntimeSupervisor& supervisor) noexcept;

  /**
   * @brief 持续处理相机帧，直到收到停止请求、窗口退出或运行时故障。
   * @param stop_requested 每轮抓帧前查询的进程停止条件。
   * @return 供应用层映射退出码的终止原因。
   */
  [[nodiscard]] RuntimeRunResult Run(const std::function<bool()>& stop_requested);

 private:
  hal::ICamera& camera_;
  VisionPipeline& pipeline_;
  ControlRuntime* control_{nullptr};
  tool::DebugWindow* window_{nullptr};
  IRuntimeDiagnosticsSink* diagnostics_{nullptr};
  tool::simulation_evaluation::SimulationEvaluator* evaluator_{nullptr};
  RuntimeSupervisor& supervisor_;
};

}  // namespace mv::runtime
