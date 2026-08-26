#pragma once

#include "frame/frame_packet.hpp"
#include "modules/fire_control/fire_control.hpp"
#include "runtime/vision_frame_diagnostics.hpp"
#include "runtime/vision_frame_output.hpp"
#include "tool/simulation_evaluation/simulation_evaluation_types.hpp"

#include <cstdint>

#include <optional>

namespace mv::runtime {

/** @brief 控制线程交给 HAL 的单周期正式输出。 */
struct ControlCycleOutput {
  modules::FireControlOutput fire_control;
};

/** @brief 控制线程交给诊断后端的单周期完整诊断。 */
struct ControlCycleDiagnostics {
  modules::FireControlDiagnostics fire_control;
};

/** @brief 诊断后端是否仍可接收数据，以及其累计内部错误数。 */
struct RuntimeDiagnosticsHealth {
  bool available{true};
  std::uint64_t error_count{0};
};

/** @brief 运行时向日志或可视化后端提交诊断的传输无关接口。 */
class IRuntimeDiagnosticsSink {
 public:
  virtual ~IRuntimeDiagnosticsSink() = default;

  virtual void PublishVision(
      const frame::FramePacket& packet, const VisionFrameOutput& output,
      const VisionFrameDiagnostics& diagnostics,
      const std::optional<tool::simulation_evaluation::SimulationEvaluationResult>&
          evaluation) noexcept = 0;

  virtual void PublishControl(const ControlCycleOutput& output,
                              const ControlCycleDiagnostics& diagnostics) noexcept = 0;

  /** @brief 返回传输无关的诊断后端健康快照。 */
  [[nodiscard]] virtual RuntimeDiagnosticsHealth SnapshotHealth() const noexcept = 0;
};

}  // namespace mv::runtime
