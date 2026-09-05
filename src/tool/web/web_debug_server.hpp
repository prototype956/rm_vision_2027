#pragma once

#include "runtime/runtime_diagnostics_sink.hpp"
#include "runtime/vision_pipeline.hpp"
#include "runtime/vision_tuning.hpp"
#include "tool/web/web_debug_config.hpp"

#include <cstdint>
#include <memory>

#include <filesystem>
#include <optional>

namespace mv::tool::web {

/** @brief 提供认证状态页、JPEG 预览和帧边界临时调参 API。 */
class WebDebugServer final : public runtime::IRuntimeDiagnosticsSink {
 public:
  WebDebugServer(Config config, runtime::VisionTuningMailbox& tuning_mailbox,
                 runtime::VisionPipelineConfig startup_config, std::filesystem::path project_root);
  ~WebDebugServer() override;

  WebDebugServer(const WebDebugServer&) = delete;
  WebDebugServer& operator=(const WebDebugServer&) = delete;

  /** @brief 使用已校验的配置密码启动预览和 HTTP 线程。 */
  [[nodiscard]] bool Start() noexcept;
  /** @brief 幂等停止监听、活动会话和预览编码线程。 */
  void Stop() noexcept;
  [[nodiscard]] bool IsRunning() const noexcept;
  [[nodiscard]] std::uint16_t BoundPort() const noexcept;

  void PublishVision(const frame::FramePacket& packet, const runtime::VisionFrameOutput& output,
                     const runtime::VisionFrameDiagnostics& diagnostics,
                     const std::optional<simulation_evaluation::SimulationEvaluationResult>&
                         evaluation) noexcept override;
  void PublishControl(const runtime::ControlCycleOutput& output,
                      const runtime::ControlCycleDiagnostics& diagnostics) noexcept override;
  [[nodiscard]] runtime::RuntimeDiagnosticsHealth SnapshotHealth() const noexcept override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::tool::web
