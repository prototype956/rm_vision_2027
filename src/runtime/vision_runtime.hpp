#pragma once

#include <functional>

namespace mv::hal {
class ICamera;
}

namespace mv::tool {
class DebugWindow;
}

namespace mv::tool::foxglove {
class VisionDebugPublisher;
}

namespace mv::runtime {

class ControlRuntime;
class VisionPipeline;

/** @brief 视觉抓帧循环的终止原因，由应用层映射为稳定进程退出码。 */
enum class VisionRunStatus {
  NORMAL,
  CAMERA_FAILURE,
  PIPELINE_FAILURE,
  CONTROL_FAILURE,
};

/** @brief 驱动相机、单帧感知流水线、调试输出和控制快照交接。 */
class VisionRuntime final {
 public:
  VisionRuntime(hal::ICamera& camera, VisionPipeline& pipeline, ControlRuntime* control,
                tool::DebugWindow* window,
                tool::foxglove::VisionDebugPublisher* diagnostics) noexcept;

  /**
   * @brief 持续处理相机帧，直到收到停止请求、窗口退出或运行时故障。
   * @param stop_requested 每轮抓帧前查询的进程停止条件。
   * @return 供应用层映射退出码的终止原因。
   */
  [[nodiscard]] VisionRunStatus Run(const std::function<bool()>& stop_requested);

 private:
  hal::ICamera& camera_;
  VisionPipeline& pipeline_;
  ControlRuntime* control_{nullptr};
  tool::DebugWindow* window_{nullptr};
  tool::foxglove::VisionDebugPublisher* diagnostics_{nullptr};
};

}  // namespace mv::runtime
