#pragma once

#include "frame/frame_types.hpp"
#include "hal/gimbal/gimbal_types.hpp"
#include "modules/armor_predictor/armor_prediction_output.hpp"
#include "modules/fire_control/fire_control_config.hpp"
#include "modules/gimbal_trajectory_planner/gimbal_trajectory_planner_config.hpp"

#include <memory>

namespace mv::hal {
class IGimbalCommandSink;
}

namespace mv::runtime {

class ControlRuntimeImpl;
class IRuntimeDiagnosticsSink;
class RuntimeSupervisor;

/** @brief 以固定周期消费最新视觉快照并向云台命令后端发布控制结果。 */
class ControlRuntime final {
 public:
  ControlRuntime(modules::FireControlConfig fire_config,
                 modules::GimbalTrajectoryPlannerConfig planner_config,
                 std::unique_ptr<hal::IGimbalCommandSink> sink,
                 IRuntimeDiagnosticsSink* diagnostics, RuntimeSupervisor& supervisor);
  ~ControlRuntime();

  ControlRuntime(const ControlRuntime&) = delete;
  ControlRuntime& operator=(const ControlRuntime&) = delete;
  ControlRuntime(ControlRuntime&&) = delete;
  ControlRuntime& operator=(ControlRuntime&&) = delete;

  /** @brief 启动固定频率控制线程；重复调用不会创建额外线程。 */
  void Start();
  /** @brief 原子替换控制线程下一周期读取的同帧预测与空间快照。 */
  void Update(const modules::ArmorPredictionOutput& prediction,
              const frame::FrameKinematics& kinematics,
              const std::optional<frame::ChassisMotionObservation>& chassis_motion,
              const std::optional<hal::GimbalActuatorTelemetry>& gimbal_actuator);
  /** @brief 停止并等待控制线程，随后向命令后端发送停止命令。 */
  void Stop() noexcept;

 private:
  std::unique_ptr<ControlRuntimeImpl> impl_;
};

}  // namespace mv::runtime
