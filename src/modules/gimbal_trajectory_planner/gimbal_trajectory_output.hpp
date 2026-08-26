#pragma once

#include <vector>

namespace mv::modules {

/** @brief 单个 MPC 离散时刻求得的双轴角度、角速度和控制输入。 */
struct PlannedGimbalPoint {
  double yaw{0.0};
  double yaw_velocity{0.0};
  double yaw_acceleration{0.0};
  double pitch{0.0};
  double pitch_velocity{0.0};
  double pitch_acceleration{0.0};
};

/** @brief 控制链消费的有效轨迹和正式前视命令。 */
struct GimbalTrajectoryOutput {
  bool valid{false};
  int command_index{1};
  double command_lookahead_s{0.01};
  PlannedGimbalPoint command;
  std::vector<PlannedGimbalPoint> trajectory;
};

}  // namespace mv::modules
