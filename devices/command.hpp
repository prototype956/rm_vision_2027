#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io {
// 射击模式枚举
enum ShootMode { center_shoot = 0, left_shoot = 1, right_shoot = 2 };

struct Command {
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
};

}  // namespace io

#endif  // IO__COMMAND_HPP
