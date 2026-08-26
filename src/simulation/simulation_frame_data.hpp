#pragma once

#include "geometry/armor_type.hpp"
#include "geometry/rigid_transform.hpp"

#include <array>
#include <cstdint>
#include <vector>

#include <optional>

namespace mv::simulation {

/** @brief 仿真器在 world 坐标系中给出的单个机器人真值。 */
struct GroundTruthTarget {
  std::uint64_t id{0};          ///< 本次仿真运行内区分目标的稳定标识。
  std::uint8_t team{0};         ///< 队伍编码：0 为红方，1 为蓝方。
  std::uint8_t armor_label{0};  ///< Talos 协议中的装甲类别编码。
  bool is_outpost{false};       ///< 是否为前哨站等特殊旋转目标。
  geometry::Vector3 position_world{geometry::Vector3::Zero()};  ///< 机器人中心世界位置。
  double yaw{0.0};           ///< 绕 world +Z 轴的航向角，单位为弧度。
  double yaw_velocity{0.0};  ///< 航向角速度，单位为弧度每秒。
};

/** @brief 与图像同帧的单块装甲板三维真值。 */
struct GroundTruthArmor {
  std::uint64_t id{0};
  std::uint8_t team{0};
  std::uint8_t label{0};
  geometry::ArmorType type{geometry::ArmorType::SMALL};
  double width_m{0.0};
  double height_m{0.0};
  geometry::RigidTransform world_t_armor;
  std::array<geometry::Vector3, 4> corners_world{};  ///< TL/TR/BR/BL。
};

/** @brief 与仿真图像同帧采样的弹丸累计统计。 */
struct ProjectileStatistics {
  std::uint64_t bullet_launch_count{0};  ///< 已生成的 17 mm 弹丸累计数。
  std::uint64_t armor_hit_count{0};      ///< 装甲有效碰撞累计数。
  std::uint32_t rune_hit_count{0};       ///< 能量机关有效命中累计数。
  std::uint32_t dart_launch_count{0};    ///< 飞镖发射累计数。
};

/** @brief 与采集图像原子同步的仿真真值和累计统计。 */
struct SimulationFrameData {
  std::optional<ProjectileStatistics> projectile_statistics;  ///< 仿真弹丸累计统计。
  std::vector<GroundTruthTarget> targets;                     ///< 当前机器人真值。
  std::vector<GroundTruthArmor> armors;                       ///< 当前装甲板真值。
};

}  // namespace mv::simulation
