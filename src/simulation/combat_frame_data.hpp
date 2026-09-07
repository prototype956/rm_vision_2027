#pragma once

#include <cstddef>
#include <cstdint>

namespace mv::simulation {

inline constexpr std::size_t COMBAT_MAX_ROBOTS = 16;
inline constexpr std::size_t COMBAT_MAX_EVENTS = 64;

/** @brief 机器人裁判属性及独立评估计数；编码和单位见 docs/test/talos_combat.md。 */
struct alignas(32) RobotCombatMeta {
  std::uint64_t robot_id{};
  std::uint32_t hp{};
  std::uint32_t max_hp{};
  float heat{};
  float heat_limit{};
  float cooling_per_second{};
  std::uint32_t allowance_remaining{};
  std::uint32_t fire_blocks{};
  std::uint8_t team{};
  std::uint8_t role{};
  std::uint8_t life{};
  std::uint8_t shooter{};
  std::uint8_t allowance_mode{};
  std::uint8_t fire_permitted{};
  std::uint8_t pad1[6]{};
  std::uint64_t actual_shots{};
  std::uint64_t rejected_requests{};
  std::uint64_t damage_dealt{};
  std::uint64_t damage_taken{};
  std::uint64_t kills{};
  std::uint64_t armor_contacts{};
  std::uint64_t damaging_hits{};
  std::uint64_t heat_lock_count{};
  double heat_locked_s{};
  std::uint8_t pad2[8]{};
};

/** @brief 有界事件；时间为回合内仿真纳秒，detail 为 NUL 结尾 UTF-8，含原因及参与者。 */
struct alignas(64) CombatEventMeta {
  std::uint64_t id{};
  std::uint64_t round_id{};
  std::uint64_t round_time_ns{};
  std::uint8_t detail[232]{};
};

/** @brief 经过 HAL 校验的 v7 同帧附件；绝对仿真时间与 10 Hz 采样时间独立。 */
struct alignas(64) CombatFrameMeta {
  std::uint64_t round_id{};
  std::uint64_t sim_time_ns{};
  std::uint64_t round_started_ns{};
  std::uint64_t referee_sample_ns{};
  std::uint64_t referee_sample_sequence{};
  std::uint64_t events_dropped{};
  std::uint32_t robot_count{};
  std::uint32_t event_count{};
  std::uint8_t referee_valid{};
  std::uint8_t pad[7]{};
  RobotCombatMeta self_referee{};
  RobotCombatMeta robots[COMBAT_MAX_ROBOTS]{};
  CombatEventMeta events[COMBAT_MAX_EVENTS]{};
};

static_assert(sizeof(RobotCombatMeta) == 128);
static_assert(sizeof(CombatEventMeta) == 256);
static_assert(sizeof(CombatFrameMeta) == 18624);
static_assert(offsetof(CombatFrameMeta, self_referee) == 64);
static_assert(offsetof(CombatFrameMeta, events) == 2240);
}  // namespace mv::simulation
