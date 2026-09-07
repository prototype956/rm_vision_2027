#include "tool/foxglove/simulation/combat_message_encoder.hpp"

#include <array>
#include <string_view>

#include <nlohmann/json.hpp>

namespace mv::tool::foxglove::simulation {
namespace {
using Json = nlohmann::json;
constexpr std::array<std::string_view, 8> BLOCK_NAMES{
    "Dead",           "NoShooter",          "HeatCoolingLock", "HeatRoundLock",
    "AllowanceEmpty", "MechanicalInterval", "FeedBusy",        "MissingOrInvalidMuzzle"};

Json Robot(const mv::simulation::RobotCombatMeta& state, bool referee_only) {
  Json reasons = Json::array();
  for (std::size_t i = 0; i < BLOCK_NAMES.size(); ++i) {
    if ((state.fire_blocks & (1U << i)) != 0)
      reasons.push_back(BLOCK_NAMES[i]);
  }
  Json result{{"robot_id", state.robot_id},
              {"team", state.team == 0 ? "red" : "blue"},
              {"role", std::array{"infantry", "sentry", "hero_target"}[state.role]},
              {"hp", state.hp},
              {"max_hp", state.max_hp},
              {"alive", state.life == 0},
              {"shooter", state.shooter == 1 ? "17mm" : "none"},
              {"heat", state.heat},
              {"heat_limit", state.heat_limit},
              {"cooling_per_second", state.cooling_per_second},
              {"allowance_mode", state.allowance_mode == 0 ? "unlimited" : "limited"},
              {"allowance_remaining",
               state.allowance_mode == 0 ? Json(nullptr) : Json(state.allowance_remaining)},
              {"fire_permitted", state.fire_permitted != 0},
              {"fire_blocks", state.fire_blocks},
              {"fire_reasons", std::move(reasons)}};
  if (!referee_only) {
    result.update(Json{{"actual_shots", state.actual_shots},
                       {"rejected_requests", state.rejected_requests},
                       {"damage_dealt", state.damage_dealt},
                       {"damage_taken", state.damage_taken},
                       {"kills", state.kills},
                       {"armor_contacts", state.armor_contacts},
                       {"damaging_hits", state.damaging_hits},
                       {"heat_lock_count", state.heat_lock_count},
                       {"heat_locked_s", state.heat_locked_s}});
  }
  return result;
}

Json ObjectSchema(Json properties) {
  return Json{{"type", "object"}, {"properties", std::move(properties)}};
}
Json RobotSchema(bool referee_only) {
  Json properties;
  for (const auto* field : {"robot_id", "hp", "max_hp", "fire_blocks"})
    properties[field] = {{"type", "integer"}};
  for (const auto* field : {"heat", "heat_limit", "cooling_per_second"})
    properties[field] = {{"type", "number"}};
  for (const auto* field : {"team", "role", "shooter", "allowance_mode"})
    properties[field] = {{"type", "string"}};
  for (const auto* field : {"alive", "fire_permitted"})
    properties[field] = {{"type", "boolean"}};
  properties["allowance_remaining"] = {{"type", {"integer", "null"}}};
  properties["fire_reasons"] = {{"type", "array"}, {"items", {{"type", "string"}}}};
  if (!referee_only) {
    for (const auto* field : {"actual_shots", "rejected_requests", "damage_dealt", "damage_taken",
                              "kills", "armor_contacts", "damaging_hits", "heat_lock_count"})
      properties[field] = {{"type", "integer"}};
    properties["heat_locked_s"] = {{"type", "number"}};
  }
  return ObjectSchema(std::move(properties));
}
}  // namespace

std::string EncodeCombat(const mv::simulation::CombatFrameMeta& combat, std::uint64_t sequence,
                         const ::foxglove::schemas::Timestamp& timestamp, bool referee_only) {
  Json result{
      {"timestamp", {{"sec", timestamp.sec}, {"nsec", timestamp.nsec}}},
      {"sequence", sequence},
      {"round_id", combat.round_id},
      {"sim_time_ns", combat.sim_time_ns},
      {"round_started_ns", combat.round_started_ns},
      {"round_time_s", static_cast<double>(combat.sim_time_ns - combat.round_started_ns) * 1e-9}};
  if (referee_only) {
    result["sample_time_ns"] = combat.referee_sample_ns;
    result["sample_sequence"] = combat.referee_sample_sequence;
    result["valid"] = combat.referee_valid != 0;
    result["self"] = combat.referee_valid ? Robot(combat.self_referee, true) : Json(nullptr);
  } else {
    result["robots"] = Json::array();
    for (std::size_t i = 0; i < combat.robot_count; ++i)
      result["robots"].push_back(Robot(combat.robots[i], false));
    result["events"] = Json::array();
    for (std::size_t i = 0; i < combat.event_count; ++i) {
      const auto& event = combat.events[i];
      result["events"].push_back({{"id", event.id},
                                  {"round_id", event.round_id},
                                  {"round_time_ns", event.round_time_ns},
                                  {"detail", reinterpret_cast<const char*>(event.detail)}});
    }
    result["events_dropped"] = combat.events_dropped;
  }
  return result.dump();
}

std::string CombatSchema(bool referee_only) {
  Json properties;
  properties["timestamp"] =
      ObjectSchema({{"sec", {{"type", "integer"}}}, {"nsec", {{"type", "integer"}}}});
  for (const auto* field : {"sequence", "round_id", "sim_time_ns", "round_started_ns"})
    properties[field] = {{"type", "integer"}};
  properties["round_time_s"] = {{"type", "number"}};
  if (referee_only) {
    properties["sample_time_ns"] = {{"type", "integer"}};
    properties["sample_sequence"] = {{"type", "integer"}};
    properties["valid"] = {{"type", "boolean"}};
    properties["self"] = RobotSchema(true);
    properties["self"]["type"] = {"object", "null"};
  } else {
    properties["robots"] = {{"type", "array"}, {"items", RobotSchema(false)}};
    properties["events_dropped"] = {{"type", "integer"}};
    properties["events"] = {{"type", "array"},
                            {"items", ObjectSchema({{"id", {{"type", "integer"}}},
                                                    {"round_id", {{"type", "integer"}}},
                                                    {"round_time_ns", {{"type", "integer"}}},
                                                    {"detail", {{"type", "string"}}}})}};
  }
  return ObjectSchema(std::move(properties)).dump();
}
}  // namespace mv::tool::foxglove::simulation
