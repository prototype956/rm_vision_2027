#include "modules/gimbal_trajectory_planner/gimbal_trajectory_planner_config.hpp"

#include "core/config.hpp"

#include <cmath>
#include <string>

namespace mv::modules {
namespace {

/** @brief 读取角度、角速度顺序的非负二维状态代价权重。 */
std::array<double, 2> ReadPair(const YAML::Node& node, const char* key,
                               const std::string& context) {
  const auto VALUE = node[key];
  if (!VALUE || !VALUE.IsSequence() || VALUE.size() != 2) {
    throw ConfigError(context + "." + key + " must contain exactly two numbers");
  }
  std::array<double, 2> result{VALUE[0].as<double>(), VALUE[1].as<double>()};
  if (!std::isfinite(result[0]) || !std::isfinite(result[1]) || result[0] < 0.0 ||
      result[1] < 0.0) {
    throw ConfigError(context + "." + key + " must contain finite nonnegative numbers");
  }
  return result;
}

}  // namespace

GimbalTrajectoryPlannerConfig ParseGimbalTrajectoryPlannerConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "gimbal trajectory planner config";
  ConfigLoader::RejectUnknownKeys(
      root, {"schema_version", "timing", "normalization", "limits", "cost", "solver"}, CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1) {
    throw ConfigError("gimbal trajectory planner config schema_version must be 1");
  }
  const auto TIMING = root["timing"];
  const auto NORMALIZATION = root["normalization"];
  const auto LIMITS = root["limits"];
  const auto COST = root["cost"];
  const auto SOLVER = root["solver"];
  ConfigLoader::RejectUnknownKeys(TIMING, {"dt_s", "horizon_steps", "command_lookahead_s"},
                                  "gimbal trajectory planner config.timing");
  ConfigLoader::RejectUnknownKeys(NORMALIZATION, {"angle_scale_rad"},
                                  "gimbal trajectory planner config.normalization");
  ConfigLoader::RejectUnknownKeys(LIMITS,
                                  {"max_yaw_velocity_rad_s", "max_pitch_velocity_rad_s",
                                   "max_yaw_acceleration_rad_s2", "max_pitch_acceleration_rad_s2"},
                                  "gimbal trajectory planner config.limits");
  ConfigLoader::RejectUnknownKeys(COST, {"q_yaw", "q_pitch", "r_yaw", "r_pitch"},
                                  "gimbal trajectory planner config.cost");
  ConfigLoader::RejectUnknownKeys(
      SOLVER, {"max_iterations", "rho", "absolute_primal_tolerance", "absolute_dual_tolerance"},
      "gimbal trajectory planner config.solver");

  GimbalTrajectoryPlannerConfig config;
  config.dt_s = ConfigLoader::Require<double>(TIMING, "dt_s", CONTEXT);
  config.horizon_steps = ConfigLoader::Require<int>(TIMING, "horizon_steps", CONTEXT);
  config.command_lookahead_s =
      ConfigLoader::Require<double>(TIMING, "command_lookahead_s", CONTEXT);
  config.normalization_angle_scale_rad =
      ConfigLoader::Require<double>(NORMALIZATION, "angle_scale_rad", CONTEXT);
  config.max_yaw_velocity_rad_s =
      ConfigLoader::Require<double>(LIMITS, "max_yaw_velocity_rad_s", CONTEXT);
  config.max_pitch_velocity_rad_s =
      ConfigLoader::Require<double>(LIMITS, "max_pitch_velocity_rad_s", CONTEXT);
  config.max_yaw_acceleration_rad_s2 =
      ConfigLoader::Require<double>(LIMITS, "max_yaw_acceleration_rad_s2", CONTEXT);
  config.max_pitch_acceleration_rad_s2 =
      ConfigLoader::Require<double>(LIMITS, "max_pitch_acceleration_rad_s2", CONTEXT);
  config.q_yaw = ReadPair(COST, "q_yaw", "gimbal trajectory planner config.cost");
  config.q_pitch = ReadPair(COST, "q_pitch", "gimbal trajectory planner config.cost");
  config.r_yaw = ConfigLoader::Require<double>(COST, "r_yaw", CONTEXT);
  config.r_pitch = ConfigLoader::Require<double>(COST, "r_pitch", CONTEXT);
  config.max_iterations = ConfigLoader::Require<int>(SOLVER, "max_iterations", CONTEXT);
  config.rho = ConfigLoader::Require<double>(SOLVER, "rho", CONTEXT);
  config.absolute_primal_tolerance =
      ConfigLoader::Require<double>(SOLVER, "absolute_primal_tolerance", CONTEXT);
  config.absolute_dual_tolerance =
      ConfigLoader::Require<double>(SOLVER, "absolute_dual_tolerance", CONTEXT);

  if (!std::isfinite(config.dt_s) || config.dt_s <= 0.0 || config.dt_s > 0.1 ||
      config.horizon_steps < 3 || config.horizon_steps > 500 ||
      !std::isfinite(config.command_lookahead_s) || config.command_lookahead_s < config.dt_s ||
      config.command_lookahead_s > config.dt_s * static_cast<double>(config.horizon_steps - 1) ||
      !std::isfinite(config.normalization_angle_scale_rad) ||
      config.normalization_angle_scale_rad <= 0.0 ||
      !std::isfinite(config.max_yaw_velocity_rad_s) || config.max_yaw_velocity_rad_s <= 0.0 ||
      !std::isfinite(config.max_pitch_velocity_rad_s) || config.max_pitch_velocity_rad_s <= 0.0 ||
      !std::isfinite(config.max_yaw_acceleration_rad_s2) ||
      config.max_yaw_acceleration_rad_s2 <= 0.0 ||
      !std::isfinite(config.max_pitch_acceleration_rad_s2) ||
      config.max_pitch_acceleration_rad_s2 <= 0.0 || !std::isfinite(config.r_yaw) ||
      config.r_yaw <= 0.0 || !std::isfinite(config.r_pitch) || config.r_pitch <= 0.0 ||
      config.max_iterations <= 0 || config.max_iterations > 1000 || !std::isfinite(config.rho) ||
      config.rho <= 0.0 || !std::isfinite(config.absolute_primal_tolerance) ||
      config.absolute_primal_tolerance <= 0.0 || config.absolute_primal_tolerance > 1.0 ||
      !std::isfinite(config.absolute_dual_tolerance) || config.absolute_dual_tolerance <= 0.0 ||
      config.absolute_dual_tolerance > 1.0) {
    throw ConfigError("gimbal trajectory planner config contains an invalid scalar range");
  }
  return config;
}

}  // namespace mv::modules
