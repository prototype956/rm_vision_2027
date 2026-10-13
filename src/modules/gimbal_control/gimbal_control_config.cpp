#include "modules/gimbal_control/gimbal_control_config.hpp"

#include "core/config.hpp"

#include <cmath>
#include <string>

namespace mv::modules {
namespace {
GimbalControlMode ParseMode(const YAML::Node& node, const char* key) {
  const auto VALUE = ConfigLoader::Require<std::string>(node, key, "gimbal control modes");
  if (VALUE == "pid")
    return GimbalControlMode::PID;
  if (VALUE == "mpc")
    return GimbalControlMode::MPC;
  throw ConfigError("gimbal mode must be pid or mpc");
}
GimbalControlConfig ParseCommon(const YAML::Node& root, std::string_view backend) {
  constexpr char CONTEXT[] = "gimbal control";
  ConfigLoader::RejectUnknownKeys(
      root, {"schema_version", "modes", "reference", "feedback_limits", "rate_limits"}, CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1)
    throw ConfigError("gimbal control schema_version must be 1");
  const auto MODES = root["modes"];
  const auto REFERENCE = root["reference"];
  const auto FEEDBACK = root["feedback_limits"];
  const auto RATES = root["rate_limits"];
  ConfigLoader::RejectUnknownKeys(MODES, {"mindvision", "talos"}, CONTEXT);
  ConfigLoader::RejectUnknownKeys(REFERENCE, {"dt_s", "horizon_steps"}, CONTEXT);
  ConfigLoader::RejectUnknownKeys(FEEDBACK, {"max_yaw_velocity_rad_s", "max_pitch_velocity_rad_s"},
                                  CONTEXT);
  ConfigLoader::RejectUnknownKeys(RATES, {"max_yaw_rate_rad_s", "max_pitch_rate_rad_s"}, CONTEXT);
  GimbalControlConfig config;
  config.mpc.reset();
  config.mindvision_mode = ParseMode(MODES, "mindvision");
  config.talos_mode = ParseMode(MODES, "talos");
  if (config.talos_mode != GimbalControlMode::MPC)
    throw ConfigError("Talos supports only MPC angle commands");
  if (backend == "mindvision")
    config.mode = config.mindvision_mode;
  else if (backend == "talos")
    config.mode = config.talos_mode;
  else
    throw ConfigError("unsupported gimbal backend");
  config.dt_s = ConfigLoader::Require<double>(REFERENCE, "dt_s", CONTEXT);
  config.horizon_steps = ConfigLoader::Require<int>(REFERENCE, "horizon_steps", CONTEXT);
  config.feedback_max_yaw_velocity_rad_s =
      ConfigLoader::Require<double>(FEEDBACK, "max_yaw_velocity_rad_s", CONTEXT);
  config.feedback_max_pitch_velocity_rad_s =
      ConfigLoader::Require<double>(FEEDBACK, "max_pitch_velocity_rad_s", CONTEXT);
  config.rate_limits.max_yaw_rate_rad_s =
      ConfigLoader::Require<double>(RATES, "max_yaw_rate_rad_s", CONTEXT);
  config.rate_limits.max_pitch_rate_rad_s =
      ConfigLoader::Require<double>(RATES, "max_pitch_rate_rad_s", CONTEXT);
  if (!std::isfinite(config.dt_s) || config.dt_s <= 0 || config.dt_s > 0.1 ||
      config.horizon_steps < 3 || config.horizon_steps > 500)
    throw ConfigError("invalid gimbal reference timing");
  for (const double RATE :
       {config.feedback_max_yaw_velocity_rad_s, config.feedback_max_pitch_velocity_rad_s,
        config.rate_limits.max_yaw_rate_rad_s, config.rate_limits.max_pitch_rate_rad_s}) {
    if (!std::isfinite(RATE) || RATE <= 0 || RATE > 4.0)
      throw ConfigError("gimbal rate limits must be finite and in (0, 4] rad/s");
  }
  return config;
}
}  // namespace

std::string_view GimbalControlModeName(GimbalControlMode mode) noexcept {
  return mode == GimbalControlMode::PID ? "pid" : "mpc";
}

GimbalControlConfig ParseGimbalControlConfig(const YAML::Node& root, std::string_view backend,
                                             const YAML::Node& algorithm) {
  auto config = ParseCommon(root, backend);
  if (config.mode == GimbalControlMode::PID) {
    config.pid = ParsePidRateControllerConfig(algorithm);
  } else {
    ConfigLoader::RejectUnknownKeys(algorithm, {"schema_version", "planner", "follower"},
                                    "gimbal MPC");
    if (ConfigLoader::Require<int>(algorithm, "schema_version", "gimbal MPC") != 1)
      throw ConfigError("gimbal MPC schema_version must be 1");
    config.mpc =
        ParseMpcTrajectoryPlannerConfig(algorithm["planner"], {config.dt_s, config.horizon_steps});
    config.follower = ParseMpcRateFollowerConfig(algorithm["follower"]);
  }
  return config;
}

GimbalControlConfig LoadGimbalControlConfig(const std::filesystem::path& root,
                                            std::string_view backend) {
  const auto COMMON = ConfigLoader::LoadFile(root / "modules/gimbal_control.yaml", 1);
  const auto CONFIG = ParseCommon(COMMON, backend);
  const auto NAME = std::string(GimbalControlModeName(CONFIG.mode)) + ".yaml";
  return ParseGimbalControlConfig(
      COMMON, backend, ConfigLoader::LoadFile(root / "modules/gimbal_control" / NAME, 1));
}
}  // namespace mv::modules
