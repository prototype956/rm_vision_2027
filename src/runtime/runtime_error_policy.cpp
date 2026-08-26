#include "runtime/runtime_error_policy.hpp"

#include "core/config.hpp"

namespace mv::runtime {

RuntimeErrorPolicy ParseRuntimeErrorPolicy(const YAML::Node& root) {
  constexpr char CONTEXT[] = "runtime error policy";
  ConfigLoader::RejectUnknownKeys(
      root, {"schema_version", "camera", "control", "optional_components"}, CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1)
    throw ConfigError("runtime error policy schema_version must be 1");

  const auto CAMERA = root["camera"];
  const auto CONTROL = root["control"];
  const auto OPTIONAL = root["optional_components"];
  ConfigLoader::RejectUnknownKeys(CAMERA, {"no_valid_frame_timeout_ms"},
                                  "runtime error policy.camera");
  ConfigLoader::RejectUnknownKeys(CONTROL, {"command_sink_unhealthy_timeout_ms"},
                                  "runtime error policy.control");
  ConfigLoader::RejectUnknownKeys(OPTIONAL,
                                  {"evaluation_disable_after_consecutive_errors",
                                   "debug_window_disable_after_consecutive_errors"},
                                  "runtime error policy.optional_components");

  const int CAMERA_TIMEOUT_MS =
      ConfigLoader::Require<int>(CAMERA, "no_valid_frame_timeout_ms", CONTEXT);
  const int COMMAND_TIMEOUT_MS =
      ConfigLoader::Require<int>(CONTROL, "command_sink_unhealthy_timeout_ms", CONTEXT);
  RuntimeErrorPolicy policy;
  policy.camera_no_valid_frame_timeout = std::chrono::milliseconds(CAMERA_TIMEOUT_MS);
  policy.command_sink_unhealthy_timeout = std::chrono::milliseconds(COMMAND_TIMEOUT_MS);
  policy.evaluation_disable_after_consecutive_errors =
      ConfigLoader::Require<int>(OPTIONAL, "evaluation_disable_after_consecutive_errors", CONTEXT);
  policy.debug_window_disable_after_consecutive_errors = ConfigLoader::Require<int>(
      OPTIONAL, "debug_window_disable_after_consecutive_errors", CONTEXT);
  if (CAMERA_TIMEOUT_MS <= 0 || COMMAND_TIMEOUT_MS <= 0 ||
      policy.evaluation_disable_after_consecutive_errors <= 0 ||
      policy.debug_window_disable_after_consecutive_errors <= 0) {
    throw ConfigError("runtime error policy thresholds must be positive");
  }
  return policy;
}

}  // namespace mv::runtime
