#include "talos_config.hpp"

#include "core/config.hpp"

#include <string>

namespace mv::hal::detail {

TalosConfig ParseTalosConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "Talos camera config";
  ConfigLoader::RejectUnknownKeys(root, {"schema_version", "shared_memory", "expected", "timeouts"},
                                  CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1) {
    throw ConfigError("Talos camera config schema_version must be 1");
  }

  const auto SHARED_MEMORY = root["shared_memory"];
  ConfigLoader::RequireMap(SHARED_MEMORY, "Talos camera config.shared_memory");
  ConfigLoader::RejectUnknownKeys(SHARED_MEMORY, {"meta_path", "image_pool_path"},
                                  "Talos camera config.shared_memory");

  const auto EXPECTED = root["expected"];
  ConfigLoader::RequireMap(EXPECTED, "Talos camera config.expected");
  ConfigLoader::RejectUnknownKeys(EXPECTED, {"width", "height", "pixel_format"},
                                  "Talos camera config.expected");

  const auto TIMEOUTS = root["timeouts"];
  ConfigLoader::RequireMap(TIMEOUTS, "Talos camera config.timeouts");
  ConfigLoader::RejectUnknownKeys(TIMEOUTS, {"connect_ms", "grab_ms", "heartbeat_ms"},
                                  "Talos camera config.timeouts");

  TalosConfig config;
  config.meta_path = ConfigLoader::Require<std::string>(SHARED_MEMORY, "meta_path",
                                                        "Talos camera config.shared_memory");
  config.image_pool_path = ConfigLoader::Require<std::string>(SHARED_MEMORY, "image_pool_path",
                                                              "Talos camera config.shared_memory");
  config.expected_width =
      ConfigLoader::Require<int>(EXPECTED, "width", "Talos camera config.expected");
  config.expected_height =
      ConfigLoader::Require<int>(EXPECTED, "height", "Talos camera config.expected");
  const auto PIXEL_FORMAT =
      ConfigLoader::Require<std::string>(EXPECTED, "pixel_format", "Talos camera config.expected");
  config.connect_timeout_ms =
      ConfigLoader::Require<int>(TIMEOUTS, "connect_ms", "Talos camera config.timeouts");
  config.grab_timeout_ms =
      ConfigLoader::Require<int>(TIMEOUTS, "grab_ms", "Talos camera config.timeouts");
  config.heartbeat_timeout_ms =
      ConfigLoader::Require<int>(TIMEOUTS, "heartbeat_ms", "Talos camera config.timeouts");

  if (config.meta_path.empty() || config.image_pool_path.empty()) {
    throw ConfigError("Talos shared-memory paths must not be empty");
  }
  if (config.expected_width <= 0 || config.expected_height <= 0 || PIXEL_FORMAT != "bgr8") {
    throw ConfigError("Talos expected output requires positive dimensions and pixel_format=bgr8");
  }
  if (config.connect_timeout_ms <= 0 || config.grab_timeout_ms <= 0 ||
      config.heartbeat_timeout_ms <= 0) {
    throw ConfigError("Talos timeouts must be positive");
  }
  return config;
}

}  // namespace mv::hal::detail
