#include "tool/web/web_debug_config.hpp"

#include "core/config.hpp"

#include <cmath>
#include <limits>

namespace mv::tool::web {

Config ParseConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "Web debug config";
  ConfigLoader::RejectUnknownKeys(root, {"schema_version", "enabled", "server", "auth", "preview"},
                                  CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1) {
    throw ConfigError("Web debug config schema_version must be 1");
  }

  const auto SERVER = root["server"];
  const auto AUTH = root["auth"];
  const auto PREVIEW = root["preview"];
  ConfigLoader::RejectUnknownKeys(SERVER, {"host", "port"}, "Web debug config.server");
  ConfigLoader::RejectUnknownKeys(AUTH, {"password"}, "Web debug config.auth");
  ConfigLoader::RejectUnknownKeys(PREVIEW, {"max_fps", "jpeg_quality"}, "Web debug config.preview");

  Config config;
  config.enabled = ConfigLoader::Require<bool>(root, "enabled", CONTEXT);
  config.server.host =
      ConfigLoader::Require<std::string>(SERVER, "host", "Web debug config.server");
  const int PORT = ConfigLoader::Require<int>(SERVER, "port", "Web debug config.server");
  config.auth.password =
      ConfigLoader::Require<std::string>(AUTH, "password", "Web debug config.auth");
  config.preview.max_fps =
      ConfigLoader::Require<double>(PREVIEW, "max_fps", "Web debug config.preview");
  config.preview.jpeg_quality =
      ConfigLoader::Require<int>(PREVIEW, "jpeg_quality", "Web debug config.preview");

  if (config.server.host.empty()) {
    throw ConfigError("Web debug config.server.host must not be empty");
  }
  if (PORT <= 0 || PORT > std::numeric_limits<std::uint16_t>::max()) {
    throw ConfigError("Web debug config.server.port must be in [1, 65535]");
  }
  if (config.auth.password.empty()) {
    throw ConfigError("Web debug config.auth.password must not be empty");
  }
  if (!std::isfinite(config.preview.max_fps) || config.preview.max_fps <= 0.0 ||
      config.preview.max_fps > 30.0) {
    throw ConfigError("Web debug config.preview.max_fps must be in (0, 30]");
  }
  if (config.preview.jpeg_quality < 1 || config.preview.jpeg_quality > 100) {
    throw ConfigError("Web debug config.preview.jpeg_quality must be in [1, 100]");
  }
  config.server.port = static_cast<std::uint16_t>(PORT);
  return config;
}

}  // namespace mv::tool::web
