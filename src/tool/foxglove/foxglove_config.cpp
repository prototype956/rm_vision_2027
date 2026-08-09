#include "tool/foxglove/foxglove_config.hpp"

#include "core/config.hpp"

#include <cmath>
#include <limits>

namespace mv::tool::foxglove {

Config ParseConfig(const YAML::Node& root, const std::filesystem::path& config_path) {
  constexpr char CONTEXT[] = "Foxglove config";
  ConfigLoader::RejectUnknownKeys(
      root, {"schema_version", "enabled", "server", "image", "recording"}, CONTEXT);

  const int VERSION = ConfigLoader::Require<int>(root, "schema_version", CONTEXT);
  if (VERSION != 1) {
    throw ConfigError("Foxglove config schema_version must be 1");
  }

  const auto SERVER = root["server"];
  const auto IMAGE = root["image"];
  const auto RECORDING = root["recording"];
  ConfigLoader::RejectUnknownKeys(SERVER, {"host", "port"}, "Foxglove config.server");
  ConfigLoader::RejectUnknownKeys(IMAGE, {"frame_id", "max_fps", "jpeg_quality"},
                                  "Foxglove config.image");
  ConfigLoader::RejectUnknownKeys(RECORDING, {"enabled", "output_dir"},
                                  "Foxglove config.recording");

  Config config;
  config.enabled = ConfigLoader::Require<bool>(root, "enabled", CONTEXT);
  config.server.host = ConfigLoader::Require<std::string>(SERVER, "host", "Foxglove config.server");
  const int PORT = ConfigLoader::Require<int>(SERVER, "port", "Foxglove config.server");
  config.image.frame_id =
      ConfigLoader::Require<std::string>(IMAGE, "frame_id", "Foxglove config.image");
  config.image.max_fps = ConfigLoader::Require<double>(IMAGE, "max_fps", "Foxglove config.image");
  config.image.jpeg_quality =
      ConfigLoader::Require<int>(IMAGE, "jpeg_quality", "Foxglove config.image");
  config.recording.enabled =
      ConfigLoader::Require<bool>(RECORDING, "enabled", "Foxglove config.recording");
  const auto OUTPUT_DIR =
      ConfigLoader::Require<std::string>(RECORDING, "output_dir", "Foxglove config.recording");

  if (config.server.host.empty()) {
    throw ConfigError("Foxglove config.server.host must not be empty");
  }
  if (PORT <= 0 || PORT > std::numeric_limits<std::uint16_t>::max()) {
    throw ConfigError("Foxglove config.server.port must be in [1, 65535]");
  }
  if (config.image.frame_id != "camera_optical") {
    throw ConfigError("Foxglove config.image.frame_id must be camera_optical");
  }
  if (!std::isfinite(config.image.max_fps) || config.image.max_fps <= 0.0 ||
      config.image.max_fps > 240.0) {
    throw ConfigError("Foxglove config.image.max_fps must be in (0, 240]");
  }
  if (config.image.jpeg_quality < 1 || config.image.jpeg_quality > 100) {
    throw ConfigError("Foxglove config.image.jpeg_quality must be in [1, 100]");
  }
  if (OUTPUT_DIR.empty()) {
    throw ConfigError("Foxglove config.recording.output_dir must not be empty");
  }

  config.server.port = static_cast<std::uint16_t>(PORT);
  config.recording.output_dir = ConfigLoader::ResolvePath(config_path.parent_path(), OUTPUT_DIR);
  return config;
}

}  // namespace mv::tool::foxglove
