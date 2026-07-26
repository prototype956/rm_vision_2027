#include "mindvision_config.hpp"

#include "core/config.hpp"

#include <string>

namespace mv::hal::detail {

MindVisionConfig ParseMindVisionConfig(const YAML::Node& root) {
  constexpr char CONTEXT[] = "MindVision camera config";
  ConfigLoader::RejectUnknownKeys(
      root, {"schema_version", "device", "output", "roi", "exposure", "capture"}, CONTEXT);
  if (ConfigLoader::Require<int>(root, "schema_version", CONTEXT) != 1) {
    throw ConfigError("MindVision camera config schema_version must be 1");
  }

  MindVisionConfig config;

  const auto DEVICE = root["device"];
  ConfigLoader::RequireMap(DEVICE, "MindVision camera config.device");
  ConfigLoader::RejectUnknownKeys(DEVICE, {"index"}, "MindVision camera config.device");
  config.device_index =
      ConfigLoader::Require<int>(DEVICE, "index", "MindVision camera config.device");

  const auto OUTPUT = root["output"];
  ConfigLoader::RequireMap(OUTPUT, "MindVision camera config.output");
  ConfigLoader::RejectUnknownKeys(OUTPUT, {"width", "height", "pixel_format"},
                                  "MindVision camera config.output");
  config.width = ConfigLoader::Require<int>(OUTPUT, "width", "MindVision camera config.output");
  config.height = ConfigLoader::Require<int>(OUTPUT, "height", "MindVision camera config.output");
  const auto PIXEL_FORMAT =
      ConfigLoader::Require<std::string>(OUTPUT, "pixel_format", "MindVision camera config.output");
  if (PIXEL_FORMAT != "bgr8") {
    throw ConfigError("MindVision camera currently requires output.pixel_format=bgr8");
  }

  const auto ROI = root["roi"];
  ConfigLoader::RequireMap(ROI, "MindVision camera config.roi");
  ConfigLoader::RejectUnknownKeys(ROI, {"centered"}, "MindVision camera config.roi");
  config.centered_roi =
      ConfigLoader::Require<bool>(ROI, "centered", "MindVision camera config.roi");
  if (!config.centered_roi) {
    throw ConfigError("MindVision camera currently requires roi.centered=true");
  }

  const auto EXPOSURE = root["exposure"];
  ConfigLoader::RequireMap(EXPOSURE, "MindVision camera config.exposure");
  ConfigLoader::RejectUnknownKeys(EXPOSURE, {"auto", "time_us"},
                                  "MindVision camera config.exposure");
  config.auto_exposure =
      ConfigLoader::Require<bool>(EXPOSURE, "auto", "MindVision camera config.exposure");
  config.exposure_us =
      ConfigLoader::Require<int>(EXPOSURE, "time_us", "MindVision camera config.exposure");

  const auto CAPTURE = root["capture"];
  ConfigLoader::RequireMap(CAPTURE, "MindVision camera config.capture");
  ConfigLoader::RejectUnknownKeys(CAPTURE, {"timeout_ms"}, "MindVision camera config.capture");
  config.grab_timeout_ms =
      ConfigLoader::Require<int>(CAPTURE, "timeout_ms", "MindVision camera config.capture");

  if (config.device_index < 0 || config.width <= 0 || config.height <= 0 ||
      config.exposure_us <= 0 || config.grab_timeout_ms <= 0) {
    throw ConfigError("MindVision device index and numeric camera parameters must be positive");
  }
  return config;
}

}  // namespace mv::hal::detail
