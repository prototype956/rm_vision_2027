#pragma once

#include <cstdint>
#include <string>

#include <yaml-cpp/yaml.h>

namespace mv::tool::web {

struct ServerConfig {
  std::string host{"0.0.0.0"};
  std::uint16_t port{8080};
};

struct AuthConfig {
  std::string password{"rmvision2027"};
};

struct PreviewConfig {
  double max_fps{8.0};
  int jpeg_quality{75};
};

/** @brief Web 调试服务的监听、认证和预览参数。 */
struct Config {
  bool enabled{true};
  ServerConfig server;
  AuthConfig auth;
  PreviewConfig preview;
};

/** @brief 解析并严格校验 Web 调试配置。 */
[[nodiscard]] Config ParseConfig(const YAML::Node& root);

}  // namespace mv::tool::web
