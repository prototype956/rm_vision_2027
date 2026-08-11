#pragma once

#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace mv {

class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class ConfigLoader {
 public:
  static YAML::Node LoadFile(const std::filesystem::path& input_path,
                             int supported_schema_version = 1) {
    const auto PATH = NormalizeExistingFile(input_path);

    if (supported_schema_version <= 0) {
      throw ConfigError("supported schema_version must be positive for '" + PATH.string() + "'");
    }

    YAML::Node root;
    try {
      root = YAML::LoadFile(PATH.string());
    } catch (const std::exception& error) {
      throw ConfigError("cannot load config '" + PATH.string() + "': " + error.what());
    }

    RequireMap(root, "config '" + PATH.string() + "'");
    const int VERSION = Require<int>(root, "schema_version", "config '" + PATH.string() + "'");
    if (VERSION != supported_schema_version) {
      throw ConfigError("unsupported schema_version in '" + PATH.string() +
                        "': " + std::to_string(VERSION) + " (expected " +
                        std::to_string(supported_schema_version) + ")");
    }
    return root;
  }

  template <typename T>
  static T Require(const YAML::Node& node, const std::string& key, const std::string& context) {
    if (!node || !node.IsMap() || !node[key]) {
      throw ConfigError(context + " is missing required key '" + key + "'");
    }
    try {
      return node[key].as<T>();
    } catch (const std::exception& error) {
      throw ConfigError(context + "." + key + " has invalid type: " + error.what());
    }
  }

  static void RequireMap(const YAML::Node& node, const std::string& context) {
    if (!node || !node.IsMap()) {
      throw ConfigError(context + " must be a YAML map");
    }
  }

  static void RejectUnknownKeys(const YAML::Node& node,
                                const std::unordered_set<std::string>& allowed,
                                const std::string& context) {
    RequireMap(node, context);
    for (const auto& item : node) {
      const auto KEY = item.first.as<std::string>();
      if (!allowed.contains(KEY)) {
        std::string message = context;
        message += " contains unknown key '";
        message += KEY;
        message += "'";
        throw ConfigError(message);
      }
    }
  }

  static std::filesystem::path ResolvePath(const std::filesystem::path& base,
                                           const std::filesystem::path& path) {
    const auto COMBINED = path.is_absolute() ? path : base / path;
    return std::filesystem::absolute(COMBINED).lexically_normal();
  }

 private:
  static std::filesystem::path NormalizeExistingFile(const std::filesystem::path& input_path) {
    auto path = std::filesystem::absolute(input_path).lexically_normal();
    if (!std::filesystem::is_regular_file(path)) {
      throw ConfigError("config file does not exist: " + path.string());
    }
    return path;
  }
};

}  // namespace mv
