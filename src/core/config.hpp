#pragma once

#include <stdexcept>
#include <string>
#include <unordered_set>

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace mv {

class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class ConfigLoader {
 public:

  static YAML::Node LoadFile(const std::filesystem::path& input_path) {
    const auto path = NormalizeExistingFile(input_path);

    YAML::Node root;
    try {
      root = YAML::LoadFile(path.string());
    } catch (const std::exception& error) {
      throw ConfigError("cannot load config '" + path.string() + "': " + error.what());
    }

    RequireMap(root, "config '" + path.string() + "'");
    const int version = Require<int>(root, "schema_version", "config '" + path.string() + "'");
    if (version != 1) {
      throw ConfigError("unsupported schema_version in '" + path.string() +
                        "': " + std::to_string(version));
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
      const auto key = item.first.as<std::string>();
      if (!allowed.contains(key)) {
        throw ConfigError(context + " contains unknown key '" + key + "'");
      }
    }
  }

  static std::filesystem::path ResolvePath(const std::filesystem::path& base,
                                           const std::filesystem::path& path) {
    const auto combined = path.is_absolute() ? path : base / path;
    return std::filesystem::absolute(combined).lexically_normal();
  }

 private:
  static std::filesystem::path NormalizeExistingFile(const std::filesystem::path& input_path) {
    const auto path = std::filesystem::absolute(input_path).lexically_normal();
    if (!std::filesystem::is_regular_file(path)) {
      throw ConfigError("config file does not exist: " + path.string());
    }
    return path;
  }
};

}  // namespace mv
