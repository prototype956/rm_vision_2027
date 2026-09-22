#include "runtime/real_geometry_config.hpp"

#include "core/config.hpp"

#include <cmath>
#include <vector>

namespace mv::runtime {
namespace {
std::vector<double> Values(const YAML::Node& root, const char* key, std::size_t size) {
  auto values = ConfigLoader::Require<std::vector<double>>(root, key, "extrinsics");
  if (values.size() != size) throw ConfigError(std::string(key) + ": wrong array length");
  for (double value : values) {
    if (!std::isfinite(value)) throw ConfigError(std::string(key) + ": non-finite value");
  }
  return values;
}

geometry::Quaternion Rotation(const YAML::Node& root, const char* key) {
  const auto VALUES = Values(root, key, 9);
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = VALUES[static_cast<std::size_t>(row * 3 + col)];
    }
  }
  if (!geometry::IsRotationMatrix(rotation)) {
    throw ConfigError(std::string(key) + ": rotation must be SO(3)");
  }
  return geometry::Quaternion(rotation).normalized();
}

geometry::Vector3 Translation(const YAML::Node& root, const char* key) {
  const auto VALUES = Values(root, key, 3);
  return {VALUES[0], VALUES[1], VALUES[2]};
}

std::filesystem::path ProfilePath(const YAML::Node& profile, const char* key,
                                  const std::filesystem::path& root) {
  const auto VALUE = ConfigLoader::Require<std::string>(profile, key, "real geometry profile");
  if (VALUE.empty()) throw ConfigError(std::string(key) + ": empty profile path");
  return ConfigLoader::ResolvePath(root, VALUE);
}
}  // namespace

RealGeometryConfig LoadRealGeometryConfig(const std::filesystem::path& config_root) {
  const auto ROOT = ConfigLoader::LoadFile(config_root / "runtime/real_geometry.yaml", 1);
  ConfigLoader::RejectUnknownKeys(ROOT, {"schema_version", "active_profile", "profiles"},
                                 "real geometry deployment");
  RealGeometryConfig config;
  config.profile = ConfigLoader::Require<std::string>(ROOT, "active_profile", "real geometry");
  const auto PROFILES = ROOT["profiles"];
  ConfigLoader::RequireMap(PROFILES, "real geometry profiles");
  if (config.profile.empty() || !PROFILES[config.profile]) {
    throw ConfigError("unknown real geometry active_profile: " + config.profile);
  }
  const auto PROFILE = PROFILES[config.profile];
  ConfigLoader::RequireMap(PROFILE, "real geometry selected profile");
  ConfigLoader::RejectUnknownKeys(PROFILE, {"imu_config", "camera_intrinsics", "extrinsics"},
                                 "real geometry selected profile");
  config.imu = hal::ParseSerialImuConfig(
      ConfigLoader::LoadFile(ProfilePath(PROFILE, "imu_config", config_root), 1));
  config.intrinsics_path = ProfilePath(PROFILE, "camera_intrinsics", config_root);
  config.extrinsics_path = ProfilePath(PROFILE, "extrinsics", config_root);
  const auto E = ConfigLoader::LoadFile(config.extrinsics_path, 1);
  ConfigLoader::RejectUnknownKeys(E,
      {"schema_version", "placeholder", "gimbal_r_imu", "gimbal_r_camera", "gimbal_t_camera_m",
       "gimbal_r_muzzle", "gimbal_t_muzzle_m"}, "extrinsics");
  config.placeholder = ConfigLoader::Require<bool>(E, "placeholder", "extrinsics");
  config.extrinsics.gimbal_q_imu = Rotation(E, "gimbal_r_imu");
  config.extrinsics.gimbal_t_camera_optical = {
      .translation = Translation(E, "gimbal_t_camera_m"),
      .rotation = Rotation(E, "gimbal_r_camera")};
  config.extrinsics.gimbal_t_muzzle = {
      .translation = Translation(E, "gimbal_t_muzzle_m"),
      .rotation = Rotation(E, "gimbal_r_muzzle")};
  return config;
}

}  // namespace mv::runtime
