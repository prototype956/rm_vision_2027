#include "runtime/real_frame_geometry.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <fmt/format.h>

namespace mv::runtime {
namespace {
std::vector<double> Values(const YAML::Node& root, const char* key, std::size_t size) {
  auto values = ConfigLoader::Require<std::vector<double>>(root, key, "real geometry");
  if (values.size() != size) throw ConfigError(std::string(key) + ": wrong array length");
  for (double value : values) {
    if (!std::isfinite(value)) throw ConfigError("non-finite geometry value");
  }
  return values;
}
}  // namespace

RealFrameGeometry::RealFrameGeometry(const std::filesystem::path& root,
                                     const hal::CameraInfo& camera)
    : config_(LoadRealGeometryConfig(root)), imu_(config_.imu) {
  const auto K = ConfigLoader::LoadFile(config_.intrinsics_path, 1);
  ConfigLoader::RejectUnknownKeys(K, {"schema_version", "width", "height", "roi_offset_x",
      "roi_offset_y", "camera_matrix", "distortion"}, "intrinsics");
  camera_model_.width = K["width"].as<std::uint32_t>();
  camera_model_.height = K["height"].as<std::uint32_t>();
  const auto MATRIX = Values(K, "camera_matrix", 9);
  const auto DISTORTION = Values(K, "distortion", 5);
  camera_model_.fx = MATRIX[0];
  camera_model_.fy = MATRIX[4];
  camera_model_.cx = MATRIX[2];
  camera_model_.cy = MATRIX[5];
  if (!camera_model_.width || !camera_model_.height || MATRIX[0] <= 0 || MATRIX[4] <= 0 ||
      MATRIX[1] != 0 || MATRIX[3] != 0 || MATRIX[6] != 0 || MATRIX[7] != 0 || MATRIX[8] != 1)
    throw ConfigError("invalid pinhole camera matrix");
  std::copy(DISTORTION.begin(), DISTORTION.end(), camera_model_.distortion.begin());
  camera_matches_ = camera.output_width == static_cast<int>(camera_model_.width) &&
      camera.output_height == static_cast<int>(camera_model_.height) &&
      camera.roi_offset_x == K["roi_offset_x"].as<int>() &&
      camera.roi_offset_y == K["roi_offset_y"].as<int>();
  MV_LOG_INFO("Geometry", "deployment={} intrinsics={} extrinsics={} R_GI_wxyz=[{},{},{},{}]",
              config_.profile, config_.intrinsics_path.string(), config_.extrinsics_path.string(),
              config_.extrinsics.gimbal_q_imu.w(), config_.extrinsics.gimbal_q_imu.x(),
              config_.extrinsics.gimbal_q_imu.y(), config_.extrinsics.gimbal_q_imu.z());
  MV_LOG_WARN("Geometry",
              "real rotation-only reference; placeholder_extrinsics={} camera_matches={} "
              "camera time is software-estimated; no real actuator output",
              config_.placeholder, camera_matches_);
}

void RealFrameGeometry::Attach(frame::FramePacket& packet) {
  packet.kinematics.reset();
  packet.camera_model.reset();
  const auto TIME = packet.capture.stamp.capture_steady_time;
  std::string reason;
  auto before = imu_.Health();
  if (!camera_matches_ || packet.capture.image.cols != static_cast<int>(camera_model_.width) ||
      packet.capture.image.rows != static_cast<int>(camera_model_.height)) {
    reason = "intrinsics_roi_mismatch";
  } else if (!TIME) {
    reason = "camera_clock_not_ready";
  } else if (generation_ != before.generation || (last_capture_ && *TIME <= *last_capture_)) {
    reason = "clock_generation_changed";
  } else {
    auto q = imu_.At(*TIME);
    auto after = imu_.Health();
    if (q && before.generation == after.generation) {
      packet.camera_model = camera_model_;
      packet.kinematics = frame::FrameKinematics{
          .world_t_gimbal = geometry::ResolveRotationOnlyWorldTGimbal(*q, config_.extrinsics),
          .gimbal_t_camera_optical = config_.extrinsics.gimbal_t_camera_optical,
          .gimbal_t_muzzle = config_.extrinsics.gimbal_t_muzzle};
      reason = "ok";
    } else {
      reason = after.reason;
    }
  }
  const auto HEALTH = imu_.Health();
  generation_ = HEALTH.generation;
  last_capture_ = TIME;
  status_ = fmt::format("IMU:{} profile={} sync={} {:.0f}Hz age={:.1f}ms ext={} cam=estimated",
      reason, config_.profile, HEALTH.synchronized, HEALTH.rate_hz, HEALTH.sample_age_ms,
      config_.placeholder ? "PLACEHOLDER" : "configured");
  const auto NOW = std::chrono::steady_clock::now();
  if (NOW - last_log_ >= std::chrono::seconds(1)) {
    last_log_ = NOW;
    const auto GYRO_GIMBAL =
        geometry::ResolveGimbalAngularVelocity(HEALTH.gyro_rad_s, config_.extrinsics);
    MV_LOG_INFO("Geometry",
        "{} crc={} dropped={} offset_us={:.1f} rtt_us={:.1f} "
        "gyro_imu_rad_s=[{:.4f},{:.4f},{:.4f}] gyro_gimbal_rad_s=[{:.4f},{:.4f},{:.4f}]",
        status_, HEALTH.crc_errors, HEALTH.dropped_packets, HEALTH.offset_us, HEALTH.rtt_us,
        HEALTH.gyro_rad_s.x(), HEALTH.gyro_rad_s.y(), HEALTH.gyro_rad_s.z(),
        GYRO_GIMBAL.x(), GYRO_GIMBAL.y(), GYRO_GIMBAL.z());
  }
}
}  // namespace mv::runtime
