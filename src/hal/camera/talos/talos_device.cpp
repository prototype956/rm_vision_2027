#include "talos_device.hpp"

#include "core/logger.hpp"
#include "talos_ipc_layout.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <thread>

#include <fcntl.h>
#include <opencv2/imgproc.hpp>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mv::hal::detail {
namespace {
using namespace talos_ipc;

uint64_t SystemNowNs() noexcept {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count());
}

uint8_t AtomicLoad(const uint8_t* value) noexcept {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

uint64_t AtomicLoad(const uint64_t* value) noexcept {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

[[nodiscard]] bool IsMapFailed(const void* mapping) noexcept {
  // MAP_FAILED 是 POSIX 规定的整数到指针哨兵值，并非业务代码中的地址转换。
  return mapping == MAP_FAILED;  // NOLINT(performance-no-int-to-ptr)
}

template <typename Slot, typename TripleBuffer>
bool Consume(TripleBuffer& buffer, Slot& slot) noexcept {
  // 发布端用最高位表示存在新数据、低两位表示可读槽位。CAS 抢占状态后再复制
  // 元数据，保证同一个槽位不会被本消费者重复读取。
  uint8_t expected = AtomicLoad(&buffer.state);
  if ((expected & K_FLAG_NEW) == 0) {
    return false;
  }
  const uint8_t READY_INDEX = expected & K_INDEX_MASK;
  if (READY_INDEX >= K_BUFFER_COUNT) {
    return false;
  }
  const uint8_t CONSUMED_STATE = buffer.read_index;
  if (!__atomic_compare_exchange_n(&buffer.state, &expected, CONSUMED_STATE, false,
                                   __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
    return false;
  }
  buffer.read_index = READY_INDEX;
  slot = buffer.slots[READY_INDEX];
  return true;
}

bool Finite(float value) noexcept {
  return std::isfinite(static_cast<double>(value));
}

// 除有限性外检查单位四元数范数，避免无效姿态进入后续 TF 组合和投影。
bool ValidTransform(const RigidTransformF32& transform) noexcept {
  for (const float VALUE : transform.translation) {
    if (!Finite(VALUE)) {
      return false;
    }
  }
  const auto& q = transform.rotation;
  if (!Finite(q.x) || !Finite(q.y) || !Finite(q.z) || !Finite(q.w)) {
    return false;
  }
  const double NORM = std::sqrt(static_cast<double>(q.x) * q.x + static_cast<double>(q.y) * q.y +
                                static_cast<double>(q.z) * q.z + static_cast<double>(q.w) * q.w);
  return std::abs(NORM - 1.0) <= 1.0e-3;
}

bool ValidCalibration(const CameraCalibrationMeta& calibration, uint32_t width,
                      uint32_t height) noexcept {
  if (calibration.width != width || calibration.height != height ||
      !std::isfinite(calibration.fx) || !std::isfinite(calibration.fy) ||
      !std::isfinite(calibration.cx) || !std::isfinite(calibration.cy) || calibration.fx <= 0.0 ||
      calibration.fy <= 0.0 || calibration.cx < 0.0 || calibration.cy < 0.0 ||
      calibration.cx > width || calibration.cy > height) {
    return false;
  }
  for (const double VALUE : calibration.distortion) {
    if (!std::isfinite(VALUE)) {
      return false;
    }
  }
  return true;
}

bool ValidChassisObservation(const ChassisObservationMeta& observation,
                             const CapturedFrameMeta& frame) noexcept {
  if (observation.frame_sequence != frame.frame_sequence ||
      observation.timestamp_ns != frame.capture_timestamp_ns || !Finite(observation.dt_s) ||
      observation.dt_s <= 0.0F || observation.dt_s > 1.0F ||
      !Finite(observation.yaw_velocity_rad_s) ||
      std::abs(observation.yaw_velocity_rad_s) > 100.0F ||
      !Finite(observation.yaw_acceleration_rad_s2)) {
    return false;
  }
  for (const float VALUE : observation.velocity_body_mps) {
    if (!Finite(VALUE) || std::abs(VALUE) > 20.0F)
      return false;
  }
  for (const float VALUE : observation.wheel_linear_mps) {
    if (!Finite(VALUE))
      return false;
  }
  for (const float VALUE : observation.wheel_angular_rad_s) {
    if (!Finite(VALUE))
      return false;
  }
  for (const float VALUE : observation.acceleration_body_mps2) {
    if (!Finite(VALUE))
      return false;
  }
  for (const float VALUE : observation.rpy_rad) {
    if (!Finite(VALUE))
      return false;
  }
  for (const float VALUE : observation.gyro_xyz_rad_s) {
    if (!Finite(VALUE))
      return false;
  }
  for (const float VALUE : observation.accel_xyz_mps2) {
    if (!Finite(VALUE))
      return false;
  }
  return true;
}

mv::geometry::RigidTransform ConvertTransform(const RigidTransformF32& value) noexcept {
  return {.translation = mv::geometry::Vector3(value.translation[0], value.translation[1],
                                               value.translation[2]),
          // Eigen 四元数构造顺序为 w/x/y/z；Talos v6 协议字段为 x/y/z/w。
          .rotation = mv::geometry::Quaternion(value.rotation.w, value.rotation.x, value.rotation.y,
                                               value.rotation.z)};
}

/** @brief 拒绝越界、非法编码及混合回合采样，避免把损坏附件交给展示端。 */
bool ValidCombat(const simulation::CombatFrameMeta& combat) {
  using simulation::COMBAT_MAX_EVENTS;
  using simulation::COMBAT_MAX_ROBOTS;
  if (combat.round_id == 0 || combat.robot_count > COMBAT_MAX_ROBOTS ||
      combat.event_count > COMBAT_MAX_EVENTS || combat.referee_valid > 1 ||
      combat.round_started_ns > combat.sim_time_ns ||
      combat.referee_sample_ns > combat.sim_time_ns ||
      combat.referee_sample_ns < combat.round_started_ns)
    return false;
  const auto VALID_ROBOT = [](const simulation::RobotCombatMeta& robot) {
    return robot.robot_id != 0 && robot.hp <= robot.max_hp && robot.max_hp > 0 && robot.team <= 1 &&
           robot.role <= 2 && robot.life <= 1 && robot.shooter <= 1 && robot.allowance_mode <= 1 &&
           robot.fire_permitted <= 1 && robot.fire_blocks <= 255 &&
           (robot.fire_permitted != 0) == (robot.fire_blocks == 0) &&
           (robot.life == 1) == (robot.hp == 0) && std::isfinite(robot.heat) && robot.heat >= 0 &&
           std::isfinite(robot.heat_limit) && robot.heat_limit >= 0 &&
           std::isfinite(robot.cooling_per_second) && robot.cooling_per_second >= 0 &&
           std::isfinite(robot.heat_locked_s) && robot.heat_locked_s >= 0;
  };
  if (combat.referee_valid &&
      (!VALID_ROBOT(combat.self_referee) || combat.referee_sample_sequence == 0))
    return false;
  for (std::size_t i = 0; i < combat.robot_count; ++i) {
    if (!VALID_ROBOT(combat.robots[i]) ||
        (i > 0 && combat.robots[i - 1].robot_id >= combat.robots[i].robot_id))
      return false;
  }
  for (std::size_t i = 0; i < combat.event_count; ++i) {
    const auto& event = combat.events[i];
    if (event.id == 0 || event.round_id != combat.round_id ||
        event.round_time_ns > combat.sim_time_ns - combat.round_started_ns ||
        event.detail[sizeof(event.detail) - 1] != 0 ||
        (i > 0 && combat.events[i - 1].id >= event.id))
      return false;
  }
  return true;
}

void PopulateFrameContext(const CapturedFrameMeta& metadata, bool chassis_valid, bool truth_valid,
                          bool projectile_valid, frame::FramePacket& packet) {
  // Grab() 已分别验证正式数据和可选仿真附件，这里只负责从 ABI 类型提升。
  const auto& camera = metadata.camera_info;
  packet.camera_model = frame::CameraModel{
      .width = camera.width,
      .height = camera.height,
      .fx = camera.fx,
      .fy = camera.fy,
      .cx = camera.cx,
      .cy = camera.cy,
      .distortion = {camera.distortion[0], camera.distortion[1], camera.distortion[2],
                     camera.distortion[3], camera.distortion[4]}};
  packet.kinematics = frame::FrameKinematics{
      .world_t_gimbal = ConvertTransform(metadata.world_t_gimbal),
      .gimbal_t_camera_optical = ConvertTransform(metadata.gimbal_t_camera_optical),
      .gimbal_t_muzzle = ConvertTransform(metadata.gimbal_t_muzzle)};

  if (chassis_valid) {
    const auto& chassis = metadata.chassis_observation;
    packet.chassis_motion = frame::ChassisMotionObservation{
        .yaw_rad = chassis.rpy_rad[2],
        .velocity_body_mps =
            Eigen::Vector2d(chassis.velocity_body_mps[0], chassis.velocity_body_mps[1]),
        .yaw_velocity_rad_s = chassis.yaw_velocity_rad_s,
        .source_sequence = chassis.frame_sequence,
        .source_timestamp_ns = chassis.timestamp_ns};
  }

  if (metadata.gimbal_telemetry_valid != 0) {
    const auto FORWARD =
        packet.kinematics->world_t_gimbal.rotation * mv::geometry::Vector3::UnitX();
    packet.gimbal_actuator = GimbalActuatorTelemetry{
        .valid = true,
        .state_timestamp_ns = metadata.capture_timestamp_ns,
        .consumed_command_timestamp_ns = metadata.gimbal_consumed_command_timestamp_ns,
        .mode = static_cast<GimbalActuatorMode>(metadata.gimbal_actuator_mode),
        .command_valid = metadata.gimbal_command_valid != 0,
        .saturation_flags = metadata.gimbal_saturation_flags,
        .actual_yaw = std::atan2(FORWARD.y(), FORWARD.x()),
        .actual_pitch = std::atan2(FORWARD.z(), std::hypot(FORWARD.x(), FORWARD.y())),
        .yaw_velocity = metadata.gimbal_yaw_velocity_rad_s,
        .pitch_velocity = metadata.gimbal_pitch_velocity_rad_s,
        .yaw_acceleration = metadata.gimbal_yaw_acceleration_rad_s2,
        .pitch_acceleration = metadata.gimbal_pitch_acceleration_rad_s2};
  }

  simulation::SimulationFrameData simulation;
  simulation.combat = metadata.combat;
  if (!truth_valid) {
    packet.simulation = std::move(simulation);
    return;
  }
  if (projectile_valid) {
    const auto& projectiles = metadata.projectile_statistics;
    simulation.projectile_statistics =
        simulation::ProjectileStatistics{.bullet_launch_count = projectiles.bullet_launch_count,
                                         .armor_hit_count = projectiles.armor_hit_count,
                                         .rune_hit_count = projectiles.rune_hit_count,
                                         .dart_launch_count = projectiles.dart_launch_count};
  }
  const auto& truth = metadata.ground_truth;
  simulation.targets.reserve(truth.target_count);
  for (std::size_t index = 0; index < truth.target_count; ++index) {
    const auto& target = truth.targets[index];
    simulation.targets.push_back({.id = target.id,
                                  .team = target.team,
                                  .armor_label = target.armor_label,
                                  .is_outpost = target.is_outpost != 0,
                                  .position_world = mv::geometry::Vector3(
                                      target.position[0], target.position[1], target.position[2]),
                                  .yaw = target.yaw,
                                  .yaw_velocity = target.yaw_velocity});
  }
  simulation.armors.reserve(truth.armor_count);
  for (std::size_t index = 0; index < truth.armor_count; ++index) {
    const auto& armor = truth.armors[index];
    simulation::GroundTruthArmor converted{
        .owner_robot_id = armor.owner_robot_id,
        .id = armor.id,
        .team = armor.team,
        .label = armor.label,
        .type = static_cast<geometry::ArmorType>(armor.armor_type),
        .width_m = armor.width_m,
        .height_m = armor.height_m,
        .world_t_armor = ConvertTransform(armor.world_t_armor)};
    for (std::size_t corner = 0; corner < converted.corners_world.size(); ++corner) {
      converted.corners_world[corner] =
          mv::geometry::Vector3(armor.corners_world[corner][0], armor.corners_world[corner][1],
                                armor.corners_world[corner][2]);
    }
    simulation.armors.push_back(std::move(converted));
  }
  packet.simulation = std::move(simulation);
}

}  // namespace

struct TalosDevice::Impl {
  TalosConfig config;                           ///< 最近一次 Open() 接收的类型化配置。
  int meta_fd{-1};                              ///< 可读写的元数据文件描述符。
  int image_pool_fd{-1};                        ///< 只读图像池文件描述符。
  void* meta_mapping{nullptr};                  ///< 元数据 mmap 起始地址。
  void* image_pool_mapping{nullptr};            ///< 图像池 mmap 起始地址。
  std::size_t image_pool_size{0};               ///< 已映射图像池的字节数。
  ShmMetaRegion* meta{nullptr};                 ///< 带类型的协议元数据视图。
  CameraInfo info;                              ///< Open() 成功后对上层公开的信息。
  bool is_open{false};                          ///< 两个映射和发布端心跳均有效。
  std::optional<uint64_t> last_frame_sequence;  ///< 最近接收帧序号，用于拒绝重复或回退帧。
  std::optional<uint64_t> last_capture_timestamp_ns;  ///< 最近采集时间，用于检查严格单调性。
  uint64_t invalid_frames{0};  ///< 本次连接以来被完整性校验拒绝的帧数。
  uint64_t invalid_simulation_frames{0};    ///< 未影响正式抓帧的非法仿真附件数。
  std::uint64_t invalid_chassis_frames{0};  ///< 已降级忽略的非法底盘观测数。
  std::chrono::steady_clock::time_point steady_anchor{};  ///< system/steady 映射锚点。
  std::chrono::system_clock::time_point system_anchor{};  ///< 与 steady_anchor 同次采样。

  /** 按映射、文件描述符的逆依赖顺序释放资源，并恢复关闭状态。 */
  void ResetMappings() noexcept {
    if (image_pool_mapping != nullptr) {
      ::munmap(image_pool_mapping, image_pool_size);
    }
    if (meta_mapping != nullptr) {
      ::munmap(meta_mapping, sizeof(ShmMetaRegion));
    }
    if (image_pool_fd >= 0) {
      ::close(image_pool_fd);
    }
    if (meta_fd >= 0) {
      ::close(meta_fd);
    }
    meta_fd = -1;
    image_pool_fd = -1;
    meta_mapping = nullptr;
    image_pool_mapping = nullptr;
    image_pool_size = 0;
    meta = nullptr;
    is_open = false;
    last_frame_sequence.reset();
    last_capture_timestamp_ns.reset();
    invalid_frames = 0;
    invalid_simulation_frames = 0;
    invalid_chassis_frames = 0;
    steady_anchor = {};
    system_anchor = {};
  }

  /** 将同主机 Unix 采集时间映射到 steady_clock；未来时刻不会被钳制伪装。 */
  [[nodiscard]] std::optional<std::chrono::steady_clock::time_point> CaptureSteadyTime(
      std::uint64_t capture_timestamp_ns,
      std::chrono::steady_clock::time_point receive_time) const noexcept {
    if (capture_timestamp_ns == 0 ||
        capture_timestamp_ns >
            static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
        steady_anchor == std::chrono::steady_clock::time_point{} ||
        system_anchor == std::chrono::system_clock::time_point{}) {
      return std::nullopt;
    }
    const auto CAPTURE_SYSTEM = std::chrono::system_clock::time_point(
        std::chrono::nanoseconds(static_cast<std::int64_t>(capture_timestamp_ns)));
    const auto MAPPED =
        steady_anchor + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            CAPTURE_SYSTEM - system_anchor);
    if (MAPPED > receive_time)
      return std::nullopt;
    return MAPPED;
  }

  bool HeartbeatFresh() const noexcept {
    if (meta == nullptr) {
      return false;
    }
    const uint64_t HEARTBEAT_NS = AtomicLoad(&meta->header.heartbeat_ns);
    const uint64_t NOW_NS = SystemNowNs();
    const uint64_t MAX_AGE_NS = static_cast<uint64_t>(config.heartbeat_timeout_ms) * 1'000'000ULL;
    // 发布端时间略领先时视为有效，以容忍系统时钟的小幅回拨或跨进程采样误差。
    return HEARTBEAT_NS != 0 && (HEARTBEAT_NS >= NOW_NS || NOW_NS - HEARTBEAT_NS <= MAX_AGE_NS);
  }

  /** 尝试建立一次完整映射；任一步失败都会回滚本次获得的全部资源。 */
  bool TryConnect(std::string& error) {
    ResetMappings();
    meta_fd = ::open(config.meta_path.c_str(), O_RDWR | O_CLOEXEC);
    if (meta_fd < 0) {
      error = "cannot open metadata '" + config.meta_path + "'";
      return false;
    }

    ShmHeader header_probe{};
    if (::pread(meta_fd, &header_probe, sizeof(header_probe), 0) != sizeof(header_probe) ||
        header_probe.magic != K_SHM_MAGIC || header_probe.version != K_SHM_VERSION) {
      error = "Talos protocol mismatch: expected v7, got v" + std::to_string(header_probe.version);
      ResetMappings();
      return false;
    }
    struct stat meta_stat {};
    if (::fstat(meta_fd, &meta_stat) != 0 ||
        meta_stat.st_size < static_cast<off_t>(sizeof(ShmMetaRegion))) {
      error = "metadata file has an invalid size";
      ResetMappings();
      return false;
    }
    meta_mapping =
        ::mmap(nullptr, sizeof(ShmMetaRegion), PROT_READ | PROT_WRITE, MAP_SHARED, meta_fd, 0);
    if (IsMapFailed(meta_mapping)) {
      meta_mapping = nullptr;
      error = "cannot map Talos metadata";
      ResetMappings();
      return false;
    }
    meta = static_cast<ShmMetaRegion*>(meta_mapping);

    const uint32_t VERSION = meta->header.version;
    const uint32_t IMAGE_WIDTH = meta->header.image_width;
    const uint32_t IMAGE_HEIGHT = meta->header.image_height;
    if (meta->header.magic != K_SHM_MAGIC || VERSION != K_SHM_VERSION) {
      error = "unsupported Talos metadata magic/version";
      ResetMappings();
      return false;
    }
    if (IMAGE_WIDTH != static_cast<uint32_t>(config.expected_width) ||
        IMAGE_HEIGHT != static_cast<uint32_t>(config.expected_height)) {
      error = "Talos output resolution does not match expected " +
              std::to_string(config.expected_width) + "x" + std::to_string(config.expected_height);
      ResetMappings();
      return false;
    }

    // 在计算三张 RGB/BGR 图像所需空间前先排除 size_t 乘法溢出。
    const auto WIDTH = static_cast<std::size_t>(IMAGE_WIDTH);
    const auto HEIGHT = static_cast<std::size_t>(IMAGE_HEIGHT);
    if (WIDTH > SIZE_MAX / HEIGHT || WIDTH * HEIGHT > SIZE_MAX / 3 / K_BUFFER_COUNT) {
      error = "Talos image dimensions overflow the image-pool size";
      ResetMappings();
      return false;
    }
    image_pool_size = WIDTH * HEIGHT * 3 * K_BUFFER_COUNT;
    image_pool_fd = ::open(config.image_pool_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (image_pool_fd < 0) {
      error = "cannot open image pool '" + config.image_pool_path + "'";
      ResetMappings();
      return false;
    }
    struct stat pool_stat {};
    if (::fstat(image_pool_fd, &pool_stat) != 0 ||
        pool_stat.st_size < static_cast<off_t>(image_pool_size)) {
      error = "Talos image-pool file has an invalid size";
      ResetMappings();
      return false;
    }
    image_pool_mapping = ::mmap(nullptr, image_pool_size, PROT_READ, MAP_SHARED, image_pool_fd, 0);
    if (IsMapFailed(image_pool_mapping)) {
      image_pool_mapping = nullptr;
      error = "cannot map Talos image pool";
      ResetMappings();
      return false;
    }
    if (!HeartbeatFresh()) {
      error = "Talos heartbeat is not active";
      ResetMappings();
      return false;
    }

    const auto STEADY_BEFORE = std::chrono::steady_clock::now();
    system_anchor = std::chrono::system_clock::now();
    const auto STEADY_AFTER = std::chrono::steady_clock::now();
    steady_anchor = STEADY_BEFORE + (STEADY_AFTER - STEADY_BEFORE) / 2;

    info.device_name = "talos:" + config.meta_path;
    info.sensor_width = config.expected_width;
    info.sensor_height = config.expected_height;
    info.output_width = config.expected_width;
    info.output_height = config.expected_height;
    info.roi_offset_x = 0;
    info.roi_offset_y = 0;
    info.exposure_us = 0;
    info.grab_timeout_ms = config.grab_timeout_ms;
    info.pixel_format = PixelFormat::BGR8;
    is_open = true;
    return true;
  }
};

TalosDevice::TalosDevice() : impl_(std::make_unique<Impl>()) {}

TalosDevice::~TalosDevice() {
  Close();
}

bool TalosDevice::Open(const TalosConfig& config) {
  if (impl_->is_open) {
    return true;
  }
  impl_->config = config;

  // 发布端可能仍在创建或扩容共享内存文件，因此在配置的窗口内轮询连接。
  const auto DEADLINE = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(impl_->config.connect_timeout_ms);
  std::string last_error = "Talos publisher is unavailable";
  do {
    if (impl_->TryConnect(last_error)) {
      MV_LOG_INFO("HAL.Camera.Talos", "connected to Talos v{} at {}x{} BGR8",
                  impl_->meta->header.version, impl_->info.output_width, impl_->info.output_height);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < DEADLINE);

  MV_LOG_ERROR("HAL.Camera.Talos", "connection timed out after {} ms: {}",
               impl_->config.connect_timeout_ms, last_error);
  impl_->ResetMappings();
  return false;
}

void TalosDevice::Close() noexcept {
  const bool WAS_OPEN = impl_->is_open;
  impl_->ResetMappings();
  if (WAS_OPEN) {
    MV_LOG_INFO("HAL.Camera.Talos", "closed");
  }
}

GrabStatus TalosDevice::Grab(frame::FramePacket& packet) {
  if (!impl_->is_open || impl_->meta == nullptr) {
    return GrabStatus::DISCONNECTED;
  }

  const auto DEADLINE =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(impl_->config.grab_timeout_ms);
  while (std::chrono::steady_clock::now() < DEADLINE) {
    if (!impl_->HeartbeatFresh()) {
      MV_LOG_ERROR("HAL.Camera.Talos", "publisher heartbeat expired");
      Close();
      return GrabStatus::DISCONNECTED;
    }

    CapturedFrameMeta metadata{};
    if (!Consume<CapturedFrameMeta>(impl_->meta->frame, metadata)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    const auto INVALID = [&]() {
      const uint64_t COUNT = ++impl_->invalid_frames;
      if (COUNT == 1 || COUNT % 100 == 0) {
        MV_LOG_WARN("HAL.Camera.Talos", "rejected invalid Talos v7 frame #{} (seq={})", COUNT,
                    metadata.frame_sequence);
      }
      return GrabStatus::INVALID_FRAME;
    };

    // 底盘观测和仿真附件仅在与正式帧严格同步时交给上层；附件失效不拒绝图像。
    const bool CHASSIS_VALID = ValidChassisObservation(metadata.chassis_observation, metadata);
    const auto& truth = metadata.ground_truth;
    const auto& projectiles = metadata.projectile_statistics;
    const bool PROJECTILE_VALID = projectiles.timestamp_ns == metadata.capture_timestamp_ns;
    bool truth_valid = truth.frame_sequence == metadata.frame_sequence &&
                       truth.timestamp_ns == metadata.capture_timestamp_ns &&
                       truth.target_count <= K_GROUND_TRUTH_MAX_TARGETS &&
                       truth.armor_count <= K_GROUND_TRUTH_MAX_ARMORS;
    for (std::size_t index = 0; truth_valid && index < truth.target_count; ++index) {
      const auto& target = truth.targets[index];
      truth_valid = target.frame_sequence == metadata.frame_sequence &&
                    target.timestamp_ns == metadata.capture_timestamp_ns && Finite(target.yaw) &&
                    Finite(target.yaw_velocity) && Finite(target.position[0]) &&
                    Finite(target.position[1]) && Finite(target.position[2]);
    }
    for (std::size_t index = 0; truth_valid && index < truth.armor_count; ++index) {
      const auto& armor = truth.armors[index];
      truth_valid = armor.team <= 1 && armor.label <= 7 && armor.armor_type <= 1 &&
                    Finite(armor.width_m) && Finite(armor.height_m) && armor.width_m > 0.0F &&
                    armor.height_m > 0.0F && ValidTransform(armor.world_t_armor);
      for (std::size_t corner = 0; truth_valid && corner < 4; ++corner) {
        truth_valid = Finite(armor.corners_world[corner][0]) &&
                      Finite(armor.corners_world[corner][1]) &&
                      Finite(armor.corners_world[corner][2]);
      }
      if (truth_valid) {
        const auto POSE = ConvertTransform(armor.world_t_armor);
        const std::array<mv::geometry::Vector3, 4> EXPECTED{
            mv::geometry::Vector3(-armor.width_m * 0.5, armor.height_m * 0.5, 0.0),
            mv::geometry::Vector3(armor.width_m * 0.5, armor.height_m * 0.5, 0.0),
            mv::geometry::Vector3(armor.width_m * 0.5, -armor.height_m * 0.5, 0.0),
            mv::geometry::Vector3(-armor.width_m * 0.5, -armor.height_m * 0.5, 0.0)};
        for (std::size_t corner = 0; truth_valid && corner < EXPECTED.size(); ++corner) {
          const mv::geometry::Vector3 ACTUAL(armor.corners_world[corner][0],
                                             armor.corners_world[corner][1],
                                             armor.corners_world[corner][2]);
          truth_valid =
              (ACTUAL - mv::geometry::TransformPoint(POSE, EXPECTED[corner])).norm() <= 0.002;
        }
      }
    }

    if (!ValidCombat(metadata.combat) || metadata.capture_timestamp_ns == 0 ||
        metadata.camera_info.timestamp_ns != metadata.capture_timestamp_ns ||
        metadata.width != static_cast<uint32_t>(impl_->info.output_width) ||
        metadata.height != static_cast<uint32_t>(impl_->info.output_height) ||
        metadata.buffer_id >= K_BUFFER_COUNT ||
        (metadata.format != K_FORMAT_RGB8 && metadata.format != K_FORMAT_BGR8) ||
        !ValidCalibration(metadata.camera_info, metadata.width, metadata.height) ||
        !ValidTransform(metadata.world_t_gimbal) ||
        !ValidTransform(metadata.gimbal_t_camera_optical) ||
        !ValidTransform(metadata.gimbal_t_muzzle) ||
        (metadata.gimbal_telemetry_valid != 0 &&
         (metadata.gimbal_telemetry_valid != 1 || metadata.gimbal_actuator_mode > 2 ||
          metadata.gimbal_command_valid > 1 || !Finite(metadata.gimbal_yaw_velocity_rad_s) ||
          !Finite(metadata.gimbal_pitch_velocity_rad_s) ||
          !Finite(metadata.gimbal_yaw_acceleration_rad_s2) ||
          !Finite(metadata.gimbal_pitch_acceleration_rad_s2))) ||
        (impl_->last_frame_sequence.has_value() &&
         metadata.frame_sequence <= *impl_->last_frame_sequence) ||
        (impl_->last_capture_timestamp_ns.has_value() &&
         metadata.capture_timestamp_ns <= *impl_->last_capture_timestamp_ns)) {
      return INVALID();
    }

    if (!truth_valid || !PROJECTILE_VALID) {
      const std::uint64_t COUNT = ++impl_->invalid_simulation_frames;
      if (COUNT == 1 || COUNT % 100 == 0) {
        MV_LOG_WARN("HAL.Camera.Talos",
                    "ignored invalid simulation attachment #{} (seq={} truth={} projectiles={})",
                    COUNT, metadata.frame_sequence, truth_valid, PROJECTILE_VALID);
      }
    }
    if (!CHASSIS_VALID) {
      const std::uint64_t COUNT = ++impl_->invalid_chassis_frames;
      if (COUNT == 1 || COUNT % 100 == 0) {
        MV_LOG_WARN("HAL.Camera.Talos", "ignored invalid chassis observation #{} (seq={})", COUNT,
                    metadata.frame_sequence);
      }
    }

    const std::size_t FRAME_SIZE =
        static_cast<std::size_t>(metadata.width) * static_cast<std::size_t>(metadata.height) * 3;
    const std::size_t OFFSET = static_cast<std::size_t>(metadata.buffer_id) * FRAME_SIZE;
    if (OFFSET > impl_->image_pool_size || FRAME_SIZE > impl_->image_pool_size - OFFSET) {
      return INVALID();
    }

    packet = {};
    const auto* source = static_cast<const uint8_t*>(impl_->image_pool_mapping) + OFFSET;
    const cv::Mat SHARED_IMAGE(static_cast<int>(metadata.height), static_cast<int>(metadata.width),
                               CV_8UC3, const_cast<uint8_t*>(source));
    // 发布端会复用共享三缓冲槽位，返回前必须复制或转换到独立拥有的 cv::Mat。
    if (metadata.format == K_FORMAT_BGR8) {
      packet.capture.image = SHARED_IMAGE.clone();
    } else {
      cv::cvtColor(SHARED_IMAGE, packet.capture.image, cv::COLOR_RGB2BGR);
    }
    if (packet.capture.image.empty()) {
      return GrabStatus::FATAL;
    }
    impl_->last_frame_sequence = metadata.frame_sequence;
    impl_->last_capture_timestamp_ns = metadata.capture_timestamp_ns;
    packet.capture.stamp.receive_steady_time = std::chrono::steady_clock::now();
    packet.capture.stamp.capture_steady_time = impl_->CaptureSteadyTime(
        metadata.capture_timestamp_ns, packet.capture.stamp.receive_steady_time);
    packet.capture.stamp.capture_timestamp_ns = metadata.capture_timestamp_ns;
    packet.capture.stamp.sequence = metadata.frame_sequence;
    packet.capture.stamp.simulation_round_id = metadata.combat.round_id;
    packet.capture.stamp.source_invalid_frames = impl_->invalid_frames;
    PopulateFrameContext(metadata, CHASSIS_VALID, truth_valid, PROJECTILE_VALID, packet);
    return GrabStatus::OK;
  }
  return GrabStatus::TIMEOUT;
}

CameraInfo TalosDevice::Info() const {
  return impl_->info;
}

bool TalosDevice::IsOpen() const noexcept {
  return impl_->is_open;
}

}  // namespace mv::hal::detail
