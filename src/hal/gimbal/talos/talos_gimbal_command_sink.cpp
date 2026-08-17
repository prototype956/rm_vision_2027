#include "hal/gimbal/talos/talos_gimbal_command_sink.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/talos/talos_ipc_layout.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include <fcntl.h>
#include <optional>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mv::hal {
namespace {

using namespace detail::talos_ipc;
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

std::uint64_t SystemNowNs() noexcept {
  const auto VALUE = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
  return VALUE > 0 ? static_cast<std::uint64_t>(VALUE) : 0;
}

std::uint64_t AtomicLoad(const std::uint64_t* value) noexcept {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

std::uint8_t AtomicLoad(const std::uint8_t* value) noexcept {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

template <typename T>
T AtomicLoadValue(const T* value) noexcept {
  T result{};
  __atomic_load(value, &result, __ATOMIC_ACQUIRE);
  return result;
}

// 发布端以 timestamp_ns 作为快照提交标记。前后两次时间戳一致时，才接受中间
// 逐字段原子读取的结果，避免把相邻仿真步的状态拼成一份遥测。
std::optional<RuntimeStateMeta> ReadRuntimeState(const RuntimeStateMeta* source) noexcept {
  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto BEFORE = AtomicLoad(&source->timestamp_ns);
    if (BEFORE == 0)
      continue;
    RuntimeStateMeta value{};
    value.timestamp_ns = BEFORE;
    value.consumed_command_timestamp_ns = AtomicLoad(&source->consumed_command_timestamp_ns);
    value.consumed_at_timestamp_ns = AtomicLoad(&source->consumed_at_timestamp_ns);
    value.target_yaw_rad = AtomicLoadValue(&source->target_yaw_rad);
    value.target_pitch_rad = AtomicLoadValue(&source->target_pitch_rad);
    value.actual_yaw_rad = AtomicLoadValue(&source->actual_yaw_rad);
    value.actual_pitch_rad = AtomicLoadValue(&source->actual_pitch_rad);
    value.yaw_velocity_rad_s = AtomicLoadValue(&source->yaw_velocity_rad_s);
    value.pitch_velocity_rad_s = AtomicLoadValue(&source->pitch_velocity_rad_s);
    value.yaw_acceleration_rad_s2 = AtomicLoadValue(&source->yaw_acceleration_rad_s2);
    value.pitch_acceleration_rad_s2 = AtomicLoadValue(&source->pitch_acceleration_rad_s2);
    value.following = AtomicLoad(&source->following);
    value.actuator_mode = AtomicLoad(&source->actuator_mode);
    value.saturation_flags = AtomicLoad(&source->saturation_flags);
    value.command_valid = AtomicLoad(&source->command_valid);
    if (BEFORE == AtomicLoad(&source->timestamp_ns))
      return value;
  }
  return std::nullopt;
}

}  // namespace

struct TalosGimbalCommandSink::Impl {
  int fd{-1};                             ///< Talos 元数据文件描述符。
  void* mapping{nullptr};                 ///< 可读写 mmap 起始地址。
  ShmMetaRegion* meta{nullptr};           ///< 带类型的 Talos v5 协议视图。
  std::uint64_t heartbeat_timeout_ns{0};  ///< 允许的发布端最大心跳间隔。
  mutable std::mutex mutex;  ///< 串行化映射生命周期、命令发布和遥测读取。

  /** @brief 解除映射并关闭描述符，将实例恢复为未连接状态。 */
  void CloseMapping() noexcept {
    if (mapping != nullptr && mapping != MAP_FAILED)
      munmap(mapping, sizeof(ShmMetaRegion));
    if (fd >= 0)
      close(fd);
    fd = -1;
    mapping = nullptr;
    meta = nullptr;
  }
};

TalosGimbalCommandSink::TalosGimbalCommandSink() : impl_(std::make_unique<Impl>()) {}

TalosGimbalCommandSink::~TalosGimbalCommandSink() {
  Close();
}

bool TalosGimbalCommandSink::Open(const YAML::Node& camera_config) noexcept {
  try {
    constexpr char CONTEXT[] = "Talos gimbal command config";
    const auto SHARED_MEMORY = camera_config["shared_memory"];
    const auto TIMEOUTS = camera_config["timeouts"];
    const auto PATH = ConfigLoader::Require<std::string>(SHARED_MEMORY, "meta_path", CONTEXT);
    const int HEARTBEAT_MS = ConfigLoader::Require<int>(TIMEOUTS, "heartbeat_ms", CONTEXT);
    if (PATH.empty() || HEARTBEAT_MS <= 0)
      throw ConfigError("invalid Talos command config");

    std::lock_guard lock(impl_->mutex);
    if (impl_->meta != nullptr)
      return true;
    impl_->fd = open(PATH.c_str(), O_RDWR | O_CLOEXEC);
    if (impl_->fd < 0) {
      MV_LOG_ERROR("HAL.Gimbal.Talos", "cannot open metadata {}", PATH);
      impl_->CloseMapping();
      return false;
    }
    struct stat status {};
    if (fstat(impl_->fd, &status) != 0 ||
        status.st_size != static_cast<off_t>(sizeof(ShmMetaRegion))) {
      MV_LOG_ERROR("HAL.Gimbal.Talos", "metadata size mismatch: {}", PATH);
      impl_->CloseMapping();
      return false;
    }
    impl_->mapping =
        mmap(nullptr, sizeof(ShmMetaRegion), PROT_READ | PROT_WRITE, MAP_SHARED, impl_->fd, 0);
    if (impl_->mapping == MAP_FAILED) {
      MV_LOG_ERROR("HAL.Gimbal.Talos", "cannot map metadata {}", PATH);
      impl_->CloseMapping();
      return false;
    }
    impl_->meta = static_cast<ShmMetaRegion*>(impl_->mapping);
    if (impl_->meta->header.magic != K_SHM_MAGIC || impl_->meta->header.version != K_SHM_VERSION) {
      MV_LOG_ERROR("HAL.Gimbal.Talos", "unsupported Talos metadata magic/version");
      impl_->CloseMapping();
      return false;
    }
    impl_->heartbeat_timeout_ns = static_cast<std::uint64_t>(HEARTBEAT_MS) * 1'000'000ULL;
    MV_LOG_INFO("HAL.Gimbal.Talos", "command sink connected to {}", PATH);
    return true;
  } catch (const std::exception& error) {
    MV_LOG_ERROR("HAL.Gimbal.Talos", "open failed: {}", error.what());
    std::lock_guard lock(impl_->mutex);
    impl_->CloseMapping();
    return false;
  }
}

void TalosGimbalCommandSink::Close() noexcept {
  // 在映射仍有效时发布 invalid 命令，使 Talos 立即退出外部跟随并禁止开火。
  if (impl_->meta != nullptr) {
    GimbalCommand stop;
    stop.timestamp_ns = SystemNowNs();
    Send(stop);
  }
  std::lock_guard lock(impl_->mutex);
  impl_->CloseMapping();
}

bool TalosGimbalCommandSink::Send(const GimbalCommand& command) noexcept {
  std::lock_guard lock(impl_->mutex);
  if (impl_->meta == nullptr)
    return false;
  auto& buffer = impl_->meta->gimbal_cmd;
  if (buffer.write_index >= K_BUFFER_COUNT)
    return false;
  auto& slot = buffer.slots[buffer.write_index];
  const bool VALID = command.valid && std::isfinite(command.yaw) && std::isfinite(command.pitch) &&
                     std::isfinite(command.target_distance_m) && command.target_distance_m >= 0.0;
  slot = {};
  slot.timestamp_ns = command.timestamp_ns != 0 ? command.timestamp_ns : SystemNowNs();
  slot.yaw_deg = VALID ? static_cast<float>(command.yaw * RAD_TO_DEG) : 0.0F;
  // Talos/Bevy 协议的 pitch 正方向与内部 ROS 世界系（+Z 向上）相反。
  slot.pitch_deg = VALID ? static_cast<float>(-command.pitch * RAD_TO_DEG) : 0.0F;
  slot.distance_m = VALID ? static_cast<float>(command.target_distance_m) : -1.0F;
  slot.fire_advice = VALID && command.fire ? 1 : 0;
  // 先完整写入槽位，再用原子交换发布“新数据 + 槽位索引”；交换返回的旧状态
  // 携带消费者已释放的槽位，作为下一次写入位置。
  const std::uint8_t READY = static_cast<std::uint8_t>(buffer.write_index | K_FLAG_NEW);
  const std::uint8_t OLD = __atomic_exchange_n(&buffer.state, READY, __ATOMIC_ACQ_REL);
  buffer.write_index = OLD & K_INDEX_MASK;
  return buffer.write_index < K_BUFFER_COUNT;
}

bool TalosGimbalCommandSink::ExternalControlEnabled() const noexcept {
  std::lock_guard lock(impl_->mutex);
  return impl_->meta != nullptr &&
         ReadRuntimeState(&impl_->meta->runtime_state).value_or(RuntimeStateMeta{}).following != 0;
}

bool TalosGimbalCommandSink::IsHealthy() const noexcept {
  std::lock_guard lock(impl_->mutex);
  if (impl_->meta == nullptr)
    return false;
  const auto HEARTBEAT = AtomicLoad(&impl_->meta->header.heartbeat_ns);
  const auto NOW = SystemNowNs();
  return HEARTBEAT <= NOW && NOW - HEARTBEAT <= impl_->heartbeat_timeout_ns;
}

std::uint64_t TalosGimbalCommandSink::HeartbeatTimestampNs() const noexcept {
  std::lock_guard lock(impl_->mutex);
  return impl_->meta != nullptr ? AtomicLoad(&impl_->meta->header.heartbeat_ns) : 0;
}

GimbalActuatorTelemetry TalosGimbalCommandSink::ActuatorTelemetry() const noexcept {
  std::lock_guard lock(impl_->mutex);
  if (impl_->meta == nullptr)
    return {};
  const auto STATE = ReadRuntimeState(&impl_->meta->runtime_state);
  if (!STATE || STATE->actuator_mode == 0 || STATE->actuator_mode > 2 || STATE->command_valid > 1 ||
      !std::isfinite(STATE->target_yaw_rad) || !std::isfinite(STATE->target_pitch_rad) ||
      !std::isfinite(STATE->actual_yaw_rad) || !std::isfinite(STATE->actual_pitch_rad) ||
      !std::isfinite(STATE->yaw_velocity_rad_s) || !std::isfinite(STATE->pitch_velocity_rad_s) ||
      !std::isfinite(STATE->yaw_acceleration_rad_s2) ||
      !std::isfinite(STATE->pitch_acceleration_rad_s2)) {
    return {};
  }
  return {.valid = true,
          .state_timestamp_ns = STATE->timestamp_ns,
          .consumed_command_timestamp_ns = STATE->consumed_command_timestamp_ns,
          .consumed_at_timestamp_ns = STATE->consumed_at_timestamp_ns,
          .mode = static_cast<GimbalActuatorMode>(STATE->actuator_mode),
          .command_valid = STATE->command_valid != 0,
          .saturation_flags = STATE->saturation_flags,
          .target_yaw = STATE->target_yaw_rad,
          .target_pitch = STATE->target_pitch_rad,
          .actual_yaw = STATE->actual_yaw_rad,
          .actual_pitch = STATE->actual_pitch_rad,
          .yaw_velocity = STATE->yaw_velocity_rad_s,
          .pitch_velocity = STATE->pitch_velocity_rad_s,
          .yaw_acceleration = STATE->yaw_acceleration_rad_s2,
          .pitch_acceleration = STATE->pitch_acceleration_rad_s2};
}

}  // namespace mv::hal
