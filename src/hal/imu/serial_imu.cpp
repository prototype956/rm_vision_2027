#include "hal/imu/serial_imu.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/serial/attitude_payload.hpp"
#include "hal/serial/controller_link.hpp"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <utility>

namespace mv::hal {
static_assert(sizeof(float) == 4, "wire protocol requires float32");
namespace {
using Clock = std::chrono::steady_clock;
double NowUs() {
  return std::chrono::duration<double, std::micro>(Clock::now().time_since_epoch()).count();
}
std::uint64_t SystemNowNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}
}  // namespace

SerialImuConfig ParseSerialImuConfig(const YAML::Node& root) {
  constexpr char CTX[] = "serial imu";
  ConfigLoader::RejectUnknownKeys(root,
                                  {"schema_version", "enabled", "device", "baud", "startup_sync_ms",
                                   "sync_ms", "timeout_ms", "cache_ms", "max_gap_ms", "wait_ms"},
                                  CTX);
  if (ConfigLoader::Require<int>(root, "schema_version", CTX) != 1)
    throw ConfigError("serial imu schema_version must be 1");
  SerialImuConfig c;
  c.enabled = ConfigLoader::Require<bool>(root, "enabled", CTX);
  c.device = ConfigLoader::Require<std::string>(root, "device", CTX);
  c.baud = ConfigLoader::Require<int>(root, "baud", CTX);
  c.startup_sync_ms = ConfigLoader::Require<int>(root, "startup_sync_ms", CTX);
  c.sync_ms = ConfigLoader::Require<int>(root, "sync_ms", CTX);
  c.timeout_ms = ConfigLoader::Require<int>(root, "timeout_ms", CTX);
  c.cache_ms = ConfigLoader::Require<int>(root, "cache_ms", CTX);
  c.max_gap_ms = ConfigLoader::Require<int>(root, "max_gap_ms", CTX);
  c.wait_ms = ConfigLoader::Require<int>(root, "wait_ms", CTX);
  if (c.baud != 460800 || c.startup_sync_ms < 20 || c.sync_ms < c.startup_sync_ms ||
      c.sync_ms > 1000 || c.timeout_ms < 3 * c.sync_ms || c.cache_ms < 100 || c.cache_ms > 10000 ||
      c.max_gap_ms < 1 || c.max_gap_ms > 100 || c.wait_ms < 0 || c.wait_ms > 100)
    throw ConfigError("serial imu timing/baud out of range");
  return c;
}

struct SerialImu::Impl {
  struct Sample {
    double mcu_us;
    geometry::Quaternion q;
  };
  SerialImuConfig config;
  mutable std::mutex mutex;
  std::condition_variable changed;
  ImuHealth health;
  std::deque<Sample> samples;
  std::uint32_t last_packet{0}, last_sample{0};
  bool have_sample{false};
  double last_mcu_us{0};
  double last_receive_us{0}, last_attitude_receive_us{0}, last_progress_us{0};
  std::optional<double> unhealthy_since_us;
  std::optional<ImuObservation> latest;
  // 最后构造，最先销毁；停止串口回调后才能释放样本与互斥锁。
  std::unique_ptr<serial::ControllerLink> link;

  explicit Impl(SerialImuConfig c) : config(std::move(c)) {
    link = std::make_unique<serial::ControllerLink>(
        serial::ControllerLinkConfig{.enabled = config.enabled,
                                     .device = config.device,
                                     .baud = config.baud,
                                     .startup_sync_ms = config.startup_sync_ms,
                                     .sync_ms = config.sync_ms,
                                     .timeout_ms = config.timeout_ms},
        [this](const serial::ControllerLinkState& state,
               const std::optional<serial::ControllerFrame>& frame) {
          return OnUpdate(state, frame);
        });
    if (!config.enabled || config.device.empty())
      MV_LOG_WARN("IMU", "{}; spatial processing unavailable", health.reason);
  }

  /** @brief 串口线程串行调用；持锁更新姿态缓存和供视觉线程读取的健康快照。 */
  bool OnUpdate(const serial::ControllerLinkState& state,
                const std::optional<serial::ControllerFrame>& frame) {
    std::lock_guard lock(mutex);
    const bool RESET = health.generation != state.generation;
    if (RESET) {
      samples.clear();
      have_sample = false;
      latest.reset();
      health.sample_age_ms = -1;
      last_progress_us = 0;
      unhealthy_since_us.reset();
      health.packet_sequence = 0;
      health.sample_sequence = 0;
    }
    if (RESET || !state.synchronized || !health.synchronized)
      health.reason = state.reason;
    health.connected = state.connected;
    health.synchronized = state.synchronized;
    health.generation = state.generation;
    health.crc_errors = state.crc_errors;
    health.rate_hz = state.rate_hz;
    health.offset_us = state.offset_us;
    health.rtt_us = state.rtt_us;
    last_receive_us = state.last_receive_us;
    last_attitude_receive_us = state.last_attitude_receive_us;
    changed.notify_all();
    if (!frame) {
      // Do not reset hand-eye sessions for mechanism/calibration rejections.
      // In the control path, recover even if packets arrive but samples freeze
      // or the clock mapping puts them in the future. Freshness limits stay strict.
      const double NOW = NowUs();
      const double AGE = samples.empty() ? -1.0 : NOW - samples.back().mcu_us - state.offset_us;
      if (!state.synchronized || config.require_calibration_sample ||
          (!samples.empty() && AGE >= 0.0 && AGE <= 100000.0)) {
        unhealthy_since_us.reset();
      } else if (!unhealthy_since_us) {
        unhealthy_since_us = NOW;
      } else if (NOW - *unhealthy_since_us >= 500000.0) {
        MV_LOG_WARN("IMU",
                    "request reconnect: reason={} age_ms={:.1f} rx_age_ms={:.1f} "
                    "sample_progress_age_ms={:.1f} packet={} sample={} duplicates={}",
                    health.reason, AGE / 1000.0,
                    last_receive_us > 0 ? (NOW - last_receive_us) / 1000.0 : -1.0,
                    last_progress_us > 0 ? (NOW - last_progress_us) / 1000.0 : -1.0,
                    health.packet_sequence, health.sample_sequence, health.duplicate_samples);
        return false;
      }
      return true;
    }
    const auto PAYLOAD = serial::DecodeAttitudePayload(*frame);
    if (!PAYLOAD)
      return true;
    const bool IS_CALIBRATION_SAMPLE = PAYLOAD->calibration;
    if (config.require_calibration_sample && !IS_CALIBRATION_SAMPLE) {
      samples.clear();
      health.reason = "calibration_sample_required";
      return true;
    }
    if (IS_CALIBRATION_SAMPLE) {
      // Preserve runtime rejection priority: version, calibration/mechanism, structure.
      const char* failure = nullptr;
      if (PAYLOAD->version != AV_CAL_PAYLOAD_VERSION)
        failure = "unsupported_calibration_payload";
      else if (const auto* mechanism = serial::CalibrationMechanismFailure(*PAYLOAD))
        failure = mechanism;
      else
        failure = serial::CalibrationStructureFailure(*frame);
      if (failure) {
        samples.clear();
        health.reason = failure;
        return true;
      }
    }
    const auto SEQ = frame->sequence;
    const double MCU = static_cast<double>(PAYLOAD->mcu_us);
    const auto SAMPLE = PAYLOAD->sample_sequence;
    health.packet_sequence = SEQ;
    health.sample_sequence = SAMPLE;
    if (have_sample && (MCU < last_mcu_us || SAMPLE < last_sample || SEQ < last_packet)) {
      samples.clear();
      health.synchronized = false;
      health.reason = "mcu_restart_or_counter_reset";
      return false;
    }
    if (have_sample && SEQ > last_packet + 1)
      health.dropped_packets += SEQ - last_packet - 1;
    last_packet = SEQ;
    const std::uint16_t REQUIRED_STATUS = IS_CALIBRATION_SAMPLE ? 0x03U : 0x01U;
    if ((PAYLOAD->status & REQUIRED_STATUS) != REQUIRED_STATUS) {
      samples.clear();
      health.reason = IS_CALIBRATION_SAMPLE ? "calibration_imu_not_ready" : "imu_not_ready";
      return true;
    }
    geometry::Quaternion q(PAYLOAD->values[0], PAYLOAD->values[1], PAYLOAD->values[2],
                           PAYLOAD->values[3]);
    geometry::Vector3 gyro(PAYLOAD->values[4], PAYLOAD->values[5], PAYLOAD->values[6]);
    if (!q.coeffs().allFinite() || q.norm() < 0.5 || q.norm() > 1.5 || !gyro.allFinite()) {
      samples.clear();
      health.reason = "invalid_attitude";
      return true;
    }
    health.gyro_rad_s = gyro;
    if (have_sample && (MCU == last_mcu_us || SAMPLE == last_sample)) {
      ++health.duplicate_samples;
      return true;
    }
    have_sample = true;
    last_sample = SAMPLE;
    last_mcu_us = MCU;
    last_progress_us = frame->receive_time_us;
    const auto NORMALIZED_Q = q.normalized();
    samples.push_back({MCU, NORMALIZED_Q});
    const double HOST_SAMPLE_US = MCU + state.offset_us;
    const double AGE_US = std::max(0.0, NowUs() - HOST_SAMPLE_US);
    const auto SAMPLE_STEADY = Clock::time_point(std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double, std::micro>(HOST_SAMPLE_US)));
    const auto AGE_NS = static_cast<std::uint64_t>(AGE_US * 1000.0);
    const auto NOW_SYSTEM_NS = SystemNowNs();
    latest =
        ImuObservation{.world_q_imu = NORMALIZED_Q,
                       .gyro_rad_s = gyro,
                       .steady_time = SAMPLE_STEADY,
                       .system_timestamp_ns = AGE_NS < NOW_SYSTEM_NS ? NOW_SYSTEM_NS - AGE_NS : 1U,
                       .sample_sequence = SAMPLE};
    while (!samples.empty() &&
           (MCU - samples.front().mcu_us > config.cache_ms * 1000.0 || samples.size() > 4096))
      samples.pop_front();
    changed.notify_all();
    return true;
  }
};

SerialImu::SerialImu(SerialImuConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}
SerialImu::~SerialImu() = default;

std::optional<geometry::Quaternion> SerialImu::At(Clock::time_point time) {
  std::unique_lock lock(impl_->mutex);
  auto& h = impl_->health;
  if (!h.synchronized)
    return std::nullopt;
  const double HOST = std::chrono::duration<double, std::micro>(time.time_since_epoch()).count();
  const auto GENERATION = h.generation;
  const auto DEADLINE = Clock::now() + std::chrono::milliseconds(impl_->config.wait_ms);
  impl_->changed.wait_until(lock, DEADLINE, [&] {
    return !h.synchronized || h.generation != GENERATION ||
           (!impl_->samples.empty() && impl_->samples.back().mcu_us + h.offset_us >= HOST);
  });
  if (!h.synchronized || h.generation != GENERATION)
    return std::nullopt;
  const double TARGET = HOST - h.offset_us;
  auto& samples = impl_->samples;
  // OnUpdate records the precise rejection reason (protocol version, calibration,
  // mechanism flags, or attitude status). Do not hide it behind a generic cache error.
  if (samples.empty())
    return std::nullopt;
  if (NowUs() - samples.back().mcu_us - h.offset_us > 100000.0) {
    h.reason = "attitude_stale";
    return std::nullopt;
  }
  if (TARGET < samples.front().mcu_us || TARGET > samples.back().mcu_us) {
    h.reason = "image_outside_imu_history";
    return std::nullopt;
  }
  const auto NEXT = std::lower_bound(samples.begin(), samples.end(), TARGET,
                                     [](const Impl::Sample& s, double t) { return s.mcu_us < t; });
  if (NEXT->mcu_us == TARGET) {
    h.reason = "ok";
    return NEXT->q;
  }
  const auto PREV = std::prev(NEXT);
  const double GAP = NEXT->mcu_us - PREV->mcu_us;
  if (GAP > impl_->config.max_gap_ms * 1000.0) {
    h.reason = "imu_sample_gap";
    return std::nullopt;
  }
  h.reason = "ok";
  return PREV->q.slerp((TARGET - PREV->mcu_us) / GAP, NEXT->q).normalized();
}
ImuHealth SerialImu::Health() const {
  std::lock_guard lock(impl_->mutex);
  auto result = impl_->health;
  const double NOW = NowUs();
  result.receive_age_ms = impl_->last_receive_us > 0
      ? (NOW - impl_->last_receive_us) / 1000.0 : -1.0;
  result.attitude_receive_age_ms = impl_->last_attitude_receive_us > 0
      ? (NOW - impl_->last_attitude_receive_us) / 1000.0 : -1.0;
  result.sample_progress_age_ms = impl_->last_progress_us > 0
      ? (NOW - impl_->last_progress_us) / 1000.0 : -1.0;
  if (result.synchronized && !impl_->samples.empty())
    result.sample_age_ms = (NOW - impl_->samples.back().mcu_us - result.offset_us) / 1000.0;
  return result;
}

std::optional<ImuObservation> SerialImu::Latest() const {
  std::lock_guard lock(impl_->mutex);
  if (!impl_->health.synchronized || !impl_->latest || impl_->samples.empty())
    return std::nullopt;
  return impl_->latest;
}

bool SerialImu::QueueGimbalCommand(float yaw_rate_rad_s, float pitch_rate_rad_s, bool enabled,
                                   bool fire, bool friction) noexcept {
  {
    std::lock_guard lock(impl_->mutex);
    if (!impl_->health.connected || !impl_->health.synchronized)
      return false;
  }
  return impl_->link &&
         impl_->link->QueueGimbalCommand(yaw_rate_rad_s, pitch_rate_rad_s, enabled, fire, friction);
}
}  // namespace mv::hal
