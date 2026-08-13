#include "modules/armor_predictor/armor_predictor.hpp"

#include "modules/armor_predictor/detail/armor_association.hpp"
#include "modules/armor_predictor/detail/armor_ekf.hpp"
#include "modules/armor_predictor/detail/four_armor_model.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <optional>

namespace mv::modules {
namespace {

int LabelPriority(ArmorLabel label, const ArmorPredictorConfig& config) noexcept {
  const auto INDEX = static_cast<std::size_t>(label);
  if (INDEX < config.label_priorities.size())
    return config.label_priorities[INDEX];
  return std::numeric_limits<int>::max();
}

bool SupportedLabel(ArmorLabel label) noexcept {
  return label == ArmorLabel::SENTRY || label == ArmorLabel::ONE || label == ArmorLabel::TWO ||
         label == ArmorLabel::THREE || label == ArmorLabel::FOUR;
}

double ArmorRollForLabel(ArmorLabel label, const ArmorPredictorConfig& config) noexcept {
  switch (label) {
    case ArmorLabel::ONE:
      return config.hero_armor_roll_rad;
    case ArmorLabel::TWO:
    case ArmorLabel::THREE:
    case ArmorLabel::FOUR:
      return config.vehicle_armor_roll_rad;
    case ArmorLabel::SENTRY:
    default:
      return 0.0;
  }
}

}  // namespace

struct ArmorPredictor::Impl {
  explicit Impl(ArmorPredictorConfig value) : config(std::move(value)) {}

  [[nodiscard]] std::vector<detail::Observation> ExtractObservations(
      const hal::CameraFrame& frame, const ArmorPnpFrameResult& pnp_result) const;
  void Reset(std::string reason);
  void Initialize(const detail::Observation& observation);
  [[nodiscard]] ArmorPredictionResult Snapshot(std::uint64_t sequence, double dt) const;
  [[nodiscard]] ArmorPredictionResult ProcessFrame(const hal::CameraFrame& frame,
                                                   const ArmorPnpFrameResult& pnp_result);

  ArmorPredictorConfig config;
  detail::ArmorEkf filter;
  TrackerState tracker_state{TrackerState::LOST};
  std::optional<ArmorLabel> label;
  std::optional<hal::CameraFrame::ArmorType> type;
  int detect_count{0};
  int temp_lost_count{0};
  bool has_timestamp{false};
  bool timestamp_uses_capture{false};
  std::uint64_t last_capture_timestamp_ns{0};
  std::chrono::steady_clock::time_point last_receive_time{};
  std::string last_reset_reason;
};

const char* TrackerStateName(TrackerState state) noexcept {
  switch (state) {
    case TrackerState::LOST:
      return "lost";
    case TrackerState::DETECTING:
      return "detecting";
    case TrackerState::TRACKING:
      return "tracking";
    case TrackerState::TEMP_LOST:
      return "temp_lost";
  }
  return "unknown";
}

std::vector<detail::Observation> ArmorPredictor::Impl::ExtractObservations(
    const hal::CameraFrame& frame, const ArmorPnpFrameResult& pnp_result) const {
  std::vector<detail::Observation> result;
  if (!frame.geometry)
    return result;
  const auto WORLD_T_CAMERA =
      geometry::Compose(frame.geometry->world_t_gimbal, frame.geometry->gimbal_t_camera_optical);
  // 真值基准 PnP 只用于评估；跟踪链严格消费正式检测输入，避免仿真真值泄漏。
  for (const auto& attempt : pnp_result.attempts) {
    if (attempt.source != PnpInputSource::DETECTION || !attempt.estimate)
      continue;
    const auto& estimate = *attempt.estimate;
    if (estimate.label > static_cast<std::uint8_t>(ArmorLabel::BASE_BIG))
      continue;
    const auto LABEL = static_cast<ArmorLabel>(estimate.label);
    if (!SupportedLabel(LABEL))
      continue;
    const auto WORLD_T_ARMOR = geometry::Compose(WORLD_T_CAMERA, estimate.camera_t_armor);
    const auto NORMAL = geometry::TransformVector(WORLD_T_ARMOR, geometry::Vector3::UnitZ());
    if (!NORMAL.allFinite() || std::hypot(NORMAL.x(), NORMAL.y()) < 1.0e-8)
      continue;
    cv::Point2f center{};
    for (const auto& corner : estimate.image_corners)
      center += corner;
    center *= 0.25F;
    result.push_back({.input_index = estimate.input_index,
                      .label = LABEL,
                      .type = estimate.type,
                      .world_t_armor = WORLD_T_ARMOR,
                      .image_center = center,
                      .yaw = std::atan2(NORMAL.y(), NORMAL.x())});
  }
  return result;
}

void ArmorPredictor::Impl::Reset(std::string reason) {
  tracker_state = TrackerState::LOST;
  label.reset();
  type.reset();
  filter.Reset();
  detect_count = 0;
  temp_lost_count = 0;
  last_reset_reason = std::move(reason);
}

void ArmorPredictor::Impl::Initialize(const detail::Observation& observation) {
  filter.Initialize(observation, config);
  label = observation.label;
  type = observation.type;
  tracker_state = TrackerState::DETECTING;
  detect_count = 1;
  temp_lost_count = 0;
  last_reset_reason.clear();
}

ArmorPredictionResult ArmorPredictor::Impl::Snapshot(std::uint64_t sequence, double dt) const {
  ArmorPredictionResult result;
  result.sequence = sequence;
  result.state = tracker_state;
  result.label = label;
  result.type = type;
  result.dt_s = dt;
  result.reset_reason = last_reset_reason;
  if (tracker_state == TrackerState::LOST)
    return result;
  const auto& state = filter.State();
  const auto& covariance = filter.Covariance();
  for (int index = 0; index < detail::K_STATE_SIZE; ++index) {
    result.state_vector[index] = state[index];
    result.covariance_diagonal[index] = covariance(index, index);
  }
  result.velocity_world = {state[1], state[3], state[5]};
  const double ARMOR_ROLL_RAD = label ? ArmorRollForLabel(*label, config) : 0.0;
  // Horizon 只对当前后验状态做匀速外推，不修改滤波器本身及其协方差。
  result.horizons.reserve(config.prediction_horizons_s.size());
  for (double seconds : config.prediction_horizons_s) {
    auto future = state;
    future[0] += future[1] * seconds;
    future[2] += future[3] * seconds;
    future[4] += future[5] * seconds;
    future[6] = detail::WrapAngle(future[6] + future[7] * seconds);
    PredictionHorizon horizon;
    horizon.seconds = seconds;
    horizon.center_world = {future[0], future[2], future[4]};
    horizon.yaw = future[6];
    for (int slot = 0; slot < 4; ++slot) {
      horizon.armors[slot] = {
          .slot = slot, .world_t_armor = detail::WorldArmorPose(future, slot, ARMOR_ROLL_RAD)};
    }
    result.horizons.push_back(std::move(horizon));
  }
  return result;
}

ArmorPredictionResult ArmorPredictor::Impl::ProcessFrame(const hal::CameraFrame& frame,
                                                         const ArmorPnpFrameResult& pnp_result) {
  double dt = 0.0;
  const bool USE_CAPTURE = frame.capture_timestamp_ns.has_value();
  // 单次跟踪周期必须使用同一种时钟；来源切换会使 dt 失去可比性，因此直接重置。
  if (has_timestamp) {
    if (USE_CAPTURE != timestamp_uses_capture) {
      Reset("timestamp_source_changed");
    } else if (USE_CAPTURE) {
      if (*frame.capture_timestamp_ns <= last_capture_timestamp_ns) {
        Reset("non_monotonic_timestamp");
      } else {
        dt = static_cast<double>(*frame.capture_timestamp_ns - last_capture_timestamp_ns) * 1.0e-9;
      }
    } else {
      dt = std::chrono::duration<double>(frame.receive_steady_time - last_receive_time).count();
      if (dt <= 0.0)
        Reset("non_monotonic_timestamp");
    }
    if (dt > config.max_dt_s)
      Reset("large_dt");
  }
  has_timestamp = true;
  timestamp_uses_capture = USE_CAPTURE;
  if (USE_CAPTURE)
    last_capture_timestamp_ns = *frame.capture_timestamp_ns;
  last_receive_time = frame.receive_steady_time;

  if (tracker_state != TrackerState::LOST && dt > 0.0)
    filter.Predict(dt, config);
  const auto OBSERVATIONS = ExtractObservations(frame, pnp_result);

  if (tracker_state == TrackerState::LOST) {
    if (!frame.geometry || OBSERVATIONS.empty())
      return Snapshot(frame.sequence, dt);
    const auto& calibration = frame.geometry->calibration;
    // 优先选择战术优先级更高的标签；同优先级选择离主点更近、PnP 通常更稳定的观测。
    const auto BEST = std::min_element(
        OBSERVATIONS.begin(), OBSERVATIONS.end(),
        [&](const detail::Observation& left, const detail::Observation& right) {
          const int left_priority = LabelPriority(left.label, config);
          const int right_priority = LabelPriority(right.label, config);
          if (left_priority != right_priority)
            return left_priority < right_priority;
          const double left_distance = std::hypot(left.image_center.x - calibration.cx,
                                                  left.image_center.y - calibration.cy);
          const double right_distance = std::hypot(right.image_center.x - calibration.cx,
                                                   right.image_center.y - calibration.cy);
          return left_distance < right_distance;
        });
    Initialize(*BEST);
    auto result = Snapshot(frame.sequence, dt);
    result.associations.push_back({.input_index = BEST->input_index,
                                   .slot = 0,
                                   .position_error_m = 0.0,
                                   .yaw_error_rad = 0.0,
                                   .observed_position_world = BEST->world_t_armor.translation,
                                   .predicted_position_world = BEST->world_t_armor.translation,
                                   .rejection_reason = {}});
    return result;
  }

  std::vector<detail::Observation> candidates;
  // 已锁定目标期间禁止跨标签或装甲尺寸更新，避免相邻机器人污染同一滤波状态。
  for (const auto& observation : OBSERVATIONS) {
    if (observation.label == *label && observation.type == *type)
      candidates.push_back(observation);
  }
  auto result = Snapshot(frame.sequence, dt);
  std::vector<int> slots;
  if (frame.geometry) {
    slots = detail::AssociateArmors(candidates, filter.State(), config, result.associations);
  }
  const bool FOUND = std::any_of(slots.begin(), slots.end(), [](int slot) { return slot >= 0; });
  if (FOUND)
    filter.Update(candidates, slots, *frame.geometry, config, result);

  // 状态转换发生在量测更新之后，使返回快照表达本帧处理完成后的最终状态。
  if (tracker_state == TrackerState::DETECTING) {
    if (FOUND) {
      if (++detect_count >= config.min_detect_count)
        tracker_state = TrackerState::TRACKING;
    } else {
      Reset("detecting_missed");
    }
  } else if (tracker_state == TrackerState::TRACKING) {
    if (!FOUND) {
      tracker_state = TrackerState::TEMP_LOST;
      temp_lost_count = 1;
    }
  } else if (tracker_state == TrackerState::TEMP_LOST) {
    if (FOUND) {
      tracker_state = TrackerState::TRACKING;
      temp_lost_count = 0;
    } else if (++temp_lost_count > config.max_temp_lost_count) {
      Reset("temporary_loss_timeout");
    }
  }
  if (tracker_state != TrackerState::LOST && filter.Diverged(config))
    Reset("state_diverged");

  auto final_result = Snapshot(frame.sequence, dt);
  final_result.associations = std::move(result.associations);
  final_result.innovation = std::move(result.innovation);
  final_result.nis = result.nis;
  return final_result;
}

ArmorPredictor::ArmorPredictor(ArmorPredictorConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

ArmorPredictor::~ArmorPredictor() = default;

ArmorPredictor::ArmorPredictor(const ArmorPredictor& other)
    : impl_(other.impl_ ? std::make_unique<Impl>(*other.impl_) : nullptr) {}

ArmorPredictor& ArmorPredictor::operator=(const ArmorPredictor& other) {
  if (this != &other)
    impl_ = other.impl_ ? std::make_unique<Impl>(*other.impl_) : nullptr;
  return *this;
}

ArmorPredictor::ArmorPredictor(ArmorPredictor&& other) noexcept = default;

ArmorPredictor& ArmorPredictor::operator=(ArmorPredictor&& other) noexcept = default;

ArmorPredictionResult ArmorPredictor::ProcessFrame(const hal::CameraFrame& frame,
                                                   const ArmorPnpFrameResult& pnp_result) {
  return impl_->ProcessFrame(frame, pnp_result);
}

}  // namespace mv::modules
