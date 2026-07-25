/**
 * @file simple_predictor.cpp
 * @brief 简单直通预测器实现
 */
#include "simple_predictor.hpp"

#include "core/logger.hpp"
#include "factory/factory.hpp"

#include <algorithm>
#include <limits>

namespace mv::modules {

namespace {
const bool SIMPLE_PREDICTOR_REGISTERED = [] {
  ::mv::Factory<::mv::IPredictor>::Register("simple",
                                            [] { return std::make_unique<SimplePredictor>(); });
  return true;
}();
}  // namespace

// ── 构造 / 析构 ────────────────────────────────────────────────────────────

SimplePredictor::SimplePredictor() = default;
SimplePredictor::~SimplePredictor() = default;

// ── 状态名称 ──────────────────────────────────────────────────────────────

const char* SimplePredictor::StateName(TrackState state) noexcept {
  switch (state) {
    case TrackState::LOST:
      return "lost";
    case TrackState::DETECTING:
      return "detecting";
    case TrackState::TRACKING:
      return "tracking";
    case TrackState::TEMP_LOST:
      return "temp_lost";
    default:
      return "unknown";
  }
}

// ── Init ──────────────────────────────────────────────────────────────────

bool SimplePredictor::Init(const YAML::Node& config) {
  if (config && config["auto_aim"] && config["auto_aim"]["tracker"]) {
    const auto& trk = config["auto_aim"]["tracker"];
    if (trk["min_detect_count"]) {
      min_detect_count_ = trk["min_detect_count"].as<int>();
    }
    if (trk["max_temp_lost_count"]) {
      max_temp_lost_count_ = trk["max_temp_lost_count"].as<int>();
    }
  }
  initialized_ = true;
  MV_LOG_INFO("SimplePredictor", "Init OK — min_detect={}, max_lost={}", min_detect_count_,
              max_temp_lost_count_);
  return true;
}

// ── Reset ─────────────────────────────────────────────────────────────────

void SimplePredictor::Reset() {
  state_ = TrackState::LOST;
  detect_count_ = 0;
  lost_count_ = 0;
  tracked_id_ = ArmorNumber::UNKNOWN;
  track_target_ = TrackTarget{};
  MV_LOG_INFO("SimplePredictor", "Tracker reset to LOST");
}

// ── Predict ───────────────────────────────────────────────────────────────

GimbalControl SimplePredictor::Predict(const std::vector<Detection>& detections,
                                       std::chrono::steady_clock::time_point timestamp,
                                       ArmorColor enemy_color) {
  GimbalControl ctrl;
  ctrl.timestamp = timestamp;

  const Detection* best = SelectBest(detections, enemy_color);

  // ── 状态机转换 ─────────────────────────────────────────────────────────
  using TS = TrackState;

  switch (state_) {
    case TS::LOST:
      if (best != nullptr) {
        state_ = TS::DETECTING;
        detect_count_ = 1;
        tracked_id_ = best->number;
        MV_LOG_DEBUG("SimplePredictor", "LOST → DETECTING");
      }
      break;

    case TS::DETECTING:
      if (best != nullptr) {
        ++detect_count_;
        if (detect_count_ >= min_detect_count_) {
          state_ = TS::TRACKING;
          MV_LOG_INFO("SimplePredictor", "DETECTING → TRACKING ({})", detect_count_);
        }
      } else {
        state_ = TS::LOST;
        detect_count_ = 0;
        MV_LOG_DEBUG("SimplePredictor", "DETECTING → LOST (lost target)");
      }
      break;

    case TS::TRACKING:
      if (best == nullptr) {
        state_ = TS::TEMP_LOST;
        lost_count_ = 1;
        MV_LOG_DEBUG("SimplePredictor", "TRACKING → TEMP_LOST");
      }
      break;

    case TS::TEMP_LOST:
      if (best != nullptr) {
        state_ = TS::TRACKING;
        lost_count_ = 0;
        MV_LOG_DEBUG("SimplePredictor", "TEMP_LOST → TRACKING (target reappeared)");
      } else {
        ++lost_count_;
        if (lost_count_ > max_temp_lost_count_) {
          state_ = TS::LOST;
          lost_count_ = 0;
          tracked_id_ = ArmorNumber::UNKNOWN;
          MV_LOG_INFO("SimplePredictor", "TEMP_LOST → LOST (timeout {})", lost_count_);
        }
      }
      break;
  }

  // ── 输出控制量 ────────────────────────────────────────────────────────────
  if ((state_ == TS::TRACKING || state_ == TS::TEMP_LOST) && best != nullptr) {
    ctrl.tracking = true;
    ctrl.yaw = best->yaw_angle;
    ctrl.pitch = best->pitch_angle;
    ctrl.distance = best->xyz_in_gimbal.norm();
    FillTrackTarget(track_target_, *best, StateName(state_));
  } else if (state_ == TS::TEMP_LOST) {
    // 保持上一帧的数据，让串口继续发送最后的控制量
    ctrl.tracking = true;
    ctrl.yaw = track_target_.yaw_predicted;
    ctrl.pitch = track_target_.pitch_predicted;
    ctrl.distance = track_target_.position.norm();
  } else {
    ctrl.tracking = false;
    track_target_.is_tracking = false;
    track_target_.tracker_state = StateName(state_);
  }

  return ctrl;
}

// ── 内部方法 ──────────────────────────────────────────────────────────────

const Detection* SimplePredictor::SelectBest(const std::vector<Detection>& detections,
                                             ArmorColor color) {
  const Detection* best = nullptr;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto& det : detections) {
    if (!det.is_solved) {
      continue;
    }
    if (color != ArmorColor::UNKNOWN && det.color != ArmorColor::UNKNOWN && det.color != color) {
      continue;
    }
    if (det.distance_to_center < min_dist) {
      min_dist = det.distance_to_center;
      best = &det;
    }
  }
  return best;
}

void SimplePredictor::FillTrackTarget(TrackTarget& target, const Detection& det,
                                      const std::string& state) {
  target.is_tracking = true;
  target.number = det.number;
  target.color = det.color;
  target.position = det.xyz_in_gimbal;
  target.velocity = Eigen::Vector3d::Zero();  // 无 EKF，速度为零
  target.yaw_predicted = det.yaw_angle;
  target.pitch_predicted = det.pitch_angle;
  target.tracker_state = state;
}

}  // namespace mv::modules
