/**
 * @file ekf_tracker.cpp
 * @brief EKF 目标跟踪状态机实现（Stage 8-B）
 *
 * 【参考】sp_vision_25/tasks/auto_aim/tracker.cpp
 *
 * 【与 sp 的主要差异】
 *   1. DETECTING → TRACKING 增加 target_->Converged() 检查；
 *   2. 使用 EkfTrackTarget 而非 sp 的 Target，接口名 PascalCase；
 *   3. NIS 发散检测放在 target_->Diverged()（radius 约束）和 NIS 窗口两处。
 */
#include "ekf_tracker.hpp"

#include "core/logger.hpp"

#include <algorithm>
#include <numeric>

namespace mv::modules::detail {

// ── 工具 ─────────────────────────────────────────────────────────────────────

const char* EkfTracker::StateName(State state) noexcept {
  switch (state) {
    case State::LOST:
      return "lost";
    case State::DETECTING:
      return "detecting";
    case State::TRACKING:
      return "tracking";
    case State::TEMP_LOST:
      return "temp_lost";
    default:
      return "unknown";
  }
}

// ── 构造 ─────────────────────────────────────────────────────────────────────

EkfTracker::EkfTracker(const EkfTrackerParams& params)
    : last_t_(std::chrono::steady_clock::now()), params_(params) {}

// ── Reset ────────────────────────────────────────────────────────────────────

void EkfTracker::Reset() {
  state_ = State::LOST;
  target_.reset();
  detect_count_ = 0;
  detecting_lost_count_ = 0;
  temp_lost_count_ = 0;
  max_temp_lost_ = params_.max_temp_lost_count;
  MV_LOG_INFO("EkfTracker", "Reset to LOST");
}

// ── Track（主入口）──────────────────────────────────────────────────────────

std::optional<EkfTrackTarget> EkfTracker::Track(const std::vector<Detection>& detections,
                                                std::chrono::steady_clock::time_point t,
                                                ArmorColor enemy_color) {
  // 1. 计算并检查 dt
  const double dt = std::chrono::duration<double>(t - last_t_).count();
  last_t_ = t;

  if (state_ != State::LOST && dt > params_.max_dt_sec) {
    MV_LOG_WARN("EkfTracker", "Large dt: {:.3f}s, resetting", dt);
    Reset();
    return std::nullopt;
  }

  // 2. 过滤颜色不匹配的检测
  std::vector<Detection> valid_dets;
  valid_dets.reserve(detections.size());
  for (const auto& d : detections) {
    if (d.color == enemy_color && d.is_solved) {
      valid_dets.push_back(d);
    }
  }

  // 3. 排序：优先级（ArmorNumber 枚举值小 = 优先级高），次按图像中心距离
  std::sort(valid_dets.begin(), valid_dets.end(), [](const Detection& a, const Detection& b) {
    if (a.number != b.number)
      return static_cast<int>(a.number) < static_cast<int>(b.number);
    return a.distance_to_center < b.distance_to_center;
  });

  // 4. 按状态机分支处理
  bool found = false;
  if (state_ == State::LOST) {
    found = SetTarget(valid_dets, t);
  } else {
    found = UpdateTarget(valid_dets, t);
  }

  // 5. 状态机转换
  StateMachine(found);

  // 6. 发散检测（radius 物理约束）
  if (state_ != State::LOST && target_ && target_->Diverged()) {
    MV_LOG_DEBUG("EkfTracker", "Target diverged (radius), resetting");
    Reset();
    return std::nullopt;
  }

  // 7. NIS 发散检测（观测与模型不符）
  if (state_ != State::LOST && target_) {
    const auto& nis_failures = target_->ekf().recent_nis_failures;
    const int fail_count = std::accumulate(nis_failures.begin(), nis_failures.end(), 0);
    const int window = static_cast<int>(target_->ekf().window_size);
    if (window > 0 && fail_count >= static_cast<int>(0.4 * window)) {
      MV_LOG_DEBUG("EkfTracker", "Target NIS diverged (fail_rate={:.2f}), resetting",
                   static_cast<double>(fail_count) / window);
      Reset();
      return std::nullopt;
    }
  }

  if (state_ == State::LOST || !target_)
    return std::nullopt;
  return *target_;
}

// ── SetTarget ────────────────────────────────────────────────────────────────

bool EkfTracker::SetTarget(const std::vector<Detection>& detections,
                           std::chrono::steady_clock::time_point t) {
  if (detections.empty())
    return false;

  const auto& det = detections.front();

  // 根据目标类型确定装甲板数（balance=2，outpost/base=3，普通=4）
  const bool is_balance = (det.type == ArmorType::BIG) &&
                          (det.number == ArmorNumber::THREE || det.number == ArmorNumber::FOUR ||
                           det.number == ArmorNumber::FIVE);
  int armor_num = 4;
  if (is_balance) {
    armor_num = 2;
  } else if (det.number == ArmorNumber::OUTPOST || det.number == ArmorNumber::BASE) {
    armor_num = 3;
  }

  // 初始旋转半径
  double radius = params_.init_radius_small;
  if (det.number == ArmorNumber::OUTPOST) {
    radius = params_.init_radius_outpost;
  } else if (det.type == ArmorType::BIG) {
    radius = params_.init_radius_big;
  }

  // 初始协方差（若参数未配置则使用 sp 默认值）
  Eigen::VectorXd P0_diag(11);
  if (is_balance) {
    P0_diag << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
  } else if (det.number == ArmorNumber::OUTPOST) {
    P0_diag << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
  } else if (det.number == ArmorNumber::BASE) {
    P0_diag << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
  } else if (params_.P0_diag.size() == 11) {
    // 全局 P0 仅用于普通目标，避免覆盖 outpost/base/balance 分支语义。
    P0_diag = params_.P0_diag;
  } else {
    P0_diag << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
  }

  target_ = EkfTrackTarget(
      det, t, radius, armor_num, P0_diag,
      ProcessNoiseParams{params_.process_noise_pos, params_.process_noise_ang,
                         params_.process_noise_outpost_pos, params_.process_noise_outpost_ang});
  return true;
}

// ── UpdateTarget ─────────────────────────────────────────────────────────────

bool EkfTracker::UpdateTarget(const std::vector<Detection>& detections,
                              std::chrono::steady_clock::time_point t) {
  // 时间预测步（即使本帧无新检测也推进状态）
  target_->Predict(t);

  // 在 detections 中寻找同 ArmorNumber（且 is_solved）的最优匹配
  const Detection* best = nullptr;
  for (const auto& det : detections) {
    if (det.number != target_->name || det.type != target_->armor_type)
      continue;
    if (!best || det.distance_to_center < best->distance_to_center) {
      best = &det;
    }
  }

  if (!best)
    return false;

  target_->Update(*best);
  return true;
}

// ── StateMachine ─────────────────────────────────────────────────────────────

void EkfTracker::StateMachine(bool found) {
  switch (state_) {
    case State::LOST: {
      if (!found)
        return;
      state_ = State::DETECTING;
      detect_count_ = 1;
      MV_LOG_DEBUG("EkfTracker", "LOST → DETECTING");
      break;
    }

    case State::DETECTING: {
      if (found) {
        ++detect_count_;
        detecting_lost_count_ = 0;  // 检测到则重置漏检计数
        // 需要达到最小检测帧数 AND EKF 已经收敛才转到 TRACKING
        if (detect_count_ >= params_.min_detect_count && target_ && target_->Converged()) {
          state_ = State::TRACKING;
          detecting_lost_count_ = 0;
          MV_LOG_INFO("EkfTracker", "DETECTING → TRACKING (count={})", detect_count_);
        }
      } else {
        ++detecting_lost_count_;
        if (detecting_lost_count_ > params_.max_detecting_lost_count) {
          // 连续漏检超过容忍帧数才真正放弃
          detect_count_ = 0;
          detecting_lost_count_ = 0;
          state_ = State::LOST;
          MV_LOG_DEBUG("EkfTracker", "DETECTING → LOST (lost_count={})", detecting_lost_count_);
        } else {
          MV_LOG_DEBUG("EkfTracker", "DETECTING miss {}/{}", detecting_lost_count_,
                       params_.max_detecting_lost_count);
        }
      }
      break;
    }

    case State::TRACKING: {
      if (found)
        return;  // 保持 TRACKING
      temp_lost_count_ = 1;
      // 前哨站允许更长的 temp_lost 容忍
      max_temp_lost_ = (target_ && target_->name == ArmorNumber::OUTPOST)
                           ? params_.outpost_max_temp_lost_count
                           : params_.max_temp_lost_count;
      state_ = State::TEMP_LOST;
      MV_LOG_DEBUG("EkfTracker", "TRACKING → TEMP_LOST");
      break;
    }

    case State::TEMP_LOST: {
      if (found) {
        state_ = State::TRACKING;
        MV_LOG_DEBUG("EkfTracker", "TEMP_LOST → TRACKING");
      } else {
        ++temp_lost_count_;
        if (temp_lost_count_ > max_temp_lost_) {
          state_ = State::LOST;
          target_.reset();
          MV_LOG_DEBUG("EkfTracker", "TEMP_LOST → LOST (count={})", temp_lost_count_);
        }
      }
      break;
    }
  }
}

}  // namespace mv::modules::detail
