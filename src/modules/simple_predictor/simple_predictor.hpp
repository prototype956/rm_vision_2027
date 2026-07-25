/**
 * @file simple_predictor.hpp
 * @brief 简单直通预测器（无 EKF，仅做目标筛选与状态跟踪）
 *
 * 【设计说明】
 *   SimplePredictor 是 Stage 6 的集成验证用版本，不含卡尔曼滤波：
 *   - 按到图像中心距离选出"最优目标"；
 *   - 维护简化跟踪状态机：lost → detecting → tracking → temp_lost；
 *   - 直接将目标的 yaw/pitch 角作为 GimbalControl 输出（无预判延迟补偿）。
 *
 *   后续可替换为 EkfPredictor（实现相同的 IPredictor 接口）。
 *
 * 【YAML 配置字段】（来自 vision.yaml 的 auto_aim.tracker 节点）
 * @code
 *   auto_aim:
 *     tracker:
 *       min_detect_count:         5   # 进入 tracking 所需连续帧数
 *       max_temp_lost_count:     15   # temp_lost 超此帧数则退回 lost
 * @endcode
 *
 * 工厂键：`"simple"`
 */
#pragma once

#include "interfaces/i_predictor.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace mv::modules {

class SimplePredictor final : public IPredictor {
 public:
  SimplePredictor();
  ~SimplePredictor() override;

  SimplePredictor(const SimplePredictor&) = delete;
  SimplePredictor& operator=(const SimplePredictor&) = delete;
  SimplePredictor(SimplePredictor&&) = delete;
  SimplePredictor& operator=(SimplePredictor&&) = delete;

  bool Init(const YAML::Node& config) override;

  [[nodiscard]] GimbalControl Predict(const std::vector<Detection>& detections,
                                      std::chrono::steady_clock::time_point timestamp,
                                      ArmorColor enemy_color) override;

  [[nodiscard]] TrackTarget GetTrackTarget() const override { return track_target_; }

  void Reset() override;

  [[nodiscard]] bool IsInitialized() const noexcept override { return initialized_; }

 private:
  // ── 跟踪器状态枚举 ────────────────────────────────────────────────────────

  enum class TrackState : uint8_t { LOST = 0, DETECTING, TRACKING, TEMP_LOST };

  static const char* StateName(TrackState state) noexcept;

  // ── 状态 ─────────────────────────────────────────────────────────────────

  TrackState state_{TrackState::LOST};
  TrackTarget track_target_;

  int detect_count_{0};  ///< 连续检测帧计数（DETECTING 阶段）
  int lost_count_{0};    ///< 连续丢失帧计数（TEMP_LOST 阶段）
  ArmorNumber tracked_id_{ArmorNumber::UNKNOWN};

  // ── 参数 ─────────────────────────────────────────────────────────────────

  static constexpr int DEFAULT_MIN_DETECT = 5;
  static constexpr int DEFAULT_MAX_LOST = 15;

  int min_detect_count_{DEFAULT_MIN_DETECT};
  int max_temp_lost_count_{DEFAULT_MAX_LOST};

  bool initialized_{false};

  // ── 内部方法 ──────────────────────────────────────────────────────────────

  /** 从 detections 中选出最靠近图像中心的已解算目标，nullptr = 无目标 */
  [[nodiscard]] static const Detection* SelectBest(const std::vector<Detection>& detections,
                                                   ArmorColor color);

  /** 根据 Detection 填充 TrackTarget 快照 */
  static void FillTrackTarget(TrackTarget& target, const Detection& det, const std::string& state);
};

}  // namespace mv::modules
