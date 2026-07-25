/**
 * @file ekf_tracker.hpp
 * @brief 基于 EKF 的目标跟踪状态机
 *
 * 【职责】
 *   EkfTracker 是 EkfPredictor 的内部状态机，负责：
 *   1. 管理跟踪状态（LOST → DETECTING → TRACKING → TEMP_LOST → LOST）；
 *   2. 在 LOST/DETECTING 阶段从 Detection 列表中选择最优目标（set_target）；
 *   3. 在 TRACKING/TEMP_LOST 阶段将新 Detection 与当前目标匹配并更新 EKF（update_target）；
 *   4. 检测 EKF 发散/收敛状态，驱动状态机转换；
 *   5. 输出当前 EkfTrackTarget（供 TrajectorySolver 使用）。
 *
 * 【状态机】
 * @code
 *   LOST ──(连续检测 min_detect 帧)──► DETECTING
 *           ◄──(目标消失)──────────────┘
 *   DETECTING ──(EKF 收敛)──► TRACKING
 *              ◄──(目标消失)──
 *   TRACKING ──(目标短暂丢失)──► TEMP_LOST
 *            ◄──(重新找到)─────────┘
 *            ◄──(发散/大 dt)──────── LOST（直跳）
 *   TEMP_LOST ──(超过 max_temp_lost 帧)──► LOST
 *             ──(重新找到)──► TRACKING
 * @endcode
 *
 * 【目标切换策略】
 *   当前目标丢失后，优先选择：
 *   1. ArmorPriority 最高（数字最小）的目标；
 *   2. 同等优先级下选离图像中心最近的目标；
 *   此策略直接对 detections 排序后取首元素，无需维护多目标列表。
 *
 * 【大 dt 保护】
 *   若相邻两帧 dt > 0.1s（相机掉线或暂停），直接重置为 LOST，
 *   防止 EKF 状态转移矩阵中 dt² / dt³ 项数值爆炸。
 *
 * 【线程安全】
 *   非线程安全，由 EkfPredictor 在单一线程顺序调用。
 */
#pragma once

#include "ekf_track_target.hpp"
#include "interfaces/types.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <optional>

namespace mv::modules::detail {

/**
 * @brief EKF 目标跟踪状态机参数（从 YAML 加载，按目标类型区分）
 */
struct EkfTrackerParams {
  int min_detect_count{5};  ///< DETECTING 转 TRACKING 所需连续更新帧数
  int max_detecting_lost_count{2};  ///< DETECTING 阶段允许的最大连续漏检帧数（超出才回 LOST）
  int max_temp_lost_count{15};          ///< TEMP_LOST 超此帧数重置为 LOST
  int outpost_max_temp_lost_count{30};  ///< 前哨站专用（转速慢，允许更长的丢失容忍）
  double max_dt_sec{0.1};  ///< dt 超过此值认为相机掉线，重置 LOST（单位：s）

  // EKF 初始化参数
  double init_radius_small{0.27};    ///< 小装甲板初始旋转半径（m）
  double init_radius_big{0.27};      ///< 大装甲板初始旋转半径（m）
  double init_radius_outpost{0.26};  ///< 前哨站初始旋转半径（m）

  // 初始协方差对角（11 维）
  Eigen::VectorXd P0_diag;  ///< 11 维，从配置读取

  // 过程噪声
  double process_noise_pos{100.0};  ///< 普通车位置过程噪声方差
  double process_noise_ang{400.0};  ///< 普通车角度过程噪声方差
  double process_noise_outpost_pos{10.0};
  double process_noise_outpost_ang{0.1};

  // 发散检测阈值（协方差迹）
  double divergence_threshold{1e6};
};

/**
 * @brief EKF 目标跟踪状态机
 *
 * 持有当前唯一跟踪目标 EkfTrackTarget，驱动状态转换。
 */
class EkfTracker {
 public:
  /** @brief 跟踪器内部状态枚举 */
  enum class State : uint8_t { LOST = 0, DETECTING, TRACKING, TEMP_LOST };

  /** @return 状态名字符串（日志/TrackTarget.tracker_state 输出用）*/
  static const char* StateName(State state) noexcept;

  // ── 构造 ─────────────────────────────────────────────────────────────

  explicit EkfTracker(const EkfTrackerParams& params);

  // ── 核心接口 ──────────────────────────────────────────────────────────

  /**
   * @brief 驱动一帧跟踪，输出当前跟踪目标（可选）
   *
   * 调用顺序：
   *   1. 检查 dt 是否过大（大 dt 保护）；
   *   2. 按优先级+距离排序 detections；
   *   3. 根据当前状态：
   *      - LOST/DETECTING → SetTarget()（查找新目标）；
   *      - TRACKING/TEMP_LOST → UpdateTarget()（更新现有目标）；
   *   4. 状态机转换（StateMachine()）；
   *   5. 检查发散/NIS 失败率，必要时重置。
   *
   * @param detections  本帧检测结果（已填充 xyz_in_world）
   * @param t           当前帧时间戳
   * @param enemy_color 敌方颜色过滤
   * @return 当前跟踪目标（仅 TRACKING 或 TEMP_LOST 状态时有值）
   */
  [[nodiscard]] std::optional<EkfTrackTarget> Track(const std::vector<Detection>& detections,
                                                    std::chrono::steady_clock::time_point t,
                                                    ArmorColor enemy_color);

  // ── 状态查询 ──────────────────────────────────────────────────────────

  /** @return 当前跟踪器状态枚举 */
  [[nodiscard]] State GetState() const noexcept { return state_; }

  /** @return 当前跟踪器状态名字符串（供 TrackTarget.tracker_state 填充）*/
  [[nodiscard]] const char* GetStateName() const noexcept { return StateName(state_); }

  /** @return 当前跟踪目标（仅在 TRACKING / TEMP_LOST 时有效）*/
  [[nodiscard]] const std::optional<EkfTrackTarget>& CurrentTarget() const noexcept {
    return target_;
  }

  // ── 重置 ─────────────────────────────────────────────────────────────

  /** @brief 清除所有跟踪状态，回到 LOST */
  void Reset();

 private:
  State state_{State::LOST};
  std::optional<EkfTrackTarget> target_{};

  int detect_count_{0};          ///< 连续检测帧计数（DETECTING 阶段）
  int detecting_lost_count_{0};  ///< DETECTING 阶段连续漏检帧计数
  int temp_lost_count_{0};       ///< 连续丢失帧计数（TEMP_LOST 阶段）
  int max_temp_lost_{0};  ///< 当前目标允许的最大丢失帧数（普通车/前哨站有别）

  std::chrono::steady_clock::time_point last_t_;
  EkfTrackerParams params_;

  // ── 内部辅助 ─────────────────────────────────────────────────────────

  /**
   * @brief 从检测列表中建立新跟踪目标
   * @return true 成功建立目标（detections 非空且颜色匹配）
   */
  bool SetTarget(const std::vector<Detection>& detections, std::chrono::steady_clock::time_point t);

  /**
   * @brief 将新检测结果匹配并更新已有目标 EKF
   * @return true 找到匹配检测并完成 Update（false = 本帧无匹配 = temp_lost）
   */
  bool UpdateTarget(const std::vector<Detection>& detections,
                    std::chrono::steady_clock::time_point t);

  /** @brief 根据 found（本帧是否有匹配）驱动状态机转换 */
  void StateMachine(bool found);
};

}  // namespace mv::modules::detail
