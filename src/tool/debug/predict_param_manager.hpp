/**
 * @file predict_param_manager.hpp
 * @brief 预测调试测试的参数状态管理器
 *
 * 将 predict_voter_test.cpp 中散落的以下内容集中管理：
 *   - ParamState 结构体（所有可调参数快照）
 *   - Foxglove 参数面板双向同步（PushToFoxglove / Register）
 *   - YAML 注入（InjectParamsToYaml）
 *   - EKF / Voter 重初始化标志（ConsumeReinitEkf / ConsumeReinitVoter）
 *   - tracking/target_state、voter/decision、debug/params JSON 构建器
 *
 * 【线程安全】
 *   ParamState 由 Mutex() 保护；Register() 注册的回调来自 WebSocket 线程，
 *   仅写 state_ 并设置 atomic 标志，不直接调用算法 Init()。
 *   主循环持锁读取 State() 快照后，调用 ConsumeReinitXxx() 判断是否需要重初始化。
 */
#pragma once

#include "interfaces/types.hpp"
#include "tool/foxglove/foxglove_sink.hpp"

#include <atomic>
#include <mutex>
#include <string>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace mv::tool {

// ============================================================================
// ParamState — 所有可调参数快照
// ============================================================================

/**
 * @brief 所有可调参数的快照
 *
 * 主循环每帧读取此结构用于算法执行；
 * Foxglove 参数回调写入此结构后设置重新初始化标志。
 */
struct ParamState {
  // ── 通用 ─────────────────────────────────────────────────────────────────
  mv::ArmorColor enemy_color{mv::ArmorColor::RED};
  bool loop_video{true};
  double playback_fps{0.0};  ///< 0 = 不限速（默认不节流，可由 Foxglove 参数面板实时调节）

  // ── EKF 跟踪参数 ─────────────────────────────────────────────────────────
  int ekf_min_detect_count{5};
  int ekf_max_detecting_lost_count{2};  ///< DETECTING 阶段漏检容忍帧数
  int ekf_max_temp_lost_count{15};
  double ekf_process_noise_pos{100.0};
  double ekf_process_noise_ang{400.0};
  double ekf_yaw_offset_deg{0.0};
  double ekf_pitch_offset_deg{0.0};
  double ekf_low_speed_delay_ms{100.0};
  double ekf_high_speed_delay_ms{70.0};
  double ekf_bullet_speed{23.0};  ///< 模拟弹速（m/s）

  // ── Voter 参数 ────────────────────────────────────────────────────────────
  bool voter_auto_fire{true};
  int voter_min_lock_frames{5};
  double voter_first_tolerance_rad{0.05};
  int voter_fire_tolerate_frames{3};

  // ── 模拟云台姿态（影响 EkfPredictor 坐标系变换；无真实 IMU 时手动拨）──────
  double sim_yaw_deg{0.0};
  double sim_pitch_deg{0.0};
  double sim_roll_deg{0.0};
};

// ============================================================================
// PredictParamManager
// ============================================================================

/**
 * @brief 预测调试测试的参数管理器
 *
 * 典型用法：
 * @code
 *   PredictParamManager pm;
 *   pm.State().enemy_color = CLI.enemy_color;
 *   pm.State().loop_video  = CLI.is_file_source;
 *   pm.InjectParamsToYaml(root_cfg);
 *
 *   pm.Register(sink);         // 注册 Foxglove 参数回调
 *   pm.PushToFoxglove(sink);   // 推送初始参数快照
 *
 *   // 主循环中
 *   if (pm.ConsumeReinitEkf())   { ... predictor.Reset(); predictor.Init(root_cfg); }
 *   if (pm.ConsumeReinitVoter()) { ... voter.Reset();     voter.Init(root_cfg); }
 *   ParamState snap;
 *   { std::lock_guard lk(pm.Mutex()); snap = pm.State(); }
 *   sink.PublishJson("tracking/target_state",
 *                    PredictParamManager::MakeTargetStateJson(target, ctrl), ts_ns);
 * @endcode
 */
class PredictParamManager {
 public:
  PredictParamManager() = default;

  // ── 参数状态访问 ──────────────────────────────────────────────────────────

  /** 直接访问参数状态（须在 Mutex() 锁下或仅在主循环单线程阶段访问）*/
  [[nodiscard]] ParamState& State() noexcept { return state_; }
  [[nodiscard]] const ParamState& State() const noexcept { return state_; }

  /** 保护 state_ 的互斥锁 */
  [[nodiscard]] std::mutex& Mutex() noexcept { return mutex_; }

  // ── 重初始化标志（consume 语义：读取后自动清除）────────────────────────────

  /**
   * @brief 检查并消费 EKF 重初始化标志
   * @return true 若参数自上次消费后发生了需要重初始化 EKF 的变更
   */
  [[nodiscard]] bool ConsumeReinitEkf() noexcept {
    return reinit_ekf_.exchange(false, std::memory_order_acq_rel);
  }

  /**
   * @brief 检查并消费 Voter 重初始化标志
   * @return true 若参数自上次消费后发生了需要重初始化 Voter 的变更
   */
  [[nodiscard]] bool ConsumeReinitVoter() noexcept {
    return reinit_voter_.exchange(false, std::memory_order_acq_rel);
  }

  // ── Foxglove 参数面板 ─────────────────────────────────────────────────────

  /**
   * @brief 注册 Foxglove 参数回调
   *
   * 回调从 WebSocket 线程调用 → 只写 state_ + 设置 atomic 标志，
   * 不在此线程直接调用算法 Init()（避免与主循环并发）。
   *
   * @param sink  Foxglove 接口（不持有 sink 的所有权，调用方须保证生命周期）
   */
  void Register(FoxgloveSink& sink);

  /**
   * @brief 处理单个 Foxglove 参数更新
   * @return true 表示该参数由预测/投票参数管理器处理
   */
  [[nodiscard]] bool HandleParameter(FoxgloveSink& sink, const std::string& name);

  /**
   * @brief 将当前 state_ 全量推送到 Foxglove 参数面板
   *
   * 在连接新客户端或参数批量更新后调用，确保面板显示与实际状态一致。
   */
  void PushToFoxglove(FoxgloveSink& sink) const;

  // ── YAML 注入 ─────────────────────────────────────────────────────────────

  /**
   * @brief 将 state_ 注入到 root 的对应路径（覆写算法 Init 所读取的字段）
   *
   * EkfPredictor::Init() 读取路径：root["auto_aim"]["ekf_predictor"]
   * CooldownVoter::Init() 读取路径：root["auto_aim"]["cooldown_voter"]
   *
   * 仅覆盖 ParamState 管理的字段，其余字段（如相机内参）保持原值。
   */
  void InjectParamsToYaml(YAML::Node& root) const;

  // ── JSON 构建器（static，可不持有 manager 对象直接调用）────────────────────

  /** @brief 构建 tracking/target_state JSON（TrackTarget 完整状态快照）*/
  static nlohmann::json MakeTargetStateJson(const mv::TrackTarget& t,
                                            const mv::GimbalControl& ctrl);

  /** @brief 构建 voter/decision JSON */
  static nlohmann::json MakeVoterJson(bool fire_permitted, const mv::TrackTarget& t,
                                      bool voter_auto_fire);

  /** @brief 构建 debug/params JSON（当前所有参数快照，用于确认调参已生效）*/
  nlohmann::json MakeParamsSnapshotJson() const;

 private:
  // ── 内部 JSON 类型转换辅助 ────────────────────────────────────────────────

  [[nodiscard]] static double JsonToDouble(const nlohmann::json& v, double fallback) noexcept;
  [[nodiscard]] static int JsonToInt(const nlohmann::json& v, int fallback) noexcept;
  [[nodiscard]] static bool JsonToBool(const nlohmann::json& v, bool fallback) noexcept;

  // ── 数据成员 ──────────────────────────────────────────────────────────────

  ParamState state_;
  mutable std::mutex mutex_;
  std::atomic<bool> reinit_ekf_{false};
  std::atomic<bool> reinit_voter_{false};
};

}  // namespace mv::tool
