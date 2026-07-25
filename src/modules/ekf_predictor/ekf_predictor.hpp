/**
 * @file ekf_predictor.hpp
 * @brief EKF 自瞄预测器（Pimpl 公开接口）
 *
 * 【设计模式：Pimpl（Pointer to Implementation）】
 *   公开头文件（此文件）仅暴露 IPredictor 接口的实现签名和工厂键，
 *   所有私有数据成员（EkfTracker、TrajectorySolver、R_gimbal2world 矩阵等）
 *   完全隐藏在 ekf_predictor.cpp 的 Impl 结构体中，外部模块 include 此头文件
 *   时既不依赖 Eigen 的具体类型，也不依赖 detail/ 子目录的内部头文件。
 *
 *   好处：
 *   1. 上层（PredictNode、main.cpp）只需知道"EkfPredictor 实现了 IPredictor"，
 *      不暴露任何 EKF 内部细节；
 *   2. 修改 EKF 参数结构、状态向量维度等，只需重编 ekf_predictor.cpp，
 *      不触发上层文件的重编译。
 *
 * 【功能说明】
 *   EkfPredictor 是 SimplePredictor 的替换版本，工厂键 "ekf"，
 *   只需修改 vision.yaml 中 `predictor.type: "ekf"` 即可切换，
 *   Pipeline 和 PredictNode 代码**零改动**。
 *
 *   相比 SimplePredictor 的新增能力：
 *   - 11 维 EKF 状态估计（支持多板装甲车旋转建模）；
 *   - IMU 四元数坐标系转换（云台系 → 世界系建模）；
 *   - 飞行时间迭代弹道补偿；
 *   - EKF NIS 收敛检验 + 协方差发散检测。
 *
 * 【YAML 配置字段】（来自 vision.yaml 的 auto_aim.ekf_predictor 节点）
 * @code
 *   auto_aim:
 *     ekf_predictor:
 *       min_detect_count:           5
 *       max_temp_lost_count:        15
 *       outpost_max_temp_lost_count: 30
 *       max_dt_sec:                 0.1
 *       init_radius_small:          0.27
 *       init_radius_big:            0.27
 *       init_radius_outpost:        0.26
 *       process_noise_pos:          100.0
 *       process_noise_ang:          400.0
 *       process_noise_outpost_pos:  10.0
 *       process_noise_outpost_ang:  0.1
 *       divergence_threshold:       1.0e6
 *       # 弹道求解参数
 *       yaw_offset_deg:             0.0
 *       pitch_offset_deg:           0.0
 *       low_speed_delay_ms:         100.0
 *       high_speed_delay_ms:        70.0
 *       decision_speed:             25.0
 *       # EKF 初始协方差对角（11 维）
 *       P0_diag: [1,1,1,1,1,1,1,1,0.05,1e-3,1e-3]
 * @endcode
 *
 * 工厂键：`"ekf"`
 */
#pragma once

#include "interfaces/i_predictor.hpp"

#include <memory>

namespace mv::modules {

/**
 * @brief EKF 自瞄预测器（Pimpl，工厂键 "ekf"）
 *
 * 替换 SimplePredictor 使用，接口完全兼容 IPredictor。
 * 持有 detail::EkfTracker 和 detail::TrajectorySolver 的所有权，
 * 均封装在 Impl 中，不暴露到公开头文件。
 */
class EkfPredictor final : public IPredictor {
 public:
  EkfPredictor();
  ~EkfPredictor() override;

  EkfPredictor(const EkfPredictor&) = delete;
  EkfPredictor& operator=(const EkfPredictor&) = delete;
  EkfPredictor(EkfPredictor&&) = delete;
  EkfPredictor& operator=(EkfPredictor&&) = delete;

  // ── IPredictor 接口实现 ───────────────────────────────────────────────

  bool Init(const YAML::Node& config) override;

  [[nodiscard]] GimbalControl Predict(const std::vector<Detection>& detections,
                                      std::chrono::steady_clock::time_point timestamp,
                                      ArmorColor enemy_color) override;

  [[nodiscard]] TrackTarget GetTrackTarget() const override;

  void Reset() override;

  [[nodiscard]] bool IsInitialized() const noexcept override;

  /**
   * @brief 注入云台姿态四元数（由 PredictNode 在每帧 Predict 前调用）
   *
   * 覆写 IPredictor 的默认空操作，实际存储四元数并在 Predict 内部
   * 将 Detection.xyz_in_gimbal 转换为世界系后送入 EKF。
   *
   * 坐标转换：
   *   R_gimbal2world = R_gimbal2imubody^T · R_imubody2imuabs · R_gimbal2imubody
   *   xyz_in_world   = R_gimbal2world · xyz_in_gimbal
   *   （参见 docs/refractor/tf/COORDINATE_SYSTEM.md）
   */
  void SetGimbalOrientation(const Eigen::Quaterniond& q) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::modules
