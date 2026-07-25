/**
 * @file i_predictor.hpp
 * @brief 目标跟踪与预测抽象接口 (IPredictor)
 *
 * 【职责边界】
 *   IPredictor 负责"跨帧跟踪目标并输出预判的云台角度"：
 *   - 输入：本帧 Detection 列表（含 3D 坐标）+ 当前时间戳
 *   - 输出：GimbalControl（要瞄准的位置 + 是否开火）
 *           TrackTarget（跟踪状态，调试/可视化用，可选）
 *
 *   预测器维护跟踪器状态机（detecting → tracking → temp_lost → lost），
 *   使用扩展卡尔曼滤波器（EKF）平滑轨迹并补偿飞行时间延迟。
 *
 * 【时间戳设计】
 *   使用 std::chrono::steady_clock（单调时钟），而不是系统时钟：
 *   - 不受 NTP 时间跳变影响，dt 计算稳定；
 *   - Pipeline 在 Grab() 后立即打时间戳，送入 Predict() 时已携带帧时间。
 *
 * 【实现约定】
 *   - Reset() 清除跟踪状态，丢失目标后重新开始；
 *   - Predict() 无目标时返回 GimbalControl{tracking=false}；
 *   - 预测器非线程安全，在专属线程（检测线程）顺序调用。
 */
#pragma once

#include "types.hpp"

#include <chrono>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace mv {

class IPredictor {
 public:
  // ── 生命周期 ─────────────────────────────────────────────────────────────
  IPredictor() = default;
  virtual ~IPredictor() = default;

  IPredictor(const IPredictor&) = delete;
  IPredictor& operator=(const IPredictor&) = delete;

 protected:
  IPredictor(IPredictor&&) = default;
  IPredictor& operator=(IPredictor&&) = default;

 public:
  // ── 核心接口 ─────────────────────────────────────────────────────────────

  /**
   * @brief 初始化预测器（加载 EKF 参数、优先级配置）
   * @param config  YAML 配置节点
   * @return true 成功
   */
  virtual bool Init(const YAML::Node& config) = 0;

  /**
   * @brief 用本帧检测结果更新跟踪器，返回云台控制指令
   *
   * @param detections  本帧已完成 PnP 解算的 Detection 列表
   * @param timestamp   本帧的采集时间（steady_clock，在 Grab() 后立即记录）
   * @param enemy_color 敌方颜色，用于过滤非目标颜色的检测结果
   * @return GimbalControl  tracking=true 时包含有效的 yaw/pitch；
   *                        tracking=false 时上位机应保持当前姿态
   */
  [[nodiscard]] virtual GimbalControl Predict(const std::vector<Detection>& detections,
                                              std::chrono::steady_clock::time_point timestamp,
                                              ArmorColor enemy_color) = 0;

  /**
   * @brief 注入当前云台姿态四元数（在 Predict() 之前调用）
   *
   * PredictNode 在每帧 Predict() 前从 SharedState 读取四元数并调用此方法。
   * 默认实现为空操作，SimplePredictor 无需覆写；
   * EkfPredictor 覆写后将四元数缓存为 R_gimbal2world，在内部转换坐标系。
   *
   * 为什么是默认空操作而非纯虚函数？
   *   - 不强制所有实现处理云台姿态（SimplePredictor 等简单实现不需要它）；
   *   - 接口向后兼容：将来新增需要云台姿态的实现只需覆写此方法即可。
   *
   * @param q  云台-生磁仿鸡对齐系到 IMU 绝对系的旋转
   *            （详见 docs/refractor/tf/COORDINATE_SYSTEM.md）
   */
  virtual void SetGimbalOrientation(const Eigen::Quaterniond& q) {
    (void)q;  // 默认空操作，SimplePredictor 不需要覆写
  }

  /**
   * @brief 获取当前跟踪目标的详细状态
   *
   * TrackTarget 有两类消费者：
   *   1. **Voter**（待实现）：读取 tracker_state / is_tracking / number / color，
   *      结合击打优先级和冷却时间决策是否允许开火，输出 fire 信号给 Shooter；
   *   2. **Foxglove 可视化**：读取 position / velocity / yaw_predicted 等字段，
   *      在上位机 UI 中绘制跟踪轨迹和预测落点，辅助调试。
   *
   * @note Voter 和 Shooter 模块尚未实现（TODO: Stage 3 后续 / Stage 4 Pipeline），
   *       实现时应从此接口取数据，而不是重新在 Pipeline 中传递 Detection 列表。
   *
   * @return TrackTarget 状态快照（调用方应拷贝而非持有引用，预测器可能在下一帧更新）
   */
  [[nodiscard]] virtual TrackTarget GetTrackTarget() const = 0;

  /**
   * @brief 重置跟踪器状态（目标切换、模式切换时调用）
   *
   * 调用后下一次 Predict() 将从 detecting 状态重新开始。
   */
  virtual void Reset() = 0;

  /** @return 预测器是否已完成初始化 */
  [[nodiscard]] virtual bool IsInitialized() const noexcept = 0;
};

}  // namespace mv
