/**
 * @file shared_state.hpp
 * @brief Pipeline 节点间共享的原子运行时状态
 *
 * 【为什么独立成一个头文件？】
 *   pipeline.hpp 包含所有节点头文件（predict_node.hpp / serial_node.hpp），
 *   而这些节点需要持有 SharedState 的引用。若将 SharedState 留在
 *   pipeline.hpp 内，则节点头文件无法 #include pipeline.hpp（循环依赖）。
 *
 *   将 SharedState 提取到此独立头文件后：
 *   - 节点头文件只需 #include "pipeline/shared_state.hpp"（轻量，无循环）；
 *   - pipeline.hpp 也 #include 此文件（统一来源，类型一致）；
 *   - SharedState 本身只依赖标准库和 Eigen，无任何业务模块依赖。
 *
 * 【字段说明】
 *   - enemy_color：SerialNode 写，DetectNode/PredictNode 读；
 *     使用 std::atomic<ArmorColor>，无锁，每帧误差 ≤1 帧可接受。
 *   - gimbal_quat： SerialNode 写（低频 ~200Hz），EkfPredictor 读（~100Hz）；
 *     Eigen::Quaterniond 非 TriviallyCopyable，改用 shared_mutex 保护：
 *     真实比赛场景下读远多于写，shared_lock 允许并发读，无 mutex 争用开销。
 */
#pragma once

#include "interfaces/types.hpp"

#include <atomic>
#include <shared_mutex>

#include <Eigen/Geometry>

namespace mv::pipeline {

/**
 * @brief 视觉流水线节点间共享的运行时状态
 *
 * 由 VisionPipeline 持有，各节点在构造时以引用方式注入，
 * 节点线程直接读写对应字段，不需要经过通道（Channel）传递。
 *
 * 使用场景：
 *   - 低延迟状态更新（串口上行颜色/IMU，需要下一帧立刻生效）；
 *   - 单向广播（SerialNode 写 → 多个读者），无需 Channel 的队列语义。
 */
struct SharedState {
  // ── enemy_color（标准原子，无锁）──────────────────────────────────────

  /**
   * @brief 敌方颜色（由 SerialNode 从上行帧解析后写入）
   *
   * DetectNode 和 PredictNode 各自用 load() 读取，允许 1~2 帧延迟。
   * 初始值 RED（上行帧接通前，以敌方红色为默认值，保证不漏检）。
   */
  std::atomic<ArmorColor> enemy_color{ArmorColor::RED};

  // ── gimbal_quat（shared_mutex，多读单写）──────────────────────────────

  /**
   * @brief 云台当前姿态四元数（由 SerialNode 从 IMU 上行数据写入）
   *
   * EkfPredictor 在每帧 Predict() 前读取此值，将 xyz_in_gimbal
   * 转换为世界坐标系后送入 EKF——这是 EKF 在世界系建模精度的关键。
   *
   * 坐标系约定（参见 docs/refractor/tf/COORDINATE_SYSTEM.md）：
   *   gimbal_quat 表示从云台系到 IMU 绝对系的旋转，即：
   *   xyz_in_world = R_gimbal2world · xyz_in_gimbal
   *   其中 R_gimbal2world 由 gimbal_quat + R_gimbal2imubody 推算（见 EkfPredictor）。
   *
   * 【占位说明（TODO: Stage 8-F）】
   *   SerialNode::TryRecv() 目前尚未解析 IMU 四元数（下位机协议待确认），
   *   初始值保持 Identity，等同于云台系 ≡ 世界系（无旋转修正），
   *   EkfPredictor 在此情况下退化为云台系直接建模，功能正确但精度略降。
   */
  mutable std::shared_mutex quat_mutex;
  Eigen::Quaterniond gimbal_quat{Eigen::Quaterniond::Identity()};

  // ── 访问器（线程安全封装）─────────────────────────────────────────────

  /** @brief 写入云台姿态（SerialNode 调用，独占锁）*/
  void SetGimbalQuat(const Eigen::Quaterniond& q) {
    std::unique_lock lock(quat_mutex);
    gimbal_quat = q.normalized();  // 归一化防止积累误差
  }

  /** @brief 读取云台姿态（EkfPredictor 调用，共享锁）*/
  [[nodiscard]] Eigen::Quaterniond GetGimbalQuat() const {
    std::shared_lock lock(quat_mutex);
    return gimbal_quat;
  }
};

}  // namespace mv::pipeline
