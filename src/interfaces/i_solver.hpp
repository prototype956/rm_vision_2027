/**
 * @file i_solver.hpp
 * @brief PnP 位姿解算抽象接口 (ISolver)
 *
 * 【职责边界】
 *   ISolver 负责"将图像坐标转换为 3D 坐标"：
 *   - 输入：Detection（含 4 个角点像素坐标）
 *   - 输出：填充 Detection.xyz_in_gimbal / yaw_angle / pitch_angle
 *
 *   ISolver 不持有 Detection 列表，每次调用只处理一个目标，
 *   批量处理由调用方循环完成（保持接口简单）。
 *
 * 【坐标系约定】
 *   - 相机坐标系：右 +X，下 +Y，前 +Z（OpenCV 标准）
 *   - 云台坐标系：右 +X，上 +Y，前 +Z（RoboMaster 惯例）
 *     解算器内部完成坐标系转换，输出已是云台系。
 *
 * 【实现约定】
 *   - Init() 加载相机内参（K 矩阵 + 畸变系数）；
 *   - Solve() 就地修改传入的 Detection，成功时 is_solved = true；
 *   - 失败时（如角点退化）is_solved 保持 false，不修改已有值。
 */
#pragma once

#include "types.hpp"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace mv {

class ISolver {
 public:
  // ── 生命周期 ─────────────────────────────────────────────────────────────
  ISolver() = default;
  virtual ~ISolver() = default;

  ISolver(const ISolver&) = delete;
  ISolver& operator=(const ISolver&) = delete;

 protected:
  ISolver(ISolver&&) = default;
  ISolver& operator=(ISolver&&) = default;

 public:
  // ── 核心接口 ─────────────────────────────────────────────────────────────

  /**
   * @brief 初始化解算器（加载相机内参）
   * @param config  YAML 节点，需包含 camera_matrix 和 dist_coeffs
   * @return true 成功
   */
  virtual bool Init(const YAML::Node& config) = 0;

  /**
   * @brief 对单个检测结果执行 PnP 位姿解算
   * @param detection  [in/out] 输入角点坐标，输出填充 3D 信息
   * @return true 解算成功（detection.is_solved 同时被置为 true）
   *
   * 调用方应在 Detect() 之后、Predict() 之前逐个调用此函数。
   */
  virtual bool Solve(Detection& detection) = 0;

  /**
   * @brief 注入云台姿态（由 IMU 上行四元数提供）
   * @param q 云台坐标系到世界坐标系的旋转四元数
   *
   * 默认实现为空操作，用于保持旧解算器兼容。
   * 需要世界系重投影/优化的实现可覆盖此接口。
   */
  virtual void SetGimbalOrientation(const Eigen::Quaterniond& q) {
    (void)q;
  }

  /** @return 解算器是否已完成初始化 */
  [[nodiscard]] virtual bool IsInitialized() const noexcept = 0;
};

}  // namespace mv
