/**
 * @file types.hpp
 * @brief 跨层共享数据类型定义
 *
 * 【为什么把类型集中在这里？】
 *
 *   旧代码中数据类型散落在各模块（basic_armor::Armor_Data、
 *   predictor::Armor、uart_serial::VisionData...），
 *   导致模块间传递数据必须 include 对方的头文件，耦合成一张网。
 *
 *   集中定义后：
 *   - IDetector 输出 Detection，IPredictor 输入也是 Detection，
 *     两者只依赖 types.hpp，互不 include；
 *   - Pipeline 只需知道数据类型，不关心谁生产谁消费；
 *   - Mock 实现直接构造数据，无需真实算法。
 *
 * 【设计约定】
 *   - 只放"穿越层边界"的数据结构，内部临时类型留在实现文件；
 *   - 用 enum class 防止隐式整数转换；
 *   - 所有 3D 坐标单位：米（m），角度单位：弧度（rad）；
 *   - 时间戳类型：std::chrono::steady_clock::time_point（单调时钟，不受系统时间影响）。
 */
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <optional>

namespace mv {

// ============================================================================
// 基础枚举
// ============================================================================

/** @brief 装甲板颜色（敌方颜色由上层从配置读取后传入）*/
enum class ArmorColor : uint8_t { RED = 0, BLUE, UNKNOWN };

/** @brief 装甲板尺寸类型 */
enum class ArmorType : uint8_t { SMALL = 0, BIG };

/** @brief 装甲板编号（对应机器人 ID）*/
enum class ArmorNumber : uint8_t {
  ONE = 0,
  TWO,
  THREE,
  FOUR,
  FIVE,
  SENTRY,
  OUTPOST,
  BASE,
  UNKNOWN
};

// ============================================================================
// 检测结果（IDetector 输出）
// ============================================================================

/**
 * @brief 单个装甲板的检测结果
 *
 * 坐标系约定：
 *   - points[0..3]：图像像素坐标，顺序固定为 BL, BR, TR, TL；
 *     其中 points[0]=BL, points[1]=BR, points[2]=TR, points[3]=TL。
 *   - xyz_in_gimbal：云台坐标系下的 3D 位置（由 ISolver 填充）；
 *     未经 PnP 求解时为零向量，is_solved 为 false。
 *   - points/box 的坐标系与 Detect() 输入帧一致：
 *     若输入为 ROI 局部图，则为局部坐标；若输入为全图，则为全图坐标。
 *     坐标还原由 RoiManager::RestoreAndUpdate() 负责。
 */
struct Detection {
  // ── 2D 信息（检测器直接输出）─────────────────────────────────────────────
  ArmorColor color{ArmorColor::UNKNOWN};
  ArmorType type{ArmorType::SMALL};
  ArmorNumber number{ArmorNumber::UNKNOWN};

  /** 图像平面四个角点（input-frame 像素坐标），顺序：BL, BR, TR, TL */
  std::array<cv::Point2f, 4> points{};

  /** 检测框（用于 ROI 裁剪和 NMS） */
  cv::Rect2f box{};

  /** 置信度 [0, 1]，传统视觉固定为 1.0 */
  float confidence{1.0F};

  /** 分类器原始 class_id（传统视觉 = -1）*/
  int class_id{-1};

  // ── 3D 信息（ISolver 填充）───────────────────────────────────────────────
  bool is_solved{false};

  /** 云台坐标系下的 3D 位置（单位：m）*/
  Eigen::Vector3d xyz_in_gimbal{Eigen::Vector3d::Zero()};

  /** PnP 解算出的 yaw/pitch 偏角（单位：rad）*/
  double yaw_angle{0.0};
  double pitch_angle{0.0};

  /**
   * 检测中心到当前 input-frame 中心的像素距离，用于优先级排序。
   * 若该帧由 ROI 管理器完成坐标还原，则应在还原后按 full-frame 重新计算。
   */
  double distance_to_center{0.0};

  // ── PnP 解算质量（ISolver 填充）──────────────────────────────────────────

  /**
   * PnP 重投影角点（图像像素坐标，顺序与 points 对应）
   * 仅 is_solved=true 时有效，用于可视化 PnP 质量
   */
  std::array<cv::Point2f, 4> reprojected_points{};

  /**
   * RMS 重投影误差（像素），越小说明 PnP 解算质量越好
   * 仅 is_solved=true 时有效
   */
  double reproj_error{0.0};

  // ── 候选解兼容字段（单解策略下由 Solver 统一复位）──────────────────────────

  /** 兼容历史可视化字段；单解策略下恒为 false */
  bool has_alt_solution{false};
  /** 兼容字段：候选解重投影角点（单解策略下每帧复位） */
  std::array<cv::Point2f, 4> reprojected_points_alt{};
  /** 兼容字段：候选解在云台坐标系下的 3D 位置（单解策略下每帧复位） */
  Eigen::Vector3d xyz_in_gimbal_alt{Eigen::Vector3d::Zero()};
  /** 兼容字段：候选解 RMS 重投影误差（单解策略下每帧复位为 0） */
  double reproj_error_alt{0.0};

  // ── 辅助方法 ─────────────────────────────────────────────────────────────

  /** @return 四个角点的中心点（图像坐标） */
  [[nodiscard]] cv::Point2f Center() const noexcept {
    return {(points[0].x + points[1].x + points[2].x + points[3].x) / 4.0F,
            (points[0].y + points[1].y + points[2].y + points[3].y) / 4.0F};
  }
};

// ============================================================================
// 预测结果与云台指令（IPredictor 输出）
// ============================================================================

/**
 * @brief 云台控制指令（预测器最终输出，直接送串口）
 *
 * 设计原则：
 *   预测器只输出"要瞄准的位置"，不关心串口协议编码细节；
 *   串口模块（HAL/ISerial）负责将此结构序列化为字节流。
 */
struct GimbalControl {
  /** 目标 yaw 偏角（单位：rad，正方向：逆时针看为正） */
  double yaw{0.0};

  /** 目标 pitch 偏角（单位：rad，正方向：向上为正）*/
  double pitch{0.0};

  /** 目标深度（单位：m）*/
  double distance{0.0};

  /** 是否请求开火 */
  bool fire{false};

  /** 跟踪器是否找到目标（false 时上位机不发送控制量）*/
  bool tracking{false};

  /** 预测时使用的时间戳（单调时钟，调试用）*/
  std::chrono::steady_clock::time_point timestamp{};
};

// ============================================================================
// 跟踪目标状态（可选，供调试 / Foxglove 可视化使用）
// ============================================================================

/** @brief 跟踪目标的完整状态（比 GimbalControl 包含更多诊断信息）*/
struct TrackTarget {
  bool is_tracking{false};
  ArmorNumber number{ArmorNumber::UNKNOWN};
  ArmorColor color{ArmorColor::UNKNOWN};

  Eigen::Vector3d position{Eigen::Vector3d::Zero()};  // 旋转中心（世界坐标系，m）
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};  // 旋转中心速度（m/s）
  double yaw_predicted{0.0};                          // 预测 yaw（rad）
  double pitch_predicted{0.0};                        // 预测 pitch（rad）

  std::string tracker_state{"lost"};  // 跟踪器内部状态名

  /**
   * @brief EKF 估计的所有装甲板空间位置（仅 EkfPredictor 填充）
   *
   * 每个元素为 [x, y, z, yaw]（4 维，世界坐标系，单位 m/rad），
   * 顺序对应装甲板 id 0, 1, 2, ...（由 EkfTrackTarget::ArmorXyzaList() 产生）。
   * SimplePredictor 不填充此字段，使用前应检查 is_empty()。
   */
  std::vector<Eigen::Vector4d> armor_positions{};
};

}  // namespace mv
