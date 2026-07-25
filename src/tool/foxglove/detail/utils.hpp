/**
 * @file utils.hpp
 * @brief detail 子发布器共用的辅助函数（header-only）
 *
 * 【包含内容】
 *   - 时间戳工具（NowNs / ResolveTs / ToTs）
 *   - foxglove 颜色常量（ColorRed / ColorBlue / ...）
 *   - mv 枚举转换（ArmorColorToFox / ArmorNumberToStr / ArmorLabel）
 *   - Foxglove 基础类型构造（MakePose / EigenToFoxPose / MakeArrow）
 *
 * 【设计约定】
 *   - 全部为 inline 函数，无编译单元，零链接代价。
 *   - 不依赖任何子发布器头文件，只依赖 foxglove/schemas.hpp、Eigen、
 *     interfaces/types.hpp，可被任意子模块引入而不产生循环依赖。
 *   - 仅供 detail/ 各子模块内部使用，不对外暴露（FoxgloveSink 不需要包含此头文件）。
 */
#pragma once

#include "interfaces/types.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Dense>
#include <foxglove/schemas.hpp>
#include <numbers>

namespace mv::tool::detail {

// ── 时间戳工具 ───────────────────────────────────────────────────────────────

/** 当前系统时间（纳秒，使用 system_clock，与 Foxglove 服务器时基一致） */
[[nodiscard]] inline uint64_t NowNs() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count());
}

/**
 * @brief 解析时间戳：ts_ns <= 0 时使用当前系统时间
 *
 * 所有 Publish() 的 ts_ns 参数应先经此函数规范化，
 * 确保时间戳始终为正且有意义（外部传 0 = 使用当前时间）。
 */
[[nodiscard]] inline uint64_t ResolveTs(int64_t ts_ns) {
  return ts_ns <= 0 ? NowNs() : static_cast<uint64_t>(ts_ns);
}

/**
 * @brief uint64_t 纳秒 → foxglove Timestamp（消息内嵌时使用）
 *
 * Foxglove Timestamp 拆分为 sec（秒）+ nsec（纳秒余数），
 * 此函数负责拆分以避免各子模块重复实现。
 */
[[nodiscard]] inline foxglove::schemas::Timestamp ToTs(uint64_t ns) {
  foxglove::schemas::Timestamp ts;
  ts.sec = static_cast<uint32_t>(ns / 1'000'000'000ULL);
  ts.nsec = static_cast<uint32_t>(ns % 1'000'000'000ULL);
  return ts;
}

// ── 颜色常量 ─────────────────────────────────────────────────────────────────
// 各函数带 alpha 参数(默认值为常用不透明度)，供 3D 实体与 2D 标注复用。
// RGBA 通道均归一化到 [0, 1]。

[[nodiscard]] constexpr foxglove::schemas::Color ColorRed(double a = 0.9) {
  return {1.0, 0.12, 0.12, a};
}
[[nodiscard]] constexpr foxglove::schemas::Color ColorBlue(double a = 0.9) {
  return {0.1, 0.5, 1.0, a};
}
[[nodiscard]] constexpr foxglove::schemas::Color ColorGreen(double a = 0.9) {
  return {0.1, 1.0, 0.2, a};
}
[[nodiscard]] constexpr foxglove::schemas::Color ColorYellow(double a = 0.9) {
  return {1.0, 0.85, 0.1, a};
}
[[nodiscard]] constexpr foxglove::schemas::Color ColorWhite(double a = 0.8) {
  return {1.0, 1.0, 1.0, a};
}
[[nodiscard]] constexpr foxglove::schemas::Color ColorGray(double a = 0.6) {
  return {0.5, 0.5, 0.5, a};
}

// ── mv 枚举转换 ───────────────────────────────────────────────────────────────

/**
 * @brief mv::ArmorColor → foxglove RGBA 颜色（RED=红 / BLUE=蓝 / 其他=白）
 *
 * 供 2D 标注和 3D 立方体着色，保持与 Foxglove 面板视觉一致。
 */
[[nodiscard]] inline foxglove::schemas::Color ArmorColorToFox(mv::ArmorColor c) {
  switch (c) {
    case mv::ArmorColor::RED:
      return ColorRed();
    case mv::ArmorColor::BLUE:
      return ColorBlue();
    default:
      return ColorWhite();
  }
}

/**
 * @brief mv::ArmorNumber → 单字符/字母字符串
 *
 * 用于标签文字构造，如 "1"、"S"（哨兵）、"O"（前哨站）、"B"（基地）。
 */
[[nodiscard]] inline std::string ArmorNumberToStr(mv::ArmorNumber n) {
  switch (n) {
    case mv::ArmorNumber::ONE:
      return "1";
    case mv::ArmorNumber::TWO:
      return "2";
    case mv::ArmorNumber::THREE:
      return "3";
    case mv::ArmorNumber::FOUR:
      return "4";
    case mv::ArmorNumber::FIVE:
      return "5";
    case mv::ArmorNumber::SENTRY:
      return "S";
    case mv::ArmorNumber::OUTPOST:
      return "O";
    case mv::ArmorNumber::BASE:
      return "B";
    default:
      return "?";
  }
}

/**
 * @brief 生成装甲板显示标签字符串
 *
 * 格式："R-1 120cm"（颜色前缀 + 数字 + 距离，仅 is_solved=true 时追加距离）。
 * 用于 ImageAnnotations TextAnnotation 和 SceneUpdate TextPrimitive 的 text 字段。
 */
[[nodiscard]] inline std::string ArmorLabel(const mv::Detection& d) {
  std::string prefix;
  switch (d.color) {
    case mv::ArmorColor::RED:
      prefix = "R-";
      break;
    case mv::ArmorColor::BLUE:
      prefix = "B-";
      break;
    default:
      prefix = "?-";
      break;
  }
  std::string label = prefix + ArmorNumberToStr(d.number);
  if (d.is_solved) {
    // 追加深度（cm）
    int depth_cm = static_cast<int>(d.xyz_in_gimbal.norm() * 100.0);
    label += " " + std::to_string(depth_cm) + "cm";
  }
  return label;
}

// ── Foxglove 基础类型构造 ─────────────────────────────────────────────────────

/**
 * @brief 构造 Pose（position + orientation）
 *
 * Foxglove 很多 Primitive 都需要 Pose，此函数消除重复代码。
 * 参数顺序与 quaternion 惯例（x, y, z, w）一致。
 */
[[nodiscard]] inline foxglove::schemas::Pose MakePose(double px, double py, double pz, double qx,
                                                      double qy, double qz, double qw) {
  foxglove::schemas::Pose pose;
  pose.position = foxglove::schemas::Vector3{px, py, pz};
  pose.orientation = foxglove::schemas::Quaternion{qx, qy, qz, qw};
  return pose;
}

/**
 * @brief Eigen Matrix4d → foxglove Pose
 *
 * 从 4×4 齐次变换矩阵提取平移向量和旋转四元数，
 * 用于 TfPublisher 等无需逐字段拆解的场景。
 */
[[nodiscard]] inline foxglove::schemas::Pose EigenToFoxPose(const Eigen::Matrix4d& T) {
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();
  return MakePose(T(0, 3), T(1, 3), T(2, 3), q.x(), q.y(), q.z(), q.w());
}

/**
 * @brief 构造指向指定方向的 ArrowPrimitive（尾部在 origin，指向 dir）
 * @param origin    箭头尾部位置（云台坐标系，单位 m）
 * @param dir       方向（单位向量）
 * @param color     颜色
 * @param length    轴长（m），默认 0.08m
 */
[[nodiscard]] inline foxglove::schemas::ArrowPrimitive MakeArrow(const Eigen::Vector3d& origin,
                                                                 const Eigen::Vector3d& dir,
                                                                 foxglove::schemas::Color color,
                                                                 double length = 0.08) {
  // 求将 +X 旋转到 dir 的四元数
  Eigen::Vector3d x_hat = Eigen::Vector3d::UnitX();
  Eigen::Vector3d axis = x_hat.cross(dir);
  double dot = x_hat.dot(dir);
  Eigen::Quaterniond q;

  if (axis.norm() < 1e-6) {
    // dir ≈ ±X
    if (dot > 0.0) {
      q = Eigen::Quaterniond::Identity();
    } else {
      q = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0);  // 180° around Z
    }
  } else {
    double angle = std::acos(std::clamp(dot, -1.0, 1.0));
    q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis.normalized()));
  }
  q.normalize();

  foxglove::schemas::ArrowPrimitive arrow;
  arrow.pose = MakePose(origin.x(), origin.y(), origin.z(), q.x(), q.y(), q.z(), q.w());
  arrow.shaft_length = length * 0.8;
  arrow.shaft_diameter = length * 0.06;
  arrow.head_length = length * 0.2;
  arrow.head_diameter = length * 0.12;
  arrow.color = color;
  return arrow;
}

}  // namespace mv::tool::detail
