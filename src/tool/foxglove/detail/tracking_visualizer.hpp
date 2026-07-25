/**
 * @file tracking_visualizer.hpp
 * @brief EKF 预测跟踪 3D 可视化发布子模块
 *
 * 发布三个 SceneUpdate 话题：
 *   tracking/armor_positions  — 所有装甲板预测位置（球体 + 序号文字）
 *   tracking/rotation_center  — 旋转中心（蓝球）+ 速度向量（绿箭头）
 *   tracking/aim_point        — 弹道求解最终瞄准点（绿球）
 *
 * 线程安全：mtx_ 保护所有 channel 的初始化和消息发送。
 */
#pragma once

#include "interfaces/types.hpp"

#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <foxglove/context.hpp>
#include <foxglove/schemas.hpp>
#include <optional>

namespace mv::tool::detail {

class TrackingVisualizer {
 public:
  /** @param ctx  foxglove 全局上下文，由 FoxgloveSink::Impl 传入并共享 */
  explicit TrackingVisualizer(foxglove::Context ctx);

  /**
   * @brief 发布跟踪 3D 可视化
   * @param target    EKF 输出的跟踪目标状态
   * @param aim_xyz   弹道求解后最终瞄准点（云台坐标系）
   * @param frame_id  坐标系名称（通常为 "world" 或 "gimbal"）
   * @param ts_ns     时间戳（纳秒，须已由 ResolveTs 解析）
   */
  void Publish(const mv::TrackTarget& target, const Eigen::Vector3d& aim_xyz,
               const std::string& frame_id, uint64_t ts_ns);

 private:
  /** @brief 懒创建三个 SceneUpdate channels（首次 Publish 时调用）*/
  void EnsureChannels();

  foxglove::Context ctx_;  ///< SDK 全局上下文

  std::optional<foxglove::schemas::SceneUpdateChannel> armor_pos_ch_;  ///< tracking/armor_positions
  std::optional<foxglove::schemas::SceneUpdateChannel>
      rot_center_ch_;                                                  ///< tracking/rotation_center
  std::optional<foxglove::schemas::SceneUpdateChannel> aim_point_ch_;  ///< tracking/aim_point

  std::mutex mtx_;  ///< 保护 channel 初始化和消息发送
};

}  // namespace mv::tool::detail
