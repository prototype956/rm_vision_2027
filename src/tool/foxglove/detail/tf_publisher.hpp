/**
 * @file tf_publisher.hpp
 * @brief TF 坐标系变换发布子模块
 *
 * 【设计说明】
 *
 *   将仴次坐标系变换推送到 Foxglove 标准 /tf topic（FrameTransforms schema）。
 *   Foxglove 3D 面板会自动根据 parent/child 其构建坐标系树，
 *   可直观展示 world → gimbal → camera 的相对关系。
 *
 *   推荐调用场景：
 *   - 每帧推送一次 ("world", "gimbal", T_wg)—来自预测器输出
 *   - Init() 时推送一次 ("gimbal", "camera", T_gc)—固定外参
 *
 *   频道名称固定为 "/tf"，与 Foxglove ROS 层约定一致。
 *
 * 线程安全：mtx_ 保护 channel 初始化和消息发送。
 */
#pragma once

#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <foxglove/context.hpp>
#include <foxglove/schemas.hpp>
#include <optional>

namespace mv::tool::detail {

class TfPublisher {
 public:
  /** @param ctx  foxglove 全局上下文，由 FoxgloveSink::Impl 传入并共享 */
  explicit TfPublisher(foxglove::Context ctx);

  /**
   * @brief 发布一条坐标系变换
   *
   * 每次调用发送一个 FrameTransforms 消息（仅包含单条 transform）。
   * Foxglove 会将多次调用的结果合并成坐标系树。
   *
   * @param parent   父坐标系名称（如 "world"）
   * @param child    子坐标系名称（如 "gimbal"）
   * @param T        4×4 齐次变换矩阵（child 相对于 parent 的位姿）
   * @param ts_ns    时间戳（纳秒，须已由 ResolveTs 解析）
   */
  void Publish(const std::string& parent, const std::string& child, const Eigen::Matrix4d& T,
               uint64_t ts_ns);

 private:
  /**
   * @brief 懒创建 /tf channel（首次 Publish 时调用）
   *
   * /tf 频道名称固定，全程只需一个 channel 实例。
   */
  void EnsureChannel();

  // ── 成员变量 ─────────────────────────────────────────────────────────────
  foxglove::Context ctx_;                                          ///< SDK 全局上下文
  std::optional<foxglove::schemas::FrameTransformsChannel> tf_ch_; ///< /tf channel（懒创建）
  std::mutex mtx_;                                                  ///< 保护 channel 初始化和消息发送
};

}  // namespace mv::tool::detail
