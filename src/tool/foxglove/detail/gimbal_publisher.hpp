/**
 * @file gimbal_publisher.hpp
 * @brief 云台控制指令 Foxglove 发布子模块
 *
 * 发布话题：control/gimbal（JSON RawChannel）
 *
 * Schema（GimbalControl）：
 *   timestamp_ns  integer  — 时间戳（ns）
 *   yaw_rad       number   — 云台 yaw 角 (rad)
 *   pitch_rad     number   — 云台 pitch 角 (rad)
 *   distance_m    number   — 距离 (m)
 *   fire          boolean  — 开火指令
 *   tracking      boolean  — 是否在跟踪目标
 *
 * 线程安全：mtx_ 保护 channel 初始化和消息发送。
 */
#pragma once

#include "interfaces/types.hpp"

#include <mutex>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <optional>

namespace mv::tool::detail {

class GimbalPublisher {
 public:
  /** @param ctx  foxglove 全局上下文，由 FoxgloveSink::Impl 传入并共享 */
  explicit GimbalPublisher(foxglove::Context ctx);

  /**
   * @brief 发布云台控制指令
   * @param ctrl   GimbalControl 数据
   * @param ts_ns  时间戳（纳秒，须已由 ResolveTs 解析）
   */
  void Publish(const mv::GimbalControl& ctrl, uint64_t ts_ns);

 private:
  /** @brief 懒创建 control/gimbal channel（首次 Publish 时调用）*/
  void EnsureChannel();

  foxglove::Context ctx_;                        ///< SDK 全局上下文
  std::optional<foxglove::RawChannel> channel_;  ///< JSON RawChannel（懒创建）
  std::mutex mtx_;                               ///< 保护 channel 初始化和消息发送
};

}  // namespace mv::tool::detail
