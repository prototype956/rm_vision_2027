/**
 * @file thread_monitor.hpp
 * @brief Pipeline 线程健康指标发布子模块
 *
 * 【设计说明】
 *
 *   将各 PipelineNode 的运行状态序列化为 JSON 并推送到
 *   pipeline/nodes topic，在 Foxglove 原始 JSON 面板中可实时查看。
 *
 *   JSON Schema （PipelineNodes）定义了以下字段：
 *   - name        (string) ：节点名称，如 "CaptureNode"
 *   - fps         (number) ：当前处理帧率
 *   - latency_ms  (number) ：本节点平均处理延迟（ms）
 *   - drop        (integer)：累计丢帧数
 *   - alive       (boolean)：进程健康否
 *   - warn        (string) ：选，丢帧时填充 "drop>N"
 *   - error       (string) ：选，节点最近错误信息
 *
 *   建议由 VisionPipeline 每 100ms 汇聚同一忽快照召用一次。
 *
 * 线程安全：mtx_ 保护 channel 初始化和消息发送。
 */
#pragma once

#include "tool/foxglove/foxglove_sink.hpp"  // FoxgloveSink::ThreadMetrics

#include <mutex>
#include <vector>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <optional>

namespace mv::tool::detail {

class ThreadMonitor {
 public:
  /** @param ctx  foxglove 全局上下文，由 FoxgloveSink::Impl 传入并共享 */
  explicit ThreadMonitor(foxglove::Context ctx);

  /**
   * @brief 发布各节点健康指标快照
   *
   * 蓝色 JSON 模式:
   * { "timestamp_ns": ..., "nodes": [{"name":..., "fps":..., ...}] }
   *
   * drop_count > 0 时自动在对应节点下添加 "warn":"drop>N" 字段。
   *
   * @param metrics  节点指标列表（由 VisionPipeline 汇聚）
   * @param ts_ns    时间戳（纳秒，须已由 ResolveTs 解析）
   */
  void Publish(const std::vector<mv::tool::FoxgloveSink::ThreadMetrics>& metrics, uint64_t ts_ns);

 private:
  /**
   * @brief 懒创建 pipeline/nodes channel（首次 Publish 时调用）
   *
   * 内嵌 JSON Schema 符号（PipelineNodes）允许 Foxglove
   * 在 JSON 面板中正确显示各字段类型。
   */
  void EnsureChannel();

  // ── 成员变量 ─────────────────────────────────────────────────────────────
  foxglove::Context ctx_;                    ///< SDK 全局上下文
  std::optional<foxglove::RawChannel> channel_; ///< pipeline/nodes JSON channel（懒创建）
  std::mutex mtx_;                            ///< 保护 channel 初始化和消息发送
};

}  // namespace mv::tool::detail
