#pragma once

#include "modules/fire_control/fire_control.hpp"
#include "tool/foxglove/foxglove_config.hpp"
#include "tool/foxglove/runtime/foxglove_session.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include <foxglove/channel.hpp>
#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::control {

/** @brief 用 Unix 时间定位的一份融合或相机实测云台反馈历史样本。 */
struct FeedbackHistorySample {
  std::uint64_t timestamp_ns{0};  ///< 用于轨迹相对时间和历史裁剪的 Unix epoch 纳秒时间。
  hal::GimbalFeedback feedback;  ///< 该时刻对应的角度和角速度。
};

/**
 * @brief 通过有界队列接收 100 Hz 火控结果，并在后台串行写入 Foxglove 和 MCAP。
 *
 * state 与 tracking 保留控制周期频率；trajectory 和 scene 按图像最大帧率降采样。
 * 队列满时丢弃最旧样本，使实时诊断优先追赶最新控制状态。Stop() 会排空已入队数据。
 */
class ControlDebugPublisher final {
 public:
  /** @brief 按会话中已启用的实时和录制上下文创建四组控制诊断频道。 */
  ControlDebugPublisher(const Config& config, runtime::FoxgloveSession& session);
  ~ControlDebugPublisher();

  ControlDebugPublisher(const ControlDebugPublisher&) = delete;
  ControlDebugPublisher& operator=(const ControlDebugPublisher&) = delete;

  /** @brief 启动后台编码与发布线程；仅调用一次，没有活动输出时保持停止状态。 */
  void Start() noexcept;
  /** @brief 将一份不可变火控结果复制到有界队列；无订阅且未录制时直接忽略。 */
  void Publish(const modules::FireControlResult& result) noexcept;
  /** @brief 停止接收新样本，排空队列，等待工作线程退出并关闭全部频道。 */
  void Stop() noexcept;
  /** @brief 返回因队列溢出或入队异常而累计丢弃的控制样本数。 */
  [[nodiscard]] std::uint64_t DroppedSamples() const noexcept;

 private:
  struct ChannelSet;  ///< 同一 Foxglove Context 下四个控制诊断频道的 RAII 集合。
  /** @brief 持续取出队首样本，直至 Stop() 后队列完全排空。 */
  void WorkerLoop() noexcept;
  /** @brief 编码一份结果，并按实时订阅和录制状态发布各频道。 */
  void Process(const modules::FireControlResult& result) noexcept;
  /** @brief 更新并裁剪融合反馈和相机实测反馈的一秒历史窗口。 */
  void UpdateHistory(const modules::FireControlResult& result) noexcept;

  static constexpr std::size_t K_CAPACITY = 512;  ///< 待编码控制结果的最大数量。
  runtime::FoxgloveSession& session_;  ///< 拥有 Context、发布锁和错误状态的外部会话。
  std::unique_ptr<ChannelSet> live_;  ///< 实时服务器频道；未配置或失败时为空。
  std::unique_ptr<ChannelSet> recording_;  ///< MCAP 录制频道；未配置或失败时为空。
  std::uint64_t live_state_id_{0};         ///< 实时 state 频道订阅查询 ID。
  std::uint64_t live_tracking_id_{0};      ///< 实时 tracking 频道订阅查询 ID。
  std::uint64_t live_trajectory_id_{0};    ///< 实时 trajectory 频道订阅查询 ID。
  std::uint64_t live_scene_id_{0};         ///< 实时 scene 频道订阅查询 ID。
  std::mutex mutex_;                       ///< 保护生产者—消费者队列。
  std::condition_variable condition_;      ///< 新样本或停止信号通知。
  std::deque<modules::FireControlResult> queue_;  ///< 按到达顺序保存的有界样本队列。
  std::thread worker_;                            ///< 唯一编码与发布线程。
  std::atomic<bool> accepting_{false};            ///< 是否仍接受 Publish() 输入。
  std::atomic<bool> stop_called_{false};          ///< 保证 Stop() 资源回收只执行一次。
  std::atomic<std::uint64_t> dropped_{0};         ///< 累计丢弃样本计数。
  std::deque<FeedbackHistorySample> estimated_history_;      ///< 最近一秒融合反馈。
  std::deque<FeedbackHistorySample> measured_history_;       ///< 最近一秒相机实测反馈。
  std::uint64_t last_measured_sequence_{~std::uint64_t{0}};  ///< 最近纳入历史的实测帧。
  std::uint64_t last_trajectory_sequence_{~std::uint64_t{0}};  ///< 最近重型样本源帧。
  std::uint64_t last_trajectory_timestamp_ns_{0};  ///< 最近重型样本发布时间。
  std::uint64_t trajectory_period_ns_{0};          ///< trajectory/scene 最小发布间隔。
};

}  // namespace mv::tool::foxglove::control
