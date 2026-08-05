#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <vector>

#include <optional>
#include <span>

namespace mv::tool::foxglove::armor_detector {

using SteadyClock = std::chrono::steady_clock;

/**
 * @brief 后台发布线程消费的一帧装甲调试数据。
 *
 * cv::Mat 使用引用计数共享相机图像，检测结果和性能指标在入队时复制，避免依赖
 * 检测器下一次调用后的内部状态。
 */
struct FrameItem {
  cv::Mat image;                      ///< 与相机帧共享所有权的原始图像。
  SteadyClock::time_point timestamp;  ///< 相机帧单调时钟时间戳。
  std::uint64_t sequence{0};          ///< 相机帧序号。
  std::vector<modules::ArmorDetection> detections;  ///< 当前帧装甲检测结果副本。
  modules::DetectorStats detector_stats;            ///< 当前帧检测性能指标副本。
};

/**
 * @brief 一次 Push() 的限流和覆盖结果。
 */
struct QueuePushResult {
  bool enqueued{false};      ///< 当前帧是否写入队列。
  bool rate_limited{false};  ///< 当前帧是否因发布频率限制被跳过。
  bool overwritten{false};   ///< 入队时是否覆盖了一帧尚未消费的数据。
};

/**
 * @brief 容量固定为 1 的限流最新帧队列。
 *
 * 生产者永不等待消费者；当后台编码落后时，新帧直接覆盖旧帧。Stop() 后不再接受
 * 新帧，但 WaitPop() 仍会返回停止前保留的最后一帧。
 */
class LatestFrameQueue final {
 public:
  /**
   * @brief 创建指定最大发布频率的队列。
   * @param max_fps 正数发布频率，调用方应保证配置已完成值域校验。
   */
  explicit LatestFrameQueue(double max_fps);

  /**
   * @brief 尝试限流并用当前帧更新队列。
   * @return 当前帧的入队、限流和覆盖状态。
   * @throws std::bad_alloc 复制检测结果失败。
   */
  [[nodiscard]] QueuePushResult Push(const hal::CameraFrame& frame,
                                     std::span<const modules::ArmorDetection> detections,
                                     const modules::DetectorStats& detector_stats);

  /**
   * @brief 阻塞等待下一帧；停止且队列排空后返回空值。
   */
  [[nodiscard]] std::optional<FrameItem> WaitPop() noexcept;

  /**
   * @brief 幂等停止生产者并唤醒等待线程。
   */
  void Stop() noexcept;

 private:
  const SteadyClock::duration PERIOD;            ///< max_fps 对应的帧间隔。
  const SteadyClock::duration JITTER_TOLERANCE;  ///< 防止边界抖动误伤正常帧的容差。
  std::mutex mutex_;                              ///< 保护调度状态、队列和停止标志。
  std::condition_variable condition_;             ///< 唤醒唯一后台消费者。
  std::optional<FrameItem> queued_frame_;         ///< 当前唯一待处理帧。
  SteadyClock::time_point next_publish_timestamp_{};  ///< 下一个允许入队的调度时刻。
  bool stopped_{false};                               ///< 停止后拒绝新帧但允许排空。
};

}  // namespace mv::tool::foxglove::armor_detector
