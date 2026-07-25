/**
 * @file capture_node.cpp
 * @brief CaptureNode 工作线程实现
 */
#include "capture_node.hpp"

#include "core/logger.hpp"

#include <chrono>

namespace mv::pipeline {

CaptureNode::CaptureNode(std::unique_ptr<hal::ICamera> camera,
                         std::shared_ptr<Channel<FramePacket>> output_ch, int max_fail)
    : PipelineNode("CaptureNode"),
      camera_(std::move(camera)),
      output_ch_(std::move(output_ch)),
      max_fail_(max_fail) {}

void CaptureNode::OnStop() {
  // Shutdown 输出通道，防止下游 DetectNode 的 Pop 阻塞等待
  if (output_ch_) {
    output_ch_->Shutdown();
  }
}

void CaptureNode::WorkLoop() {
  MV_LOG_INFO("CaptureNode", "Worker started.");

  uint64_t current_frame_id = 0;
  int fail_count = 0;

  while (!ShouldStop()) {
    cv::Mat frame;
    if (!camera_->Grab(frame) || frame.empty()) {
      ++fail_count;
      if (fail_count >= max_fail_) {
        MV_LOG_ERROR("CaptureNode", "Grab() failed {} times consecutively. Setting error.",
                     max_fail_);
        SetError(1);
        break;
      }
      // 短暂等待后重试（防止 CPU 空转）
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
      continue;
    }

    // 立即打时间戳（尽量靠近硬件触发时刻）
    const auto TIMESTAMP = std::chrono::steady_clock::now();
    fail_count = 0;  // 恢复正常，重置计数

    FramePacket pkt{
        .frame = std::move(frame),
        .timestamp = TIMESTAMP,
        .frame_id = current_frame_id++,
    };

    const bool PUSHED = output_ch_->Push(std::move(pkt));
    if (!PUSHED) {
      // 通道已关闭（Stop() 被调用），退出循环
      break;
    }

    IncrementProcessed();
  }

  MV_LOG_INFO("CaptureNode", "Worker stopped. Processed {} frames.", ProcessedCount());
}

}  // namespace mv::pipeline
