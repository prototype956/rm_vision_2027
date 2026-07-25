/**
 * @file capture_node.hpp
 * @brief 采集节点（Pipeline 第一级）
 *
 * 【职责】
 *   - 持有 ICamera，以最高帧率连续 Grab()；
 *   - 每帧打时间戳、分配 frame_id；
 *   - 将 FramePacket 推入输出通道（channel_out_）。
 *
 * 【设计考量】
 *   CaptureNode 不持有 Channel 所有权（unique_ptr），
 *   而是持有 shared_ptr，原因：
 *   - Pipeline 持有 channel 的真正所有权（统一创建/销毁）；
 *   - 两侧节点（Capture + Detect）各持 shared_ptr，生命周期安全；
 *   - Stop() 时 Pipeline 先 Shutdown channel，
 *     再调用 CaptureNode::Stop()（防止 Grab 一直阻塞输出端关闭时死锁）。
 *
 * 【重试机制】
 *   Grab() 失败时（相机暂时断帧）不立刻退出，
 *   而是递增 grab_fail_count_，超过阈值（默认 30 帧）则设置 error_code_
 *   并退出循环，由 Pipeline::CheckErrors() 触发上层状态机的 ERROR 转换。
 *
 * 【使用示例】
 * @code
 *   auto cam_ch = std::make_shared<Channel<FramePacket>>(4);
 *   auto cam    = Factory<hal::ICamera>::Create("mindvision");
 *   cam->Open(config);
 *
 *   CaptureNode cap_node{std::move(cam), cam_ch};
 *   cap_node.Start();
 *   // ...
 *   cap_node.Stop();
 * @endcode
 */
#pragma once

#include "channel.hpp"
#include "hal/camera/i_camera.hpp"
#include "node.hpp"
#include "packet.hpp"

#include <memory>

namespace mv::pipeline {

class CaptureNode final : public PipelineNode {
 public:
  /**
   * @param camera      相机实例（已调用 Open()）
   * @param output_ch   输出通道（FramePacket）
   * @param max_fail    连续 Grab 失败超过此帧数触发错误
   */
  CaptureNode(std::unique_ptr<hal::ICamera> camera, std::shared_ptr<Channel<FramePacket>> output_ch,
              int max_fail = 30);

  ~CaptureNode() override = default;

  CaptureNode(const CaptureNode&) = delete;
  CaptureNode& operator=(const CaptureNode&) = delete;
  CaptureNode(CaptureNode&&) = delete;
  CaptureNode& operator=(CaptureNode&&) = delete;

 protected:
  void WorkLoop() override;
  void OnStop() override;

 private:
  std::unique_ptr<hal::ICamera> camera_;
  std::shared_ptr<Channel<FramePacket>> output_ch_;
  int max_fail_;
};

}  // namespace mv::pipeline
