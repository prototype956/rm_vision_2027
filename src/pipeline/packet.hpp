/**
 * @file packet.hpp
 * @brief Pipeline 节点间传递的数据包类型
 *
 * 【设计原则】
 *   Pipeline 的每一级节点将采集/处理结果打包为一个"数据包"，
 *   通过 Channel<T> 传递给下一级节点。每个数据包携带：
 *   - 业务数据（帧图像、检测列表、控制指令等）
 *   - 时间戳（在 CaptureNode::Grab() 后立即记录，全程不变）
 *   - frame_id（单调递增，用于对齐调试日志和 Foxglove 可视化）
 *
 * 【数据包流动路径】
 *
 *   CaptureNode
 *       │  FramePacket
 *       ▼
 *   DetectNode  (IDetector + ISolver)
 *       │  DetectPacket
 *       ▼
 *   PredictNode (IPredictor + IVoter)
 *       │  ControlPacket
 *       ▼
 *   SerialNode  (IShooter + ISerial)
 *       │  RecvPacket（上行，反向通知）
 *       ▼
 *   [共享状态: enemy_color / mode 等]
 *
 * 【注意事项】
 *   - cv::Mat 是引用计数矩阵，拷贝数据包时只增加引用计数，无深拷贝开销；
 *   - 所有包含 cv::Mat 的数据包禁止跨线程"写"同一块 Mat，
 *     DetectNode 处理时应先 clone() 再修改（如画框调试）；
 *   - 时间戳使用 steady_clock（单调时钟），不受系统时间修改影响。
 */
#pragma once

#include "interfaces/types.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace mv::pipeline {

// ────────────────────────────────────────────────────────────────────────────
// 第一级：采集包（CaptureNode → DetectNode）
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief 相机原始帧数据包
 *
 * 由 CaptureNode 在 Grab() 成功后立即构造。
 * 时间戳打在 Grab() 返回后的第一行，尽量靠近硬件触发时刻。
 */
struct FramePacket {
  /** 原始帧（BGR, CV_8UC3）——只读引用，DetectNode 内若需修改须先 clone() */
  cv::Mat frame{};

  /** 采集时间戳（Grab() 返回后立即记录，全程不变） */
  std::chrono::steady_clock::time_point timestamp{};

  /** 单调递增帧序号（CaptureNode 内部计数，从 0 开始）*/
  uint64_t frame_id{0};
};

// ────────────────────────────────────────────────────────────────────────────
// 第二级：检测包（DetectNode → PredictNode）
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief 检测 + PnP 解算完成后的数据包
 *
 * 由 DetectNode 在调用 IDetector::Detect() + ISolver::Solve() 后构造。
 * detections 中的每个 Detection：
 *   - is_solved = true  ← 已完成 PnP 解算
 *   - xyz_in_gimbal 已填充（云台坐标系，单位 m）
 *   - yaw_angle / pitch_angle 已填充
 */
struct DetectPacket {
  /** 本帧所有检测+解算完成的装甲板列表（空表示本帧无目标） */
  std::vector<Detection> detections{};

  /**
   * @brief 透传原始帧（保留用于 Foxglove 可视化和调试）
   *
   * PredictNode 可将预测落点画在此帧上后发布到 Foxglove；
   * 正式比赛可将此字段留空（frame.empty() == true）以节省拷贝开销。
   */
  cv::Mat frame{};

  /** 透传采集时间戳（与 FramePacket 相同，不重新打）*/
  std::chrono::steady_clock::time_point timestamp{};

  /** 透传帧序号 */
  uint64_t frame_id{0};

  /** 本帧检测所用的敌方颜色（由 SerialNode 反向写入共享状态后传下来）*/
  ArmorColor enemy_color{ArmorColor::UNKNOWN};
};

// ────────────────────────────────────────────────────────────────────────────
// 第三级：控制包（PredictNode → SerialNode）
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief 预测 + Voter 确认后的云台控制数据包
 *
 * 由 PredictNode 在调用 IPredictor::Predict() + IVoter::Vote() 后构造。
 * control.fire 已由 Voter 签字：
 *   - true  = 允许开火
 *   - false = 禁止开火（无目标 / 冷却中 / 置信度不足）
 */
struct ControlPacket {
  /** 最终云台控制指令（包含 yaw/pitch/fire/tracking）*/
  GimbalControl control{};

  /** 跟踪目标状态快照（用于 Foxglove 可视化和日志）*/
  TrackTarget track_target{};

  /** 透传采集时间戳 */
  std::chrono::steady_clock::time_point timestamp{};

  /** 透传帧序号 */
  uint64_t frame_id{0};
};

// ────────────────────────────────────────────────────────────────────────────
// 反向：下位机上行数据（SerialNode → 共享状态）
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief 下位机上行反馈（由 SerialNode 解析后广播到 Pipeline 共享状态）
 *
 * 不在 Pipeline 主流中传递（主流是单向的），
 * 而是由 SerialNode 解析后写入 Pipeline::SharedState，
 * CaptureNode / DetectNode 读取 enemy_color 和 mode 来调整行为。
 *
 * 字段含义来自 RoboMaster 裁判系统协议（具体实现在 UartSerial/SerialProtocol）。
 */
struct RecvPacket {
  /** 敌方颜色（由下位机根据比赛模式设置）*/
  ArmorColor enemy_color{ArmorColor::UNKNOWN};

  /** 运行模式（对应旧代码 uart::AUTO_AIM / ENERGY_BUFF 等）*/
  uint8_t mode{0};

  /** 当前弹速（m/s，用于弹道补偿）*/
  float bullet_speed{15.0F};

  /** 接收时间戳 */
  std::chrono::steady_clock::time_point timestamp{};
};

}  // namespace mv::pipeline
