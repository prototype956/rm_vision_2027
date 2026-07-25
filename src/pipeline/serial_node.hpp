/**
 * @file serial_node.hpp
 * @brief 串口收发节点（Pipeline 第四级 + 反向上行）
 *
 * 【职责（双向）】
 *   下行（发送）：
 *     - 从 input_ch_（ControlPacket）取数据；
 *     - 调用 IShooter::Send() 将 GimbalControl 序列化并通过 ISerial 发出；
 *     - 发送失败时递增错误计数，超阈值触发 error_code_。
 *
 *   上行（接收）：
 *     - 每次发送后立即尝试 Recv（非阻塞）；
 *     - 解析 RecvPacket（敌方颜色 / 弹速 / 模式），
 *       更新 Pipeline::SharedState 中的原子变量；
 *     - 此更新无需加锁，因为：
 *         * ArmorColor 是 uint8_t，原子读写天然对齐；
 *         * 下一帧 Detect/Predict 读到新值即可，允许 1~2 帧延迟。
 *
 * 【线程模型说明】
 *   SerialNode 只有一个工作线程，串行处理"发送 → 接收"。
 *   现实场景中下行发送 ~200Hz，接收轮询延迟可以接受。
 *   如需更精确的接收时序（如 IMU 数据高速上报），可以拆分为两个线程：
 *     - send_worker_：消费 ControlPacket，发送下行帧；
 *     - recv_worker_：专门轮询 Recv，解析上行帧。
 *   目前单线程模式已够用，留作扩展点。
 *
 * 【使用示例】
 * @code
 *   SerialNode ser_node{
 *     std::move(serial), std::move(shooter),
 *     control_ch, enemy_color_ref
 *   };
 *   ser_node.Start();
 * @endcode
 */
#pragma once

#include "channel.hpp"
#include "hal/serial/i_serial.hpp"
#include "interfaces/i_shooter.hpp"
#include "node.hpp"
#include "packet.hpp"
#include "shared_state.hpp"

#include <atomic>
#include <memory>

namespace mv::pipeline {

class SerialNode final : public PipelineNode {
 public:
  /**
   * @param serial       串口实例（已 Open()）
   * @param shooter      发射器（已 Init()，负责弹道补偿 + 帧打包）
   * @param input_ch     输入通道（ControlPacket）
   * @param state        共享状态（写入 enemy_color + gimbal_quat）
   * @param max_send_fail  连续发送失败超过此次数触发 error_code_
   *
   * 使用 SharedState& 而非单独的 enemy_color：
   *   SerialNode 需要同时写入颜色和 IMU 四元数，两者封装在同一个引用中传入，
   *   避免未来扩展上行数据类型时修改构造参数列表。
   */
  SerialNode(std::unique_ptr<hal::ISerial> serial, std::unique_ptr<IShooter> shooter,
             std::shared_ptr<Channel<ControlPacket>> input_ch, SharedState& state,
             int max_send_fail = 30);

  ~SerialNode() override = default;

  SerialNode(const SerialNode&) = delete;
  SerialNode& operator=(const SerialNode&) = delete;
  SerialNode(SerialNode&&) = delete;
  SerialNode& operator=(SerialNode&&) = delete;

 protected:
  void WorkLoop() override;
  void OnStop() override;

 private:
  /** 尝试接收并解析上行帧，更新共享状态 */
  void TryRecv();

  std::unique_ptr<hal::ISerial> serial_;
  std::unique_ptr<IShooter> shooter_;
  std::shared_ptr<Channel<ControlPacket>> input_ch_;
  SharedState& state_;  ///< Pipeline 共享状态（enemy_color 和 gimbal_quat 均写入此）
  int max_send_fail_;
};

}  // namespace mv::pipeline
