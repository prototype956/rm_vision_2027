/**
 * @file i_shooter.hpp
 * @brief 弹道补偿与串口编码抽象接口 (IShooter)
 *
 * 【职责边界】
 *   IShooter 负责"将云台控制指令序列化并通过串口发出"：
 *   - 输入：GimbalControl（预测器 + Voter 确认后的最终指令）
 *   - 输出：写入 ISerial（字节流）
 *
 *   IShooter 封装的内容：
 *   1. 弹道补偿（根据距离、初速度、重力修正 pitch）；
 *   2. 帧打包（帧头、数据域、CRC 校验）；
 *   3. 字节流写出（调用 ISerial::Send）。
 *
 *   IShooter 与 ISerial 分离的原因：
 *   - ISerial 只知道"发字节"，不知道协议格式；
 *   - IShooter 只知道"GimbalControl → 协议帧"，不知道平台 I/O；
 *   - 这样可以单独测试弹道补偿（不需要打开串口），
 *     也可以单独测试 ISerial（不需要实际控制指令）。
 *
 * 【调用时序（Pipeline 中 SerialNode）】
 *   SerialNode::Process(ControlPacket pkt):
 *     shooter_->Send(*serial_, pkt.control);   // 发送控制量
 *     auto recv = serial_->Recv(...);          // 接收下位机反馈
 *     // 解析反馈，更新 enemy_color / 模式 etc.
 *
 * 【接收侧设计说明】
 *   下位机上行数据（模式、颜色等）的解析责任在 SerialNode 内部，
 *   而不在 IShooter。IShooter 仅负责发送方向。
 *   原因：接收解析通常不涉及弹道数学，且每条协议的字段完全不同，
 *   放在 Node 层更便于直接更新 Pipeline 的共享状态。
 *
 * 【实现约定】
 *   - Send() 失败时返回 false 并记录日志，不抛异常；
 *   - Send() 非线程安全——在串口线程单线程调用；
 *   - Init() 加载弹道补偿参数（初速度、重力加速度）。
 */
#pragma once

#include "types.hpp"

#include <yaml-cpp/yaml.h>

namespace mv::hal {
// 前向声明，避免接口层依赖 HAL 头文件
// 调用方在 .cpp 中 #include "hal/serial/i_serial.hpp"
class ISerial;
}  // namespace mv::hal

namespace mv {

class IShooter {
 public:
  // ── 生命周期 ─────────────────────────────────────────────────────────────
  IShooter() = default;
  virtual ~IShooter() = default;

  IShooter(const IShooter&) = delete;
  IShooter& operator=(const IShooter&) = delete;

 protected:
  IShooter(IShooter&&) = default;
  IShooter& operator=(IShooter&&) = default;

 public:
  // ── 核心接口 ─────────────────────────────────────────────────────────────

  /**
   * @brief 初始化（加载弹道补偿参数）
   * @param config  YAML 配置节点
   * @return true 成功
   */
  virtual bool Init(const YAML::Node& config) = 0;

  /**
   * @brief 将云台控制指令编码为协议帧并通过串口发送
   *
   * 内部流程：
   *   1. 对 control.pitch 做弹道补偿（可选，由配置开关）；
   *   2. 将 yaw/pitch/fire 等字段打包为协议帧（帧头 + 数据 + CRC）；
   *   3. 调用 serial.Send(data, len) 写出字节流。
   *
   * @param serial   串口实例（由 SerialNode 持有，此处借用引用）
   * @param control  最终云台控制指令（fire 字段已由 Voter 签字）
   * @return true = 发送成功（字节全部写入内核缓冲区）
   */
  virtual bool Send(hal::ISerial& serial, const GimbalControl& control) = 0;

  /** @return 发射器是否已完成初始化 */
  [[nodiscard]] virtual bool IsInitialized() const noexcept = 0;
};

}  // namespace mv
