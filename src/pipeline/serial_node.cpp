/**
 * @file serial_node.cpp
 * @brief SerialNode 工作线程实现（正式协议 v1.0）
 *
 * 【上行帧格式】28 字节，详见 src/hal/serial/rm_protocol.hpp：
 *   [0]    0xAA  帧头1
 *   [1]    0xFF  帧头2（上行标识）
 *   [2]    0x16  payload 长度 = 22
 *   [3]    seq   u8   序列号 0~255 循环
 *   [4]    color u8   0=RED(打蓝), 1=BLUE(打红)
 *   [5]    mode  u8   见 UpMode 枚举
 *   [6]    rid   u8   机器人 ID
 *   [7-8]  bspd  i16 LE  弹速 × 100 [m/s]
 *   [9-10]  q_w  i16 LE  四元数 w × 10000
 *   [11-12] q_x  i16 LE  四元数 x × 10000
 *   [13-14] q_y  i16 LE  四元数 y × 10000
 *   [15-16] q_z  i16 LE  四元数 z × 10000
 *   [17-18] yaw  i16 LE  云台 yaw   × 100 [rad]（冗余兜底）
 *   [19-20] pit  i16 LE  云台 pitch × 100 [rad]（冗余兜底）
 *   [21-22] ywv  i16 LE  yaw 角速度   × 100 [rad/s]
 *   [23-24] ptv  i16 LE  pitch 角速度 × 100 [rad/s]
 *   [25-26] crc16 u16 LE  CRC16-CCITT（覆盖 [0]~[24]）
 *   [27]   0x0D  帧尾
 *
 * 【帧同步策略】
 *   采用「搜帧头 + 定长读」两阶段策略：
 *     1. 循环单字节读取，找到 0xAA 0xFF 双字节帧头；
 *     2. 再读剩余 kUpFrameLen-2 字节，凑成完整帧；
 *     3. 校验帧尾 + CRC16，通过后解析 payload。
 *   此策略在串口噪声/对齐丢失后可快速重新同步。
 */
#include "serial_node.hpp"

#include "core/logger.hpp"
#include "hal/serial/rm_protocol.hpp"

#include <array>
#include <thread>

#include <Eigen/Geometry>

namespace mv::pipeline {

static constexpr auto POP_TIMEOUT = std::chrono::milliseconds{5};

SerialNode::SerialNode(std::unique_ptr<hal::ISerial> serial, std::unique_ptr<IShooter> shooter,
                       std::shared_ptr<Channel<ControlPacket>> input_ch, SharedState& state,
                       int max_send_fail)
    : PipelineNode("SerialNode"),
      serial_(std::move(serial)),
      shooter_(std::move(shooter)),
      input_ch_(std::move(input_ch)),
      state_(state),
      max_send_fail_(max_send_fail) {}

void SerialNode::OnStop() {
  if (input_ch_) {
    input_ch_->Shutdown();
  }
}

void SerialNode::WorkLoop() {
  MV_LOG_INFO("SerialNode", "Worker started.");

  int send_fail_count = 0;

  while (!ShouldStop()) {
    ControlPacket ctrl_pkt;
    if (!input_ch_->Pop(ctrl_pkt, POP_TIMEOUT)) {
      // 超时时也尝试接收上行（保持串口接收响应性）
      TryRecv();
      continue;
    }

    // ── 下行：发送云台控制指令 ────────────────────────────────────────────
    const bool SENT = shooter_->Send(*serial_, ctrl_pkt.control);
    if (!SENT) {
      ++send_fail_count;
      MV_LOG_WARN("SerialNode", "Send failed ({}/{}).", send_fail_count, max_send_fail_);
      if (send_fail_count >= max_send_fail_) {
        MV_LOG_ERROR("SerialNode", "Serial send failed {} times. Setting error.", max_send_fail_);
        SetError(2);
        break;
      }
    } else {
      send_fail_count = 0;
    }

    // ── 上行：尝试接收下位机反馈 ─────────────────────────────────────────
    TryRecv();

    IncrementProcessed();
  }

  MV_LOG_INFO("SerialNode", "Worker stopped. Processed {} packets.", ProcessedCount());
}

void SerialNode::TryRecv() {
  if (!serial_ || !serial_->IsOpen()) {
    return;
  }

  // ── 阶段 1：搜帧头（找 0xAA 0xFF 双字节）─────────────────────────────────
  //
  // 每次调用最多扫描 UP_FRAME_LEN 字节，防止噪声帧导致 TryRecv() 长时间阻塞。
  // 搜到 0xAA 后读第二字节确认是否为 0xFF；
  // 若不是则将第二字节作为下一轮候选继续判断。

  std::array<uint8_t, protocol::UP_FRAME_LEN> buf{};
  std::size_t num_recv = 0;

  // 单字节轮询，找 0xAA
  static constexpr std::size_t MAX_SCAN = protocol::UP_FRAME_LEN;
  std::size_t scanned = 0;
  while (scanned < MAX_SCAN) {
    num_recv = 0;
    if (!serial_->Recv(buf.data(), 1, num_recv) || num_recv < 1) {
      return;  // 无数据，等下一次调用
    }
    ++scanned;
    if (buf[0] == protocol::FRAME_HEAD_0) {
      // 读第二字节确认方向标识
      num_recv = 0;
      if (!serial_->Recv(buf.data() + 1, 1, num_recv) || num_recv < 1) {
        return;
      }
      if (buf[1] == protocol::UP_HEAD_1) {
        break;  // 找到上行帧头
      }
      // 不是上行帧头，buf[1] 已消耗，继续搜
    }
  }
  if (scanned >= MAX_SCAN) {
    return;  // 本轮扫描未找到，等下次
  }

  // ── 阶段 2：读剩余字节（凑满 UP_FRAME_LEN 字节）──────────────────────────
  //
  // 已读 buf[0]=0xAA, buf[1]=0xFF，还需读 UP_FRAME_LEN-2 字节。
  num_recv = 0;
  const std::size_t REMAIN = protocol::UP_FRAME_LEN - 2U;
  if (!serial_->Recv(buf.data() + 2, REMAIN, num_recv) || num_recv < REMAIN) {
    MV_LOG_WARN("SerialNode", "Incomplete up-frame: got {} / {} remaining bytes", num_recv, REMAIN);
    return;
  }

  // ── 阶段 3：帧尾校验 ────────────────────────────────────────────────────
  if (buf[protocol::UP_FRAME_LEN - 1U] != protocol::FRAME_TAIL) {
    MV_LOG_WARN("SerialNode", "Up-frame tail mismatch: {:#02x}", buf[protocol::UP_FRAME_LEN - 1U]);
    return;
  }

  // ── 阶段 4：CRC16 校验（覆盖 [0]~[UP_CRC_LEN-1]）───────────────────────
  const uint16_t CRC_CALC = protocol::Crc16Ccitt(buf.data(), protocol::UP_CRC_LEN);
  const uint16_t CRC_RECV =
      static_cast<uint16_t>(buf[protocol::UP_CRC_LEN]) |
      static_cast<uint16_t>(static_cast<uint16_t>(buf[protocol::UP_CRC_LEN + 1U]) << 8U);
  if (CRC_CALC != CRC_RECV) {
    MV_LOG_WARN("SerialNode", "Up-frame CRC16 mismatch: calc={:#04x} recv={:#04x}", CRC_CALC,
                CRC_RECV);
    return;
  }

  // ── 阶段 5：解析 payload ─────────────────────────────────────────────────
  //
  // 以下偏移量与 rm_protocol.hpp 上行帧格式定义严格一一对应。

  // [3] seq — 可用于丢包检测（暂记录，后续可扩展统计）
  const uint8_t SEQ = buf[3];
  (void)SEQ;  // 消除"未使用"警告，后续接入丢包统计时删除此行

  // [4] color：0=RED(我方蓝→打红), 1=BLUE(我方红→打蓝)
  const uint8_t COLOR_RAW = buf[4];
  if (COLOR_RAW == 0U) {
    state_.enemy_color.store(ArmorColor::RED);
  } else if (COLOR_RAW == 1U) {
    state_.enemy_color.store(ArmorColor::BLUE);
  }
  // 其他值：保持原有颜色，防止非法字段污染状态

  // [5] mode — 保留给 VisionFSM 使用，暂不处理
  // [6] robot_id — 保留，暂不处理

  // [7-8] bullet_speed × 100 [m/s]，Little-Endian i16
  const auto BSPD_RAW =
      static_cast<int16_t>(static_cast<uint16_t>(buf[7]) | (static_cast<uint16_t>(buf[8]) << 8U));
  const double BULLET_SPEED = static_cast<double>(BSPD_RAW) / 100.0;
  (void)BULLET_SPEED;  // TODO: 接入 IShooter 弹速更新接口

  // [9-16] 四元数 (w, x, y, z) × 10000，Little-Endian i16
  //
  // 坐标系约定（与 sp_vision_25/io/cboard 一致）：
  //   q 表示"IMU 绝对世界系 → 云台本体系"的旋转，即：
  //   xyz_world = q.toRotationMatrix() * xyz_gimbal
  //   EkfPredictor 读取后直接用于 R_gimbal2world 变换。

  // 辅助 lambda：从 buf 指针偏移处读取 i16 LE（避免重复代码）
  // NOLINTNEXTLINE(readability-identifier-naming)
  auto read_i16 = [data = buf.data()](std::size_t offset) noexcept -> int16_t {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return static_cast<int16_t>(static_cast<uint16_t>(data[offset]) |
                                (static_cast<uint16_t>(data[offset + 1U]) << 8U));
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  };

  static constexpr double Q_SCALE = 1.0 / 10000.0;
  const double QUAT_W = static_cast<double>(read_i16(9)) * Q_SCALE;
  const double QUAT_X = static_cast<double>(read_i16(11)) * Q_SCALE;
  const double QUAT_Y = static_cast<double>(read_i16(13)) * Q_SCALE;
  const double QUAT_Z = static_cast<double>(read_i16(15)) * Q_SCALE;

  // 合法性检验：|q|² 应接近 1.0（× 10000 截断引入最大误差约 0.0003）
  const double QNORM2 = QUAT_W * QUAT_W + QUAT_X * QUAT_X + QUAT_Y * QUAT_Y + QUAT_Z * QUAT_Z;
  if (std::abs(QNORM2 - 1.0) > 1e-2) {
    MV_LOG_WARN("SerialNode",
                "Invalid quaternion norm²={:.4f} (w={:.4f} x={:.4f} y={:.4f} z={:.4f})", QNORM2,
                QUAT_W, QUAT_X, QUAT_Y, QUAT_Z);
    return;
  }

  state_.SetGimbalQuat(Eigen::Quaterniond(QUAT_W, QUAT_X, QUAT_Y, QUAT_Z));

  // [17-24] 欧拉角 yaw/pitch 及角速度（冗余兜底，暂不使用）
  // read_i16(17) = yaw × 100 [rad]
  // read_i16(19) = pitch × 100 [rad]
  // read_i16(21) = yaw_vel × 100 [rad/s]
  // read_i16(23) = pitch_vel × 100 [rad/s]
}

}  // namespace mv::pipeline
