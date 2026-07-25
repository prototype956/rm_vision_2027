/**
 * @file rm_protocol.hpp
 * @brief RoboMaster 上下位机串口通信帧格式规范（正式协议 v1.0）
 *
 * 【物理层约定】
 *   - 接口类型：UART（/dev/ttyUSBx 或 /dev/ttyACMx）
 *   - 推荐波特率：921600 bps（兼容 115200，由 YAML 配置）
 *   - 数据格式：8N1（8位数据位，无校验，1位停止位）
 *
 * 【帧完整性校验】
 *   CRC16-CCITT（polynomial=0x1021，初始值=0xFFFF）。
 *   计算范围：从帧头第一字节到 payload 末尾（含帧头、长度、payload，
 *   不含 CRC 字节本身和帧尾）。
 *
 * 【字节序】全部多字节字段统一 Little-Endian。
 *
 * 【角度/四元数精度】× 10000 整数传输，精度约 0.006°（即 ~0.1 mrad）。
 *
 * 【帧方向区分（第二帧头字节）】
 *   - 下行（上位机 → MCU）：0xAA 0x0F
 *   - 上行（MCU → 上位机）：0xAA 0xFF
 *
 * ============================================================================
 * 下行帧格式（上位机 → MCU），共 15 字节：
 * ============================================================================
 *
 *  偏移  长度   字段           类型      说明
 *  [0]   1     FRAME_HEAD0    u8        0xAA，帧头1（双向共用）
 *  [1]   1     DOWN_HEAD1     u8        0x0F，帧头2（下行标识）
 *  [2]   1     payload_len    u8        0x0A = 10（固定值）
 *  [3]   1     seq            u8        序列号，0~255 循环滚动
 *  [4]   1     detected       u8        0=未识别目标, 1=已识别
 *  [5]   1     shoot          u8        脉冲触发：1=本帧开火, 0=不开火
 *  [6]   2     yaw            i16 LE    预测瞄准角 yaw   × 10000 [rad]
 *  [8]   2     pitch          i16 LE    预测瞄准角 pitch × 10000 [rad]
 *  [10]  2     distance       u16 LE    目标距离 × 100 [m]（暂置 0 保留）
 *  [12]  2     crc16          u16 LE    CRC16-CCITT（覆盖 [0]~[11]）
 *  [14]  1     FRAME_TAIL     u8        0x0D，帧尾（双向共用）
 *
 * ============================================================================
 * 上行帧格式（MCU → 上位机），共 28 字节：
 * ============================================================================
 *
 *  偏移  长度   字段           类型      说明
 *  [0]   1     FRAME_HEAD0    u8        0xAA，帧头1
 *  [1]   1     UP_HEAD1       u8        0xFF，帧头2（上行标识）
 *  [2]   1     payload_len    u8        0x16 = 22（固定值）
 *  [3]   1     seq            u8        序列号，0~255 循环滚动
 *  [4]   1     color          u8        0=RED（打蓝）, 1=BLUE（打红）
 *  [5]   1     mode           u8        见 UpMode 枚举
 *  [6]   1     robot_id       u8        见 RobotId 枚举
 *  [7]   2     bullet_speed   i16 LE    弹速 × 100 [m/s]（如 1500=15.00 m/s）
 *  [9]   2     q_w            i16 LE    云台姿态四元数 w × 10000
 *  [11]  2     q_x            i16 LE    云台姿态四元数 x × 10000
 *  [13]  2     q_y            i16 LE    云台姿态四元数 y × 10000
 *  [15]  2     q_z            i16 LE    云台姿态四元数 z × 10000
 *  [17]  2     yaw            i16 LE    云台当前 yaw   × 100 [rad]（兜底冗余）
 *  [19]  2     pitch          i16 LE    云台当前 pitch × 100 [rad]（兜底冗余）
 *  [21]  2     yaw_vel        i16 LE    yaw 角速度   × 100 [rad/s]
 *  [23]  2     pitch_vel      i16 LE    pitch 角速度 × 100 [rad/s]
 *  [25]  2     crc16          u16 LE    CRC16-CCITT（覆盖 [0]~[24]）
 *  [27]  1     FRAME_TAIL     u8        0x0D，帧尾
 *
 * 【四元数坐标系约定（与 sp_vision_25/io/cboard 一致）】
 *   q 表示 IMU 绝对系（世界系）到云台本体系的旋转：
 *     xyz_world = R(q) * xyz_gimbal
 *   即与 sp_vision_25 中 imu_at() 返回的四元数含义相同。
 *   EkfPredictor 读取后直接用于 R_gimbal2world 变换。
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mv::protocol {

// ── 通用帧常量 ───────────────────────────────────────────────────────────────

inline constexpr uint8_t FRAME_HEAD_0 = 0xAAU;  ///< 帧头第一字节（双向共用）
inline constexpr uint8_t FRAME_TAIL = 0x0DU;    ///< 帧尾（双向共用）

// ── 下行帧（上位机 → MCU） ────────────────────────────────────────────────────

inline constexpr uint8_t DOWN_HEAD_1 = 0x0FU;       ///< 下行帧头第二字节
inline constexpr uint8_t DOWN_PAYLOAD_LEN = 0x0AU;  ///< 下行 payload 固定长度 = 10
inline constexpr std::size_t DOWN_FRAME_LEN = 15U;  ///< 下行总帧长（字节）

/// CRC16 覆盖范围：[0]..[11]（不含 CRC 字节本身和帧尾）
inline constexpr std::size_t DOWN_CRC_LEN = 12U;

// ── 上行帧（MCU → 上位机） ────────────────────────────────────────────────────

inline constexpr uint8_t UP_HEAD_1 = 0xFFU;       ///< 上行帧头第二字节
inline constexpr uint8_t UP_PAYLOAD_LEN = 0x16U;  ///< 上行 payload 固定长度 = 22
inline constexpr std::size_t UP_FRAME_LEN = 28U;  ///< 上行总帧长（字节）
inline constexpr uint8_t UP_COLOR_RED = 0U;       ///< 红方（协议定义）
inline constexpr uint8_t UP_COLOR_BLUE = 1U;      ///< 蓝方（协议定义）

/// CRC16 覆盖范围：[0]..[24]（不含 CRC 字节本身和帧尾）
inline constexpr std::size_t UP_CRC_LEN = 25U;

/**
 * @brief 将上行 color 字段归一到协议定义范围（0/1）
 *
 * 非法值统一按 RED(0) 处理，避免上层出现未定义分支。
 */
[[nodiscard]] inline uint8_t NormalizeUpColor(uint8_t color) noexcept {
  return (color == UP_COLOR_BLUE) ? UP_COLOR_BLUE : UP_COLOR_RED;
}

// ── 模式枚举 ─────────────────────────────────────────────────────────────────

enum class UpMode : uint8_t {
  AUTO_AIM = 0,    ///< 自动瞄准（默认）
  SMALL_BUFF = 1,  ///< 小能量机关
  BIG_BUFF = 2,    ///< 大能量机关
  IDLE = 3,        ///< 空闲（仅 IMU 上报）
};

// ── 机器人 ID 枚举 ────────────────────────────────────────────────────────────

enum class RobotId : uint8_t {
  HERO = 0,         ///< 英雄
  INFANTRY = 1,     ///< 步兵
  SENTRY = 2,       ///< 哨兵
  UAV = 3,          ///< 无人机
  ENGINEERING = 4,  ///< 工程
};

// ── 下行帧解包结构 ────────────────────────────────────────────────────────────

/**
 * @brief 下行帧解包结果（供调试 / 日志打印使用）
 *
 * RmShooter::BuildFrame() 填充此结构后序列化为字节流。
 */
struct DownFrame {
  uint8_t seq{0};
  uint8_t detected{0};   ///< 0=未识别, 1=已识别
  uint8_t shoot{0};      ///< 脉冲触发：1=本帧开火, 0=不开火
  int16_t yaw{0};        ///< 瞄准角 yaw   × 10000 [rad]，Little-Endian
  int16_t pitch{0};      ///< 瞄准角 pitch × 10000 [rad]，Little-Endian
  uint16_t distance{0};  ///< 目标距离 × 100 [m]，Little-Endian
};

// ── 上行帧解包结构 ────────────────────────────────────────────────────────────

/**
 * @brief 上行帧解包结果（SerialNode::TryRecv() 填充，供 SharedState 更新）
 */
struct UpFrame {
  uint8_t seq{0};
  uint8_t color{0};  ///< 0=RED(打蓝方), 1=BLUE(打红方)
  UpMode mode{UpMode::AUTO_AIM};
  RobotId robot_id{RobotId::INFANTRY};
  int16_t bullet_speed{0};  ///< 弹速 × 100 [m/s]
  int16_t q_w{10000};       ///< 四元数 w × 10000（单位四元数初始值）
  int16_t q_x{0};           ///< 四元数 x × 10000
  int16_t q_y{0};           ///< 四元数 y × 10000
  int16_t q_z{0};           ///< 四元数 z × 10000
  int16_t yaw{0};           ///< 云台当前 yaw   × 100 [rad]
  int16_t pitch{0};         ///< 云台当前 pitch × 100 [rad]
  int16_t yaw_vel{0};       ///< yaw 角速度   × 100 [rad/s]
  int16_t pitch_vel{0};     ///< pitch 角速度 × 100 [rad/s]
};

// ── CRC16-CCITT ───────────────────────────────────────────────────────────────

/**
 * @brief CRC16-CCITT 软件实现（polynomial=0x1021，init=0xFFFF）
 *
 * @param data  起始指针（帧头第一字节）
 * @param len   计算字节数（不含 CRC 字节本身和帧尾）
 * @return uint16_t  CRC16 校验值（Little-Endian 写入帧时低字节在前）
 *
 * @note 与 sp_vision_25 CAN 帧校验逻辑保持算法一致（均为比特级实现，
 *       无查表）。若下位机使用查表版本，只需保证初始值和多项式相同即可。
 */
[[nodiscard]] inline uint16_t Crc16Ccitt(const uint8_t* data, std::size_t len) noexcept {
  uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8U);
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1U);
      }
    }
  }
  return crc;
}

/**
 * @brief 按协议固定偏移从完整上行帧（28 字节）解包 UpFrame
 *
 * @param frame  完整上行帧起始地址（必须以 0xAA 0xFF 开头）
 * @param out    解包输出
 */
inline void DecodeUpFrame(const uint8_t* frame, UpFrame* out) {
  if (frame == nullptr || out == nullptr) {
    return;
  }

  out->seq = frame[3];
  out->color = NormalizeUpColor(frame[4]);
  out->mode = static_cast<UpMode>(frame[5]);
  out->robot_id = static_cast<RobotId>(frame[6]);
  out->bullet_speed = static_cast<int16_t>(static_cast<uint16_t>(frame[7]) |
                                           (static_cast<uint16_t>(frame[8]) << 8U));
  out->q_w = static_cast<int16_t>(static_cast<uint16_t>(frame[9]) |
                                  (static_cast<uint16_t>(frame[10]) << 8U));
  out->q_x = static_cast<int16_t>(static_cast<uint16_t>(frame[11]) |
                                  (static_cast<uint16_t>(frame[12]) << 8U));
  out->q_y = static_cast<int16_t>(static_cast<uint16_t>(frame[13]) |
                                  (static_cast<uint16_t>(frame[14]) << 8U));
  out->q_z = static_cast<int16_t>(static_cast<uint16_t>(frame[15]) |
                                  (static_cast<uint16_t>(frame[16]) << 8U));
  out->yaw = static_cast<int16_t>(static_cast<uint16_t>(frame[17]) |
                                  (static_cast<uint16_t>(frame[18]) << 8U));
  out->pitch = static_cast<int16_t>(static_cast<uint16_t>(frame[19]) |
                                    (static_cast<uint16_t>(frame[20]) << 8U));
  out->yaw_vel = static_cast<int16_t>(static_cast<uint16_t>(frame[21]) |
                                      (static_cast<uint16_t>(frame[22]) << 8U));
  out->pitch_vel = static_cast<int16_t>(static_cast<uint16_t>(frame[23]) |
                                        (static_cast<uint16_t>(frame[24]) << 8U));
}

/**
 * @brief 在接收缓冲区中搜索并解析第一帧合法上行帧
 *
 * 解析流程：帧头匹配(0xAA 0xFF) → CRC16 校验 → 解包。
 *
 * @param rx_buf    接收缓冲区
 * @param received  有效字节数
 * @param out       解析成功时输出 UpFrame
 * @return true     找到并成功解析合法上行帧
 */
[[nodiscard]] inline bool TryParseUpFrame(const uint8_t* rx_buf, std::size_t received,
                                          UpFrame* out) {
  if (rx_buf == nullptr || out == nullptr) {
    return false;
  }

  for (std::size_t i = 0; i + UP_FRAME_LEN <= received; ++i) {
    if (rx_buf[i] != FRAME_HEAD_0 || rx_buf[i + 1] != UP_HEAD_1) {
      continue;
    }

    const uint8_t* frame = rx_buf + i;
    const uint16_t CRC_CALC = Crc16Ccitt(frame, UP_CRC_LEN);
    const uint16_t CRC_RECV =
        static_cast<uint16_t>(frame[25]) | (static_cast<uint16_t>(frame[26]) << 8U);
    if (CRC_CALC != CRC_RECV) {
      continue;
    }

    DecodeUpFrame(frame, out);
    return true;
  }

  return false;
}

}  // namespace mv::protocol
