/**
 * @file serial_visualizer.hpp
 * @brief 串口上行帧接收可视化助手（Foxglove Topic 推送）
 *
 * 【职责】
 *   接收每次 Recv() 返回的原始字节流，内部维护环形拼帧缓冲区，
 *   完成帧头定位 → CRC 校验 → UpFrame 解码，并通过 FoxgloveSink 推送
 *   三路调试 Topic：
 *     - serial/rx_status  : 最近一帧上行数据解析结果（含全部字段、物理单位）
 *     - serial/rx_raw_hex : 最近一次 Recv() 的原始字节十六进制转储
 *     - serial/stats      : 累计统计（帧率、解析成功率、失败分类计数等）
 *
 * 【Foxglove Topic 字段说明】
 *
 *   serial/rx_status（每节流帧刷新一次，parse_ok=true 后字段才有效）：
 *     parse_ok           : 自程序启动后是否已成功解析过至少一帧（不会因半包抖动变 false）
 *     chunk_parse_ok     : 本次 FeedBytes() 是否解出了至少一帧
 *     rx_bytes           : 本次 FeedBytes() 喂入的原始字节数
 *     seq                : MCU 上报的帧序号（uint8，每帧 +1，可用于检测丢帧）
 *     color              : 归一化后的颜色（0=RED / 1=BLUE，已过滤协议外值）
 *     color_raw          : MCU 原始颜色字节（未归一化，用于排查电控传值异常）
 *     mode               : 当前机器人模式（具体含义由电控定义）
 *     robot_id           : 机器人 ID（裁判系统分配，用于多机识别与哨兵颜色切换）
 *     bullet_speed_mps   : 弹速（m/s），由裁判系统实测值经 MCU 转发
 *     q_w/q_x/q_y/q_z   : 云台姿态四元数（世界系 → 云台系），乘以 1/10000 得实际值
 *     yaw_rad            : 云台偏航角（rad）
 *     pitch_rad          : 云台俯仰角（rad）
 *     yaw_vel_rps        : 偏航角速度（rad/s）
 *     pitch_vel_rps      : 俯仰角速度（rad/s）
 *     last_rx_ms_ago     : 距上一次成功解析的毫秒数（超过 120ms 即 serial_alive=false）
 *
 *   serial/stats（累计统计，长期趋势参考）：
 *     rx_total           : FeedBytes() 被调用的总次数（每次非阻塞 Recv 有数据时 +1）
 *     rx_parse_ok        : 累计成功解码的完整帧数（健康链路 ≈ rx_hz × 运行秒数）
 *     chunk_parse_ok     : 其中至少解出一帧的 FeedBytes 调用次数
 *     chunk_success_rate : chunk_parse_ok / rx_total，「每次 Recv 都能凑成完整帧」的比例
 *                          ⚠ 此值偏低属正常现象（见下方「为什么 chunk_success_rate 不高」）
 *     parse_success_rate : rx_parse_ok / (rx_parse_ok + parse_fail_count)，帧级成功率
 *                          short_packet 被计入 parse_fail，因此仍会被压低；
 *                          重点关注 crc_fail_count 是否为 0
 *     parse_fail_count   : 帧级解析失败总次数（= short + header_miss + crc_fail 之和）
 *     short_packet_count : 半包等待次数（缓冲区有帧头但字节数 < 28）——最常见，属正常
 *     header_miss_count  : 丢弃无效字节的次数（缓冲区头部无 0xAA 0xFF）
 *     crc_fail_count     : CRC 或格式校验失败次数（≠ 0 说明链路有干扰或波特率不匹配）
 *     seq_jump_count     : seq 不连续次数（衡量 MCU 侧丢帧，正常链路应为 0）
 *     rx_hz              : 成功解析帧的长期平均帧率
 *
 * 【为什么 chunk_success_rate 偏低（如 64%）】
 *   非阻塞 UART 读（VMIN=0/VTIME=0）每次 Recv 返回的字节数不定，
 *   经常出现：前一次 Recv 收了 20 字节（帧头找到），这次又收了 8 字节才凑满 28 字节。
 *   中间那次 Recv 会触发 SHORT_PACKET 并计入「失败」，导致 chunk_success_rate 下降。
 *   这并不代表链路有问题——只要 crc_fail_count = 0 且 rx_hz 稳定，链路就是健康的。
 *
 *   提高 chunk_success_rate 的思路：
 *   1. 增大 Recv 缓冲区，单次读取更多字节（不那么容易读到半包）；
 *   2. 适当提高 VTIME（阻塞等待更多字节），但会增加主循环延迟，取决于需求；
 *   3. 不依赖此指标，改关注 rx_hz / crc_fail_count / seq_jump_count。
 *
 * 【典型用法】
 * @code
 *   mv::tool::SerialVisualizer serial_viz;
 *
 *   // 主循环 — 串口接收处
 *   const auto result = serial_viz.FeedBytes(rx_buffer.data(), received);
 *   if (result.parsed_any) {
 *     // 使用 result.latest_frame 更新运行时状态
 *   }
 *
 *   // Foxglove 发布处（节流门控后）
 *   serial_viz.Publish(sink, ts_ns);
 * @endcode
 */
#pragma once

#include "hal/serial/rm_protocol.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mv::tool {

// 前向声明，避免引入 foxglove_sink.hpp 的重量级依赖
class FoxgloveSink;

/**
 * @brief 串口上行帧接收可视化助手
 *
 * 非线程安全：与主循环同线程使用，无需加锁。
 */
class SerialVisualizer {
 public:
  /**
   * @brief FeedBytes() 的返回结果
   */
  struct FeedResult {
    bool parsed_any{false};  ///< 本次喂入的字节流中至少解出了一个完整帧
    mv::protocol::UpFrame latest_frame{};  ///< 最后解码成功的帧（仅 parsed_any=true 时有效）
  };

  /**
   * @brief 喂入一批从串口 Recv() 获取的原始字节，内部完成拼帧与解析
   *
   * 每次非阻塞 Recv() 返回数据后调用，传入本次收到的字节。
   * 内部环形缓冲区负责跨 Recv 拼帧，无需调用方关心帧边界对齐。
   *
   * @param data  本次 Recv() 返回的字节缓冲区首地址
   * @param len   本次有效字节数
   * @return FeedResult  是否解出帧以及最后一帧内容
   */
  FeedResult FeedBytes(const uint8_t* data, std::size_t len);

  /**
   * @brief 将上行串口可视化数据推送到 Foxglove
   *
   * 调用前无需判断 HasClients()，FoxgloveSink 内部已有门控。
   *
   * @param sink   FoxgloveSink 引用
   * @param ts_ns  帧时间戳（纳秒）
   */
  void Publish(FoxgloveSink& sink, int64_t ts_ns) const;

 private:
  enum class ParseFailKind : uint8_t {
    SHORT_PACKET = 0,  ///< 帧头已找到但字节数不足 28，等待下次 Recv
    HEADER_MISS = 1,   ///< 缓冲区头部不是合法帧头，丢弃垃圾字节
    CRC_FAIL = 2,      ///< CRC 或格式校验失败，丢弃一字节后重试
  };

  // 由 FeedBytes() 内部调用，更新统计并缓存最近一帧原始字节
  void OnRxData(const uint8_t* raw_data, std::size_t len, const mv::protocol::UpFrame* up_frame,
                bool parse_ok);
  // 由 FeedBytes() 内部调用，按失败类别累计计数
  void OnParseFailure(ParseFailKind kind);
  // FeedBytes() 内层循环的单步结果
  struct ParseStepResult {
    bool decoded{false};       ///< 本步成功解码一帧
    bool stop{false};          ///< true = 跳出 while（等待更多数据）
    bool short_packet{false};  ///< stop=true 的原因是字节数不足一帧
  };
  // 尝试从 rx_stream_buffer_ 头部解析一帧：定位帧头 / CRC 校验 / 解码
  ParseStepResult TryParseOneFrame(mv::protocol::UpFrame& out_frame);
  // ── 环形拼帧缓冲区 ──────────────────────────────────────────────────────────
  std::vector<uint8_t> rx_stream_buffer_{};

  // ── 最近一次 Recv 原始字节（用于 serial/rx_raw_hex 推送）─────────────────────
  std::vector<uint8_t> last_rx_raw_{};

  // ── 最近一帧解析结果 ────────────────────────────────────────────────────────
  mv::protocol::UpFrame last_up_frame_{};
  bool last_chunk_parse_ok_{false};
  bool has_valid_up_frame_{false};

  // ── 累计统计 ─────────────────────────────────────────────────────────────────
  struct RxStats {
    uint64_t rx_total{0};            ///< Recv() 有数据的调用总次数（chunk 级）
    uint64_t chunk_parse_ok{0};      ///< chunk 级解析成功次数（至少解出 1 帧）
    uint64_t rx_parse_ok{0};         ///< 成功解析为 UpFrame 的次数
    uint64_t parse_fail_count{0};    ///< 帧级解析失败总次数
    uint64_t short_packet_count{0};  ///< 帧级失败：收到半包
    uint64_t header_miss_count{0};   ///< 帧级失败：未找到合法帧头
    uint64_t crc_fail_count{0};      ///< 帧级失败：CRC 校验失败
    uint64_t seq_jump_count{0};      ///< seq 不连续次数
    uint8_t last_seq{0};             ///< 上一帧序号（用于 seq jump 检测）
    bool seq_initialized{false};     ///< 是否已收到第一帧有效 seq

    // 用于计算 rx_hz 的时间基准（仅含成功解析帧）
    std::chrono::steady_clock::time_point first_ok_time{};
    std::chrono::steady_clock::time_point last_ok_time{};
    bool timing_initialized{false};
  };

  RxStats stats_{};
};

}  // namespace mv::tool
