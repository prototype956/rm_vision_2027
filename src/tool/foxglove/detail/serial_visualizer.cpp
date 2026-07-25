/**
 * @file serial_visualizer.cpp
 * @brief 串口上行帧接收可视化助手实现
 */
#include "tool/foxglove/detail/serial_visualizer.hpp"

#include "tool/foxglove/foxglove_sink.hpp"

#include <chrono>
#include <cstdio>
#include <string>

#include <nlohmann/json.hpp>

namespace mv::tool {

// 环形缓冲区容量上限（字节）
static constexpr std::size_t RX_STREAM_MAX_SIZE = 4096U;

// ── TryParseOneFrame ──────────────────────────────────────────────────────────

SerialVisualizer::ParseStepResult SerialVisualizer::TryParseOneFrame(
    mv::protocol::UpFrame& out_frame) {
  // ── 寻找合法帧头 0xAA 0xFF ───────────────────────────────────────────────
  std::size_t header_pos = rx_stream_buffer_.size();
  for (std::size_t i = 0; i + 1 < rx_stream_buffer_.size(); ++i) {
    if (rx_stream_buffer_[i] == mv::protocol::FRAME_HEAD_0 &&
        rx_stream_buffer_[i + 1] == mv::protocol::UP_HEAD_1) {
      header_pos = i;
      break;
    }
  }

  // 整个缓冲区中没有合法帧头
  if (header_pos == rx_stream_buffer_.size()) {
    // 保留末尾 0xAA（可能是下一帧头第一字节）
    const bool KEEP_LAST =
        (!rx_stream_buffer_.empty() && rx_stream_buffer_.back() == mv::protocol::FRAME_HEAD_0);
    const std::size_t DROP = KEEP_LAST ? (rx_stream_buffer_.size() - 1U) : rx_stream_buffer_.size();
    if (DROP > 0) {
      OnParseFailure(ParseFailKind::HEADER_MISS);
      rx_stream_buffer_.erase(rx_stream_buffer_.begin(),
                              rx_stream_buffer_.begin() + static_cast<std::ptrdiff_t>(DROP));
    }
    return {.decoded = false, .stop = true};
  }

  // 帧头前有垃圾字节，丢弃后继续重新尝试
  if (header_pos > 0) {
    OnParseFailure(ParseFailKind::HEADER_MISS);
    rx_stream_buffer_.erase(rx_stream_buffer_.begin(),
                            rx_stream_buffer_.begin() + static_cast<std::ptrdiff_t>(header_pos));
    return {.decoded = false, .stop = false};
  }

  // 帧头已在 buf[0]，字节数不足一帧，等待下次 Recv
  if (rx_stream_buffer_.size() < mv::protocol::UP_FRAME_LEN) {
    return {.decoded = false, .stop = true, .short_packet = true};
  }

  // ── CRC + 格式校验 ────────────────────────────────────────────────────────
  const uint8_t* frame_data = rx_stream_buffer_.data();
  const uint16_t CRC_CALC = mv::protocol::Crc16Ccitt(frame_data, mv::protocol::UP_CRC_LEN);
  const uint16_t CRC_RECV =
      static_cast<uint16_t>(frame_data[25]) | (static_cast<uint16_t>(frame_data[26]) << 8U);
  const bool FORMAT_OK = frame_data[2] == mv::protocol::UP_PAYLOAD_LEN &&
                         frame_data[mv::protocol::UP_FRAME_LEN - 1U] == mv::protocol::FRAME_TAIL;

  if (!FORMAT_OK || CRC_CALC != CRC_RECV) {
    // 丢弃一字节后重试，避免误丢后续合法帧
    OnParseFailure(ParseFailKind::CRC_FAIL);
    rx_stream_buffer_.erase(rx_stream_buffer_.begin());
    return {.decoded = false, .stop = false, .short_packet = false};
  }

  // ── 解码成功 ──────────────────────────────────────────────────────────────
  mv::protocol::DecodeUpFrame(frame_data, &out_frame);
  rx_stream_buffer_.erase(
      rx_stream_buffer_.begin(),
      rx_stream_buffer_.begin() + static_cast<std::ptrdiff_t>(mv::protocol::UP_FRAME_LEN));
  return {.decoded = true, .stop = false};
}

// ── FeedBytes ─────────────────────────────────────────────────────────────────

SerialVisualizer::FeedResult SerialVisualizer::FeedBytes(const uint8_t* data, std::size_t len) {
  // 溢出保护：超出上限时从头部丢弃旧数据，保留最新字节
  if (rx_stream_buffer_.size() + len > RX_STREAM_MAX_SIZE) {
    const std::size_t OVERFLOW = rx_stream_buffer_.size() + len - RX_STREAM_MAX_SIZE;
    if (OVERFLOW >= rx_stream_buffer_.size()) {
      rx_stream_buffer_.clear();
    } else {
      rx_stream_buffer_.erase(rx_stream_buffer_.begin(),
                              rx_stream_buffer_.begin() + static_cast<std::ptrdiff_t>(OVERFLOW));
    }
  }

  rx_stream_buffer_.insert(rx_stream_buffer_.end(), data, data + len);

  FeedResult result{};
  bool short_packet_reported = false;

  while (!rx_stream_buffer_.empty()) {
    auto step = TryParseOneFrame(result.latest_frame);
    if (step.decoded) {
      result.parsed_any = true;
      short_packet_reported = false;  // 解码成功后重置，允许下一次半包计数
    }
    if (step.stop) {
      if (step.short_packet && !short_packet_reported) {
        OnParseFailure(ParseFailKind::SHORT_PACKET);
        short_packet_reported = true;
      }
      break;
    }
  }

  OnRxData(data, len, result.parsed_any ? &result.latest_frame : nullptr, result.parsed_any);
  return result;
}

// ── OnRxData ─────────────────────────────────────────────────────────────────

void SerialVisualizer::OnRxData(const uint8_t* raw_data, std::size_t len,
                                const mv::protocol::UpFrame* up_frame, bool parse_ok) {
  ++stats_.rx_total;
  last_rx_raw_.assign(raw_data, raw_data + len);
  last_chunk_parse_ok_ = parse_ok;
  if (parse_ok) {
    ++stats_.chunk_parse_ok;
  }

  if (parse_ok && up_frame != nullptr) {
    ++stats_.rx_parse_ok;
    last_up_frame_ = *up_frame;
    has_valid_up_frame_ = true;

    // seq 跳变检测（已初始化后才检测，忽略 255→0 的正常回绕）
    if (stats_.seq_initialized) {
      const auto EXPECTED = static_cast<uint8_t>(stats_.last_seq + 1U);
      if (up_frame->seq != EXPECTED) {
        ++stats_.seq_jump_count;
      }
    }
    stats_.last_seq = up_frame->seq;
    stats_.seq_initialized = true;

    // 时序基准
    const auto NOW = std::chrono::steady_clock::now();
    if (!stats_.timing_initialized) {
      stats_.first_ok_time = NOW;
      stats_.timing_initialized = true;
    }
    stats_.last_ok_time = NOW;
  }
}

void SerialVisualizer::OnParseFailure(ParseFailKind kind) {
  ++stats_.parse_fail_count;
  switch (kind) {
    case ParseFailKind::SHORT_PACKET:
      ++stats_.short_packet_count;
      break;
    case ParseFailKind::HEADER_MISS:
      ++stats_.header_miss_count;
      break;
    case ParseFailKind::CRC_FAIL:
      ++stats_.crc_fail_count;
      break;
  }
}

// ── Publish ──────────────────────────────────────────────────────────────────

void SerialVisualizer::Publish(FoxgloveSink& sink, int64_t ts_ns) const {
  // ── serial/rx_status ─────────────────────────────────────────────────────
  {
    nlohmann::json rx_status;
    // parse_ok 表示"已有有效上行帧"，避免被分片失败瞬时抖动覆盖。
    rx_status["parse_ok"] = has_valid_up_frame_;
    rx_status["chunk_parse_ok"] = last_chunk_parse_ok_;
    rx_status["rx_bytes"] = last_rx_raw_.size();

    if (has_valid_up_frame_) {
      constexpr double QUATERNION_SCALE = 1.0 / 10000.0;
      constexpr double ANGLE_SCALE = 1.0 / 100.0;  // yaw/pitch/vel × 100 传输，避免 int16 溢出
      constexpr double BULLET_SPEED_SCALE = 1.0 / 100.0;

      rx_status["seq"] = last_up_frame_.seq;
      rx_status["color"] = mv::protocol::NormalizeUpColor(last_up_frame_.color);
      rx_status["color_raw"] = last_up_frame_.color;
      rx_status["mode"] = static_cast<uint8_t>(last_up_frame_.mode);
      rx_status["robot_id"] = static_cast<uint8_t>(last_up_frame_.robot_id);
      rx_status["bullet_speed_mps"] = last_up_frame_.bullet_speed * BULLET_SPEED_SCALE;
      rx_status["q_w"] = last_up_frame_.q_w * QUATERNION_SCALE;
      rx_status["q_x"] = last_up_frame_.q_x * QUATERNION_SCALE;
      rx_status["q_y"] = last_up_frame_.q_y * QUATERNION_SCALE;
      rx_status["q_z"] = last_up_frame_.q_z * QUATERNION_SCALE;
      rx_status["yaw_rad"] = last_up_frame_.yaw * ANGLE_SCALE;
      rx_status["pitch_rad"] = last_up_frame_.pitch * ANGLE_SCALE;
      rx_status["yaw_vel_rps"] = last_up_frame_.yaw_vel * ANGLE_SCALE;
      rx_status["pitch_vel_rps"] = last_up_frame_.pitch_vel * ANGLE_SCALE;
    }

    if (stats_.timing_initialized) {
      const auto MS_AGO = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - stats_.last_ok_time)
                              .count();
      rx_status["last_rx_ms_ago"] = MS_AGO;
    }

    sink.PublishJson("serial/rx_status", rx_status, ts_ns);
  }

  // ── serial/rx_raw_hex ────────────────────────────────────────────────────
  if (!last_rx_raw_.empty()) {
    static constexpr std::string_view HEX_DIGITS = "0123456789ABCDEF";
    std::string hex;
    hex.reserve(last_rx_raw_.size() * 3U);
    for (std::size_t i = 0; i < last_rx_raw_.size(); ++i) {
      if (i > 0) {
        hex += ' ';
      }
      hex += HEX_DIGITS[(last_rx_raw_[i] >> 4U) & 0x0FU];
      hex += HEX_DIGITS[last_rx_raw_[i] & 0x0FU];
    }

    nlohmann::json raw_hex;
    raw_hex["len"] = last_rx_raw_.size();
    raw_hex["hex"] = hex;
    sink.PublishJson("serial/rx_raw_hex", raw_hex, ts_ns);
  }

  // ── serial/stats ─────────────────────────────────────────────────────────
  {
    nlohmann::json stats;
    stats["rx_total"] = stats_.rx_total;
    stats["chunk_parse_ok"] = stats_.chunk_parse_ok;
    stats["rx_parse_ok"] = stats_.rx_parse_ok;
    stats["parse_fail_count"] = stats_.parse_fail_count;
    stats["short_packet_count"] = stats_.short_packet_count;
    stats["header_miss_count"] = stats_.header_miss_count;
    stats["crc_fail_count"] = stats_.crc_fail_count;
    stats["seq_jump_count"] = stats_.seq_jump_count;

    const uint64_t FRAME_TOTAL = stats_.rx_parse_ok + stats_.parse_fail_count;
    const auto FRAME_PARSE_RATE = (FRAME_TOTAL > 0) ? static_cast<double>(stats_.rx_parse_ok) /
                                                          static_cast<double>(FRAME_TOTAL)
                                                    : 0.0;
    stats["parse_success_rate"] = FRAME_PARSE_RATE;

    const auto CHUNK_PARSE_RATE =
        (stats_.rx_total > 0)
            ? static_cast<double>(stats_.chunk_parse_ok) / static_cast<double>(stats_.rx_total)
            : 0.0;
    stats["chunk_success_rate"] = CHUNK_PARSE_RATE;

    // rx_hz：成功解析帧的长期平均帧率
    double rx_hz = 0.0;
    if (stats_.rx_parse_ok > 1 && stats_.timing_initialized) {
      const auto ELAPSED_S =
          std::chrono::duration<double>(stats_.last_ok_time - stats_.first_ok_time).count();
      if (ELAPSED_S > 0.0) {
        rx_hz = static_cast<double>(stats_.rx_parse_ok - 1U) / ELAPSED_S;
      }
    }
    stats["rx_hz"] = rx_hz;

    sink.PublishJson("serial/stats", stats, ts_ns);
  }
}

}  // namespace mv::tool
