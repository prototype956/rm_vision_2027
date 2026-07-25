/**
 * @file hud_status_publisher.cpp
 * @brief HUD 状态 JSON 同步实现
 */
#include "tool/foxglove/detail/hud_status_publisher.hpp"

#include <chrono>

#include <foxglove/channel.hpp>
#include <spdlog/spdlog.h>

namespace mv::tool::detail {

// ============================================================================
// HudStatusPublisher
// ============================================================================

HudStatusPublisher::HudStatusPublisher(foxglove::Context ctx) : ctx_(std::move(ctx)) {}

HudStatusPublisher::~HudStatusPublisher() = default;

void HudStatusPublisher::Publish(const HudStatus& status, uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);

  // 转换为 JSON
  const auto json_data = StatusToJson(status);

  // 通过 context 发送（此处直接生成 RawChannel 并发布）
  auto res = foxglove::RawChannel::create("hud/status", "json", {}, ctx_);
  if (!res.has_value()) {
    spdlog::warn("[HudStatusPublisher] Failed to create channel: {}",
                 foxglove::strerror(res.error()));
    return;
  }

  const std::string json_str = json_data.dump();
  const uint64_t timestamp_ns =
      (ts_ns > 0) ? ts_ns
                  : static_cast<uint64_t>(std::chrono::nanoseconds(
                                              std::chrono::system_clock::now().time_since_epoch())
                                              .count());

  res.value().log(reinterpret_cast<const std::byte*>(json_str.data()), json_str.size(),
                  timestamp_ns);
}

nlohmann::json HudStatusPublisher::StatusToJson(const HudStatus& status) {
  nlohmann::json json_obj;
  json_obj["timestamp_us"] = status.timestamp_us;
  json_obj["fps"] = status.fps;
  json_obj["detection_count"] = status.detection_count;
  json_obj["tracking"] = status.tracking;
  json_obj["serial_alive"] = status.serial_alive;
  json_obj["enemy_color"] = status.enemy_color;
  json_obj["target_yaw_deg"] = status.target_yaw_deg;
  json_obj["target_pitch_deg"] = status.target_pitch_deg;
  json_obj["target_distance_m"] = status.target_distance_m;
  return json_obj;
}

}  // namespace mv::tool::detail
