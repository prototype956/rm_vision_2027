/**
 * @file gimbal_publisher.cpp
 * @brief GimbalPublisher 实现：懒创建 control/gimbal JSON channel 并序列化 GimbalControl
 */
#include "tool/foxglove/detail/gimbal_publisher.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace mv::tool::detail {

GimbalPublisher::GimbalPublisher(foxglove::Context ctx) : ctx_(std::move(ctx)) {}

void GimbalPublisher::EnsureChannel() {
  if (channel_.has_value())
    return;

  static const std::string SCHEMA_STR = R"({"type":"object","properties":)"
                                        R"({"timestamp_ns":{"type":"integer"},)"
                                        R"("yaw_rad":{"type":"number"},)"
                                        R"("pitch_rad":{"type":"number"},)"
                                        R"("distance_m":{"type":"number"},)"
                                        R"("fire":{"type":"boolean"},)"
                                        R"("tracking":{"type":"boolean"}}})";

  foxglove::Schema schema;
  schema.name = "GimbalControl";
  schema.encoding = "jsonschema";
  schema.data = reinterpret_cast<const std::byte*>(SCHEMA_STR.data());
  schema.data_len = SCHEMA_STR.size();

  auto res = foxglove::RawChannel::create("control/gimbal", "json", schema, ctx_);
  if (res.has_value()) {
    channel_.emplace(std::move(res.value()));
  } else {
    spdlog::error("[GimbalPublisher] Failed to create control/gimbal channel: {}",
                  foxglove::strerror(res.error()));
  }
}

void GimbalPublisher::Publish(const mv::GimbalControl& ctrl, uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);
  EnsureChannel();
  if (!channel_.has_value())
    return;

  nlohmann::json msg;
  msg["timestamp_ns"] = ts_ns;
  msg["yaw_rad"] = ctrl.yaw;
  msg["pitch_rad"] = ctrl.pitch;
  msg["distance_m"] = ctrl.distance;
  msg["fire"] = ctrl.fire;
  msg["tracking"] = ctrl.tracking;

  const std::string json_str = msg.dump();
  channel_->log(reinterpret_cast<const std::byte*>(json_str.data()), json_str.size(), ts_ns);
}

}  // namespace mv::tool::detail
