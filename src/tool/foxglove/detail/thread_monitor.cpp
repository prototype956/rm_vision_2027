/**
 * @file thread_monitor.cpp
 * @brief ThreadMonitor 实现
 *
 * 将各 PipelineNode 的健康指标序列化为 JSON 推送到 pipeline/nodes topic。
 * 示例输出（Foxglove JSON 面板）：
 * {
 *   "timestamp_ns": 1741392000000000,
 *   "nodes": [
 *     {"name":"CaptureNode",  "fps":120.1, "latency_ms":2.3, "drop":0,  "alive":true},
 *     {"name":"DetectNode",   "fps":85.7,  "latency_ms":9.8, "drop":2,  "alive":true,
 * "warn":"drop>0"},
 *     {"name":"PredictNode",  "fps":85.6,  "latency_ms":0.9, "drop":0,  "alive":true},
 *     {"name":"SerialNode",   "fps":85.5,  "latency_ms":0.4, "drop":0,  "alive":true}
 *   ]
 * }
 */
#include "tool/foxglove/detail/thread_monitor.hpp"

#include "tool/foxglove/detail/utils.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace mv::tool::detail {

ThreadMonitor::ThreadMonitor(foxglove::Context ctx) : ctx_(std::move(ctx)) {}

void ThreadMonitor::EnsureChannel() {
  if (!channel_.has_value()) {
    static const std::string SCHEMA_STR =
        R"({"type":"object","properties":{"timestamp_ns":{"type":"integer"},"nodes":{"type":"array","items":{"type":"object","properties":{"name":{"type":"string"},"fps":{"type":"number"},"latency_ms":{"type":"number"},"drop":{"type":"integer"},"alive":{"type":"boolean"},"warn":{"type":"string"},"error":{"type":"string"}}}}}})";
    foxglove::Schema schema;
    schema.name = "PipelineNodes";
    schema.encoding = "jsonschema";
    schema.data = reinterpret_cast<const std::byte*>(SCHEMA_STR.data());
    schema.data_len = SCHEMA_STR.size();
    auto res = foxglove::RawChannel::create("pipeline/nodes", "json", schema, ctx_);
    if (res.has_value()) {
      channel_.emplace(std::move(res.value()));
    } else {
      spdlog::error("[ThreadMonitor] Failed to create pipeline/nodes channel: {}",
                    foxglove::strerror(res.error()));
    }
  }
}

void ThreadMonitor::Publish(const std::vector<mv::tool::FoxgloveSink::ThreadMetrics>& metrics,
                            uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);
  EnsureChannel();
  if (!channel_.has_value()) {
    return;
  }

  nlohmann::json doc;
  doc["timestamp_ns"] = ts_ns;
  nlohmann::json nodes = nlohmann::json::array();

  for (const auto& m : metrics) {
    nlohmann::json node;
    node["name"] = m.node_name;
    node["fps"] = m.fps;
    node["latency_ms"] = m.latency_ms;
    node["drop"] = m.drop_count;
    node["alive"] = m.is_alive;
    if (m.drop_count > 0) {
      node["warn"] = "drop>" + std::to_string(m.drop_count);
    }
    if (!m.error_msg.empty()) {
      node["error"] = m.error_msg;
    }
    nodes.push_back(std::move(node));
  }
  doc["nodes"] = std::move(nodes);

  std::string json_str = doc.dump();
  channel_->log(reinterpret_cast<const std::byte*>(json_str.data()), json_str.size(), ts_ns);
}

}  // namespace mv::tool::detail
