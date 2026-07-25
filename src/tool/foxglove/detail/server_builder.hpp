/**
 * @file server_builder.hpp
 * @brief WebSocket Server 选项构建辅助（仅供 foxglove_sink.cpp 内部使用）
 *
 * 将 Impl 构造函数中长达 ~120 行的 WebSocketServerOptions 回调组装逻辑
 * 提取为独立函数，降低 foxglove_sink.cpp 阅读复杂度。
 *
 * ⚠️ 设计约束：本头文件只应从 foxglove_sink.cpp 包含，且必须在
 *    FoxgloveSink::Impl 定义之后（以便 inline 函数体使用 Impl 成员类型）。
 *    外部 .hpp 不应包含此文件。
 */
#pragma once

#include "tool/foxglove/foxglove_sink.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/server.hpp>
#include <foxglove/server/parameter.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace mv::tool::detail {

/**
 * @brief 所有 Server 回调需要访问的 Impl 状态引用集合
 *
 * 不直接传 Impl& 是为了让 server_builder.hpp 对 Impl 的具体定义无感知，
 * 避免头文件间的循环/前向声明问题。
 */
struct ServerCallbackArgs {
  std::atomic<int>& client_count;
  std::unordered_map<std::string, foxglove::Parameter>& param_store;
  std::mutex& param_mutex;
  FoxgloveSink::ParameterCallback& param_callback;
  /// server 在 BuildServerOptions() 调用时尚未创建；
  /// lambda 按引用捕获，运行时（onClientConnect 触发时）才实际访问——此时 server 已赋值。
  std::unique_ptr<foxglove::WebSocketServer>& server;
};

/**
 * @brief 构建 WebSocketServerOptions 并注册所有回调
 *
 * @param cfg   FoxgloveSink 配置（name / host / port）
 * @param ctx   Foxglove 全局上下文（已由 Impl 构造）
 * @param args  Impl 成员引用集合
 * @return      已填充所有回调的 WebSocketServerOptions
 */
inline foxglove::WebSocketServerOptions BuildServerOptions(const FoxgloveSink::Config& cfg,
                                                           const foxglove::Context& ctx,
                                                           ServerCallbackArgs args) {
  foxglove::WebSocketServerOptions options;
  options.context = ctx;
  options.name = cfg.name;
  options.host = cfg.host;
  options.port = cfg.port;
  options.capabilities = foxglove::WebSocketServerCapabilities::ClientPublish |
                         foxglove::WebSocketServerCapabilities::Parameters;
  options.supported_encodings = {"json", "protobuf"};

  // 1. 获取参数（Client → Server）
  options.callbacks.onGetParameters =
      [&pstore = args.param_store, &pmtx = args.param_mutex](
          uint32_t /*client_id*/, std::optional<std::string_view> /*req_id*/,
          const std::vector<std::string_view>& names) -> std::vector<foxglove::Parameter> {
    std::lock_guard<std::mutex> lock(pmtx);
    std::vector<foxglove::Parameter> result;
    if (names.empty()) {
      for (const auto& [k, v] : pstore) {
        result.push_back(v.clone());
      }
    } else {
      for (const auto& sv : names) {
        if (auto it = pstore.find(std::string(sv)); it != pstore.end()) {
          result.push_back(it->second.clone());
        }
      }
    }
    return result;
  };

  // 2. 设置参数（Client → Server）
  // 必须先释放 param_mutex 再通知 param_callback：
  // 原因：param_callback 可能调用 UpdateParameters() 再次加 param_mutex，
  // 持锁通知会导致递归死锁。
  options.callbacks.onSetParameters =
      [&pstore = args.param_store, &pmtx = args.param_mutex, &pcb = args.param_callback](
          uint32_t /*client_id*/, std::optional<std::string_view> /*req_id*/,
          const std::vector<foxglove::ParameterView>& params) -> std::vector<foxglove::Parameter> {
    std::vector<foxglove::Parameter> result;
    std::vector<std::string> changed;
    {
      std::lock_guard<std::mutex> lock(pmtx);
      for (const auto& p : params) {
        std::string name(p.name());
        auto new_p = p.clone();
        pstore.insert_or_assign(name, new_p.clone());
        result.emplace_back(std::move(new_p));
        changed.push_back(name);
      }
    }
    // 锁外通知，防止死锁
    if (pcb) {
      for (const auto& name : changed) {
        pcb(name, nlohmann::json::object());
      }
    }
    return result;
  };

  // 3. 参数订阅/取消订阅（占位）
  options.callbacks.onParametersSubscribe = [](const std::vector<std::string_view>&) {};
  options.callbacks.onParametersUnsubscribe = [](const std::vector<std::string_view>&) {};

  // 4. Channel 订阅/取消订阅日志
  options.callbacks.onSubscribe = [](uint64_t ch_id, const foxglove::ClientMetadata& c) {
    spdlog::debug("[FoxgloveSink] Client {} subscribed to channel {}", c.id, ch_id);
  };
  options.callbacks.onUnsubscribe = [](uint64_t ch_id, const foxglove::ClientMetadata& c) {
    spdlog::debug("[FoxgloveSink] Client {} unsubscribed from channel {}", c.id, ch_id);
  };

  // 5a. 客户端连接：计数 +1，推送已存储参数快照
  options.callbacks.onClientConnect = [&cnt = args.client_count, &pstore = args.param_store,
                                       &pmtx = args.param_mutex, &srv = args.server]() {
    const int n = cnt.fetch_add(1, std::memory_order_relaxed) + 1;
    spdlog::info("[FoxgloveSink] Client connected (total: {}). Publishing parameter snapshot...",
                 n);
    std::vector<foxglove::Parameter> snapshot;
    {
      std::lock_guard<std::mutex> lock(pmtx);
      for (const auto& [k, v] : pstore) {
        snapshot.push_back(v.clone());
      }
    }
    if (!snapshot.empty() && srv) {
      srv->publishParameterValues(std::move(snapshot));
    }
  };

  // 5b. 客户端断开：计数 -1
  options.callbacks.onClientDisconnect = [&cnt = args.client_count]() {
    const int n = cnt.fetch_sub(1, std::memory_order_relaxed) - 1;
    spdlog::info("[FoxgloveSink] Client disconnected (remaining: {})", n);
  };

  return options;
}

}  // namespace mv::tool::detail
