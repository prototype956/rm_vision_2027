#include "foxglove_publisher.hpp"

#include <chrono>
#include <mutex>
#include <unordered_map>

#include <foxglove/channel.hpp>
#include <foxglove/foxglove.hpp>  // Added for log level
#include <foxglove/schemas.hpp>
#include <foxglove/server.hpp>
#include <foxglove/server/parameter.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <variant>

using namespace std::chrono_literals;

struct FoxglovePublisher::Impl {
  foxglove::Context context;
  std::unique_ptr<foxglove::WebSocketServer> server;
  bool is_running = false;

  // 存储已发布的 Channels
  std::unordered_map<std::string, foxglove::RawChannel> channels;
  // 存储图片专用 Channels
  std::unordered_map<std::string, foxglove::schemas::RawImageChannel> image_channels;
  std::mutex channels_mutex;

  // 参数存储
  std::unordered_map<std::string, foxglove::Parameter> param_store;
  std::mutex param_mutex;

  // 参数回调
  FoxglovePublisher::ParameterCallback param_callback;

  // 构造函数
  Impl(const std::string& config_file) {
    (void)config_file;
    // 配置日志级别
    foxglove::setLogLevel(foxglove::LogLevel::Info);

    // 创建共享上下文
    context = foxglove::Context::create();

    foxglove::WebSocketServerOptions options = {};
    options.context = context;
    options.name = "MiracleVision-Robot";
    options.host = "192.168.80.128";
    options.port = 8765;
    options.capabilities = foxglove::WebSocketServerCapabilities::ClientPublish |
                           foxglove::WebSocketServerCapabilities::Parameters;
    options.supported_encodings = {"json", "protobuf"};

    // --- 参数回调函数设置 ---

    // 1. 获取参数 (Client -> Server Request)
    options.callbacks.onGetParameters =
        [this](
            uint32_t client_id, std::optional<std::string_view> request_id,
            const std::vector<std::string_view>& param_names) -> std::vector<foxglove::Parameter> {
      (void)client_id;
      (void)request_id;
      std::lock_guard<std::mutex> lock(param_mutex);
      std::vector<foxglove::Parameter> result;

      if (param_names.empty()) {
        // Return all parameters
        for (const auto& kv : param_store) {
          result.push_back(kv.second.clone());  // Must clone!
        }
      } else {
        // Return selected parameters
        for (const auto& name_view : param_names) {
          std::string name(name_view);
          auto it = param_store.find(name);
          if (it != param_store.end()) {
            result.push_back(it->second.clone());  // Must clone!
          }
        }
      }
      return result;
    };

    // 2. 设置参数 (Client -> Server Request)
    options.callbacks.onSetParameters = [this](uint32_t client_id,
                                               std::optional<std::string_view> request_id,
                                               const std::vector<foxglove::ParameterView>& params)
        -> std::vector<foxglove::Parameter> {
      (void)client_id;
      (void)request_id;

      std::vector<foxglove::Parameter> result;
      std::vector<std::string> changed_param_names;  // 记录修改的参数名

      {
        std::lock_guard<std::mutex> lock(param_mutex);

        for (const auto& param : params) {
          const std::string name(param.name());

          if (auto it = param_store.find(name); it != param_store.end()) {
            it->second = param.clone();
            result.emplace_back(param.clone());
            changed_param_names.push_back(name);
          } else {
            // 参数不存在,创建新参数
            auto new_param = param.clone();
            param_store.emplace(name, new_param.clone());
            result.emplace_back(std::move(new_param));
          }
        }
      }  // 锁在这里释放

      // 在锁外部通知应用层 (避免死锁)
      if (param_callback) {
        for (const auto& name : changed_param_names) {
          param_callback(name, nlohmann::json::object());
        }
      }

      return result;
    };

    // 3. 订阅参数更新 (Client Subscribe)
    options.callbacks.onParametersSubscribe = [](const std::vector<std::string_view>& names) {
      (void)names;  // Optional logging
    };

    options.callbacks.onParametersUnsubscribe = [](const std::vector<std::string_view>& names) {
      (void)names;  // Optional logging
    };

    // --- End Parameter Callbacks ---

    // 设置回调函数
    options.callbacks.onSubscribe = [](uint64_t channel_id,
                                       const foxglove::ClientMetadata& client) {
      spdlog::info("Client {} subscribed to channel {}", client.id, channel_id);
    };

    options.callbacks.onUnsubscribe = [](uint64_t channel_id,
                                         const foxglove::ClientMetadata& client) {
      spdlog::info("Client {} unsubscribed from channel {}", client.id, channel_id);
    };

    // 客户端连接回调 - 自动发送所有已存储的参数
    options.callbacks.onClientConnect = [this]() {
      spdlog::info("Client connected. Publishing all parameters...");

      // 重要: 先复制参数,释放锁后再发布,避免死锁
      std::vector<foxglove::Parameter> params_to_publish;
      {
        std::lock_guard<std::mutex> lock(param_mutex);
        if (!param_store.empty()) {
          for (const auto& [name, param] : param_store) {
            params_to_publish.push_back(param.clone());
          }
        }
      }  // 锁在这里释放

      // 在锁外部发布参数
      if (!params_to_publish.empty() && server) {
        server->publishParameterValues(std::move(params_to_publish));
        spdlog::info("Published {} parameters to new client", params_to_publish.size());
      }
    };

    // 创建服务器实例
    auto server_result = foxglove::WebSocketServer::create(std::move(options));
    if (!server_result.has_value()) {
      spdlog::error("Failed to create Foxglove server: {}",
                    foxglove::strerror(server_result.error()));
      throw std::runtime_error("Foxglove server creation failed");
    }
    server = std::make_unique<foxglove::WebSocketServer>(std::move(server_result.value()));
  }

  // 辅助函数：创建或获取 Channel Handle
  foxglove::RawChannel* getOrCreateChannel(const std::string& topic, const std::string& encoding,
                                           const std::string& schema_name,
                                           const std::string& schema_text,
                                           const std::string& schema_encoding = "jsonschema") {
    std::lock_guard<std::mutex> lock(channels_mutex);

    auto it = channels.find(topic);
    if (it != channels.end()) {
      return &it->second;
    }

    // 简化: 不使用 schema,避免生命周期问题
    std::optional<foxglove::Schema> schema = std::nullopt;

    // 创建新 Channel
    auto channels_result = foxglove::RawChannel::create(topic, encoding, schema, context);

    if (!channels_result.has_value()) {
      std::cerr << "Failed to create channel: " << foxglove::strerror(channels_result.error())
                << '\n';
      return nullptr;
    }

    // 存入 Map 并返回
    auto emplaced = channels.emplace(topic, std::move(channels_result.value()));
    return &emplaced.first->second;
  }
};

FoxglovePublisher::FoxglovePublisher(const std::string& config_file)
    : pImpl_(std::make_unique<Impl>(config_file)), config_file_(config_file) {}

FoxglovePublisher::~FoxglovePublisher() {
  stop();
}

void FoxglovePublisher::start() {
  if (pImpl_->server) {
    // Server 可能是自动启动的，具体取决于 SDK 实现
    // 如果有 start 方法，可以在这里调用
    // pImpl_->server->start();
    spdlog::info("Foxglove server started on port 8765");
    pImpl_->is_running = true;
  }
}

void FoxglovePublisher::stop() {
  if (pImpl_->is_running && pImpl_->server) {
    pImpl_->server->stop();
    pImpl_->is_running = false;
    spdlog::info("Foxglove server stopped");
  }
}

// 占位函数实现，后续填充
void FoxglovePublisher::startRecording(const std::string& file_path) {
  (void)file_path;
}
void FoxglovePublisher::stopRecording() {}
void FoxglovePublisher::publishImage(const cv::Mat& image, const std::string& topic) {
  if (!pImpl_->server || image.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(pImpl_->channels_mutex);

  // 查找或创建 RawImageChannel
  auto it = pImpl_->image_channels.find(topic);
  if (it == pImpl_->image_channels.end()) {
    auto channel_res = foxglove::schemas::RawImageChannel::create(topic, pImpl_->context);
    if (!channel_res.has_value()) {
      spdlog::error("Failed to create image channel '{}': {}", topic,
                    foxglove::strerror(channel_res.error()));
      return;
    }
    it = pImpl_->image_channels.emplace(topic, std::move(channel_res.value())).first;
  }

  // 构建 RawImage 消息
  foxglove::schemas::RawImage msg;
  msg.width = image.cols;
  msg.height = image.rows;
  msg.step = static_cast<uint32_t>(image.step);
  msg.encoding = "bgr8";
  msg.frame_id = "camera";

  size_t data_size = image.total() * image.elemSize();
  msg.data.resize(data_size);
  std::memcpy(msg.data.data(), image.data, data_size);

  // 设置时间戳
  auto now = std::chrono::system_clock::now();
  auto time_since_epoch = now.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch);
  auto nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch - seconds);

  foxglove::schemas::Timestamp ts;
  ts.sec = static_cast<uint32_t>(seconds.count());
  ts.nsec = static_cast<uint32_t>(nanoseconds.count());
  msg.timestamp = ts;

  it->second.log(msg);
}
void FoxglovePublisher::publishArmorDetection(const std::vector<basic_armor::Armor_Data>& armors) {
  (void)armors;
}
void FoxglovePublisher::publishTracking(const predictor::Target& target) {
  (void)target;
}
void FoxglovePublisher::publishSerialData(const nlohmann::json& serial_data) {
  if (!pImpl_->server)
    return;

  // 添加时间戳
  auto now = std::chrono::system_clock::now();
  auto timestamp =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  nlohmann::json msg = serial_data;
  msg["timestamp"] = timestamp;

  // 序列化为字符串
  std::string json_str = msg.dump();

  // 发布到 Foxglove - 简化版本,不使用复杂的 schema
  auto* channel = pImpl_->getOrCreateChannel("serial_data", "json",
                                             "",  // 不使用 schema name
                                             ""   // 不使用 schema text
  );

  if (channel) {
    channel->log(reinterpret_cast<const std::byte*>(json_str.data()), json_str.size(), timestamp);
  }
}
void FoxglovePublisher::publishTransform(const std::string& parent_frame,
                                         const std::string& child_frame,
                                         const Eigen::Matrix4d& transform, int64_t timestamp_ns) {
  (void)parent_frame;
  (void)child_frame;
  (void)transform;
  (void)timestamp_ns;
}
void FoxglovePublisher::publishPerformanceMetrics(const nlohmann::json& metrics) {
  (void)metrics;
}
void FoxglovePublisher::setParameterCallback(ParameterCallback cb) {
  pImpl_->param_callback = cb;
}

void FoxglovePublisher::updateParameters(const nlohmann::json& params) {
  if (!pImpl_->server)
    return;

  std::lock_guard<std::mutex> lock(pImpl_->param_mutex);
  std::vector<foxglove::Parameter> params_to_publish;

  // 遍历 JSON 对象
  for (auto& [key, val] : params.items()) {
    std::string name = key;
    foxglove::Parameter param(name);  // 初始化为未设置参数

    if (val.is_boolean()) {
      param = foxglove::Parameter(name, val.get<bool>());
    } else if (val.is_number_integer()) {
      param = foxglove::Parameter(name, val.get<int64_t>());
    } else if (val.is_number_float()) {
      param = foxglove::Parameter(name, val.get<double>());
    } else if (val.is_string()) {
      param = foxglove::Parameter(name, val.get<std::string>());
    } else if (val.is_array() && !val.empty()) {
      if (val[0].is_number()) {
        // 数值数组
        param = foxglove::Parameter(name, val.get<std::vector<double>>());
      } else if (val[0].is_string()) {
        // 字符串数组 -> 序列化为 JSON 字符串
        std::string json_str = val.dump();
        param = foxglove::Parameter(name, json_str);
      }
    } else if (val.is_object()) {
      // 对象类型也序列化为 JSON 字符串
      std::string json_str = val.dump();
      param = foxglove::Parameter(name, json_str);
    }

    // 更新本地存储并添加到列表
    pImpl_->param_store.insert_or_assign(name, std::move(param));
    params_to_publish.push_back(pImpl_->param_store.at(name).clone());
  }

  // 发布参数更新
  if (!params_to_publish.empty()) {
    pImpl_->server->publishParameterValues(std::move(params_to_publish));
  }
}

nlohmann::json FoxglovePublisher::getParameterValue(const std::string& name) {
  std::lock_guard<std::mutex> lock(pImpl_->param_mutex);

  auto it = pImpl_->param_store.find(name);
  if (it == pImpl_->param_store.end()) {
    return nullptr;
  }

  const auto& param = it->second;

  // 使用 SDK 提供的类型安全方法
  if (param.is<bool>()) {
    return param.get<bool>();
  } else if (param.is<int64_t>()) {
    return param.get<int64_t>();
  } else if (param.is<double>()) {
    return param.get<double>();
  } else if (param.is<std::string>()) {
    return param.get<std::string>();
  } else if (param.is<std::vector<double>>()) {
    return param.get<std::vector<double>>();
  } else if (param.is<std::vector<int64_t>>()) {
    return param.get<std::vector<int64_t>>();
  }

  return nullptr;
}
