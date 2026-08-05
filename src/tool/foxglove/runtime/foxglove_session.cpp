#include "tool/foxglove/runtime/foxglove_session.hpp"

#include "core/logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <foxglove/mcap.hpp>
#include <foxglove/server.hpp>
#include <unistd.h>

namespace mv::tool::foxglove::runtime {
namespace {

std::string RecordingFileName() {
  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm local{};
  localtime_r(&now, &local);
  std::ostringstream stream;
  stream << "miraclevision_" << std::put_time(&local, "%Y%m%d_%H%M%S") << '_' << getpid()
         << ".mcap";
  return stream.str();
}

void SafeDecrement(std::atomic<std::uint64_t>& value) noexcept {
  auto current = value.load(std::memory_order_relaxed);
  while (current > 0 &&
         !value.compare_exchange_weak(current, current - 1, std::memory_order_relaxed)) {
  }
}

}  // namespace

struct FoxgloveSession::Impl {
  explicit Impl(const Config& input_config) : config(input_config) {
    if (!config.enabled) {
      return;
    }
    // 此处只创建 Context，业务组件会在 Session::Start() 前向其中注册频道。
    live_context = ::foxglove::Context::create();
    live_configured.store(true, std::memory_order_relaxed);
    if (config.recording.enabled) {
      recording_context = ::foxglove::Context::create();
      recording_path = config.recording.output_dir / RecordingFileName();
      recording_configured.store(true, std::memory_order_relaxed);
    }
  }

  void ReportError(std::string_view operation, ::foxglove::FoxgloveError error,
                   std::atomic<std::uint64_t>& counter) noexcept {
    const auto count = counter.fetch_add(1, std::memory_order_relaxed) + 1;
    // 持续故障时限制日志频率，避免调试链路反向拖慢视觉主循环。
    if (count == 1 || count % 100 == 0) {
      MV_LOG_ERROR("Foxglove", "{} error #{}: {}", operation, count, ::foxglove::strerror(error));
    }
  }

  void StartLive() {
    ::foxglove::WebSocketServerOptions options;
    options.context = live_context;
    options.name = "MiracleVision";
    options.host = config.server.host;
    options.port = config.server.port;
    options.callbacks.onClientConnect = [this] {
      current_clients.fetch_add(1, std::memory_order_relaxed);
      client_connects.fetch_add(1, std::memory_order_relaxed);
    };
    options.callbacks.onClientDisconnect = [this] {
      SafeDecrement(current_clients);
      client_disconnects.fetch_add(1, std::memory_order_relaxed);
    };
    options.callbacks.onSubscribe = [this](std::uint64_t channel_id,
                                           const ::foxglove::ClientMetadata&) {
      subscriptions.Subscribe(channel_id);
    };
    options.callbacks.onUnsubscribe = [this](std::uint64_t channel_id,
                                             const ::foxglove::ClientMetadata&) {
      subscriptions.Unsubscribe(channel_id);
    };

    auto result = ::foxglove::WebSocketServer::create(std::move(options));
    if (!result.has_value()) {
      throw std::runtime_error(std::string("start WebSocket server: ") +
                               ::foxglove::strerror(result.error()));
    }
    server = std::make_unique<::foxglove::WebSocketServer>(std::move(result).value());
    live_active.store(true, std::memory_order_release);
    MV_LOG_INFO("Foxglove", "WebSocket listening on ws://{}:{}", config.server.host,
                server->port());
  }

  void StartRecording() {
    std::filesystem::create_directories(config.recording.output_dir);
    const auto path = recording_path.string();
    ::foxglove::McapWriterOptions options;
    options.context = recording_context;
    options.path = path;
    options.profile = "foxglove";
    options.compression = ::foxglove::McapCompression::Zstd;
    options.truncate = false;
    auto result = ::foxglove::McapWriter::create(options);
    if (!result.has_value()) {
      throw std::runtime_error(std::string("open MCAP writer: ") +
                               ::foxglove::strerror(result.error()));
    }
    writer = std::make_unique<::foxglove::McapWriter>(std::move(result).value());
    recording_active.store(true, std::memory_order_release);
    MV_LOG_INFO("Foxglove", "MCAP recording: {}", recording_path.string());
  }

  Config config;
  ::foxglove::Context live_context;
  ::foxglove::Context recording_context;
  SubscriptionRegistry subscriptions;
  std::unique_ptr<::foxglove::WebSocketServer> server;
  std::unique_ptr<::foxglove::McapWriter> writer;
  std::filesystem::path recording_path;

  std::atomic<bool> started{false};
  std::atomic<bool> stopped{false};
  std::atomic<bool> live_configured{false};
  std::atomic<bool> recording_configured{false};
  std::atomic<bool> live_active{false};
  std::atomic<bool> recording_active{false};
  std::atomic<bool> recording_closed_cleanly{false};
  std::atomic<std::uint64_t> live_errors{0};
  std::atomic<std::uint64_t> recording_errors{0};
  std::atomic<std::uint64_t> client_connects{0};
  std::atomic<std::uint64_t> client_disconnects{0};
  std::atomic<std::uint64_t> current_clients{0};
};

FoxgloveSession::FoxgloveSession(const Config& config) : impl_(std::make_unique<Impl>(config)) {}

FoxgloveSession::~FoxgloveSession() {
  Stop();
}

bool FoxgloveSession::LiveConfigured() const noexcept {
  return impl_->live_configured.load(std::memory_order_relaxed);
}

bool FoxgloveSession::RecordingConfigured() const noexcept {
  return impl_->recording_configured.load(std::memory_order_relaxed);
}

const ::foxglove::Context& FoxgloveSession::LiveContext() const noexcept {
  return impl_->live_context;
}

const ::foxglove::Context& FoxgloveSession::RecordingContext() const noexcept {
  return impl_->recording_context;
}

void FoxgloveSession::RegisterLiveChannel(std::uint64_t channel_id) {
  impl_->subscriptions.Register(channel_id);
}

SubscriptionSnapshot FoxgloveSession::Subscription(std::uint64_t channel_id) const noexcept {
  return impl_->subscriptions.Snapshot(channel_id);
}

void FoxgloveSession::FailLiveSetup(std::string_view message) noexcept {
  impl_->live_errors.fetch_add(1, std::memory_order_relaxed);
  impl_->live_configured.store(false, std::memory_order_release);
  MV_LOG_ERROR("Foxglove", "live server disabled: {}", message);
}

void FoxgloveSession::FailRecordingSetup(std::string_view message) noexcept {
  impl_->recording_errors.fetch_add(1, std::memory_order_relaxed);
  impl_->recording_configured.store(false, std::memory_order_release);
  MV_LOG_ERROR("Foxglove", "MCAP recording disabled: {}", message);
}

void FoxgloveSession::ReportLiveError(std::string_view operation,
                                      ::foxglove::FoxgloveError error) noexcept {
  impl_->ReportError(operation, error, impl_->live_errors);
  if (IsFatalSinkError(error)) {
    impl_->live_active.store(false, std::memory_order_release);
  }
}

void FoxgloveSession::ReportRecordingError(std::string_view operation,
                                           ::foxglove::FoxgloveError error) noexcept {
  impl_->ReportError(operation, error, impl_->recording_errors);
  if (IsFatalSinkError(error)) {
    impl_->recording_active.store(false, std::memory_order_release);
  }
}

void FoxgloveSession::Start() noexcept {
  if (impl_->started.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  // 两个 sink 独立启动和降级，网络故障不应阻止本地 MCAP，反之亦然。
  if (LiveConfigured()) {
    try {
      impl_->StartLive();
    } catch (const std::exception& error) {
      FailLiveSetup(error.what());
    }
  }
  if (RecordingConfigured()) {
    try {
      impl_->StartRecording();
    } catch (const std::exception& error) {
      FailRecordingSetup(error.what());
    }
  }
}

void FoxgloveSession::Stop() noexcept {
  if (impl_->stopped.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  if (impl_->writer) {
    // close() 会写入 MCAP footer，测试程序依赖该结果判断录制是否完整。
    const auto error = impl_->writer->close();
    impl_->recording_closed_cleanly.store(error == ::foxglove::FoxgloveError::Ok,
                                          std::memory_order_relaxed);
    if (error != ::foxglove::FoxgloveError::Ok) {
      ReportRecordingError("close MCAP", error);
    }
    impl_->writer.reset();
  }
  impl_->recording_active.store(false, std::memory_order_release);

  if (impl_->server) {
    const auto error = impl_->server->stop();
    if (error != ::foxglove::FoxgloveError::Ok) {
      ReportLiveError("stop live server", error);
    }
    impl_->server.reset();
  }
  impl_->live_active.store(false, std::memory_order_release);
}

bool FoxgloveSession::LiveActive() const noexcept {
  return impl_->live_active.load(std::memory_order_relaxed);
}

bool FoxgloveSession::RecordingActive() const noexcept {
  return impl_->recording_active.load(std::memory_order_relaxed);
}

bool FoxgloveSession::AnyActive() const noexcept {
  return LiveActive() || RecordingActive();
}

SessionSnapshot FoxgloveSession::Snapshot() const noexcept {
  return {
      .live_errors = impl_->live_errors.load(std::memory_order_relaxed),
      .recording_errors = impl_->recording_errors.load(std::memory_order_relaxed),
      .client_connects = impl_->client_connects.load(std::memory_order_relaxed),
      .client_disconnects = impl_->client_disconnects.load(std::memory_order_relaxed),
      .current_clients = impl_->current_clients.load(std::memory_order_relaxed),
      .live_active = LiveActive(),
      .recording_active = RecordingActive(),
      .recording_closed_cleanly = impl_->recording_closed_cleanly.load(std::memory_order_relaxed),
  };
}

const std::filesystem::path& FoxgloveSession::RecordingPath() const noexcept {
  return impl_->recording_path;
}

bool IsFatalSinkError(::foxglove::FoxgloveError error) noexcept {
  return error == ::foxglove::FoxgloveError::SinkClosed ||
         error == ::foxglove::FoxgloveError::IoError ||
         error == ::foxglove::FoxgloveError::McapError;
}

}  // namespace mv::tool::foxglove::runtime
