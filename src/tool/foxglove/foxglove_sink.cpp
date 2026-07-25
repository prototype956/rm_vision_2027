/**
 * @file foxglove_sink.cpp
 * @brief FoxgloveSink Pimpl 实现
 *
 * 【Pimpl 布局】
 *   Impl struct 集中管理：
 *   - foxglove::Context        ：SDK 全局上下文，所有子发布器共享同一实例
 *   - foxglove::WebSocketServer ：WebSocket 服务端，由 detail::BuildServerOptions 构建
 *   - 七个子发布器              ：ImagePublisher / DetectionPublisher / PnpVisualizer /
 *                                TfPublisher / ThreadMonitor / GimbalPublisher /
 *                                TrackingVisualizer
 *   - param_store               ：foxglove::Parameter key→value 映射
 *
 * 【Channel 生命周期】
 *   每个子发布器内部懒创建自己的 Channel（EnsureChannels()），
 *   首次 Publish() 时才 advertise。
 *
 * 【零客户端门控】
 *   PublishImage / PublishPnpResult 在 HasClients()==false 时提前返回，
 *   避免昂贵的图像 clone + memcpy 拖累 Pipeline 帧率。
 *
 * 【参数双向调节实现】
 *   回调组装逻辑已封装到 detail/server_builder.hpp，
 *   onSetParameters 释放锁后再通知 param_callback，防止递归死锁。
 */
#include "tool/foxglove/foxglove_sink.hpp"

#include "tool/foxglove/detail/detection_publisher.hpp"
#include "tool/foxglove/detail/gimbal_publisher.hpp"
#include "tool/foxglove/detail/hud_status_publisher.hpp"
#include "tool/foxglove/detail/image_publisher.hpp"
#include "tool/foxglove/detail/pnp_visualizer.hpp"
#include "tool/foxglove/detail/server_builder.hpp"
#include "tool/foxglove/detail/tf_publisher.hpp"
#include "tool/foxglove/detail/thread_monitor.hpp"
#include "tool/foxglove/detail/tracking_visualizer.hpp"
#include "tool/foxglove/detail/traditional_vision_debug_publisher.hpp"
#include "tool/foxglove/detail/utils.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <foxglove/channel.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/schemas.hpp>
#include <foxglove/server.hpp>
#include <foxglove/server/parameter.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <spdlog/spdlog.h>

namespace mv::tool {

// ============================================================================
// Impl
// ============================================================================

struct FoxgloveSink::Impl {
  // ── Server 核心 ──────────────────────────────────────────────────────────
  foxglove::Context ctx;
  std::unique_ptr<foxglove::WebSocketServer> server;
  bool is_running{false};

  // ── 子发布器 ──────────────────────────────────────────────────────────────
  std::unique_ptr<detail::ImagePublisher> image_pub;
  std::unique_ptr<detail::DetectionPublisher> detection_pub;
  std::unique_ptr<detail::PnpVisualizer> pnp_viz;
  std::unique_ptr<detail::TraditionalVisionDebugPublisher> tv_debug_pub;
  std::unique_ptr<detail::HudStatusPublisher> hud_status_pub;
  std::unique_ptr<detail::TfPublisher> tf_pub;
  std::unique_ptr<detail::ThreadMonitor> thread_monitor;
  std::unique_ptr<detail::GimbalPublisher> gimbal_pub;
  std::unique_ptr<detail::TrackingVisualizer> tracking_viz;

  // ── 通用 JSON channels（PublishJson 懒创建）─────────────────────────────
  std::unordered_map<std::string, foxglove::RawChannel> json_chs;
  std::mutex json_chs_mtx;

  // ── 连接状态 ──────────────────────────────────────────────────────────────
  std::atomic<int> client_count{0};

  [[nodiscard]] bool HasClients() const noexcept {
    return client_count.load(std::memory_order_relaxed) > 0;
  }

  // ── 参数双向调节 ─────────────────────────────────────────────────────────
  std::unordered_map<std::string, foxglove::Parameter> param_store;
  mutable std::mutex param_mutex;
  ParameterCallback param_callback;

  // ── 构造 ─────────────────────────────────────────────────────────────────
  explicit Impl(const FoxgloveSink::Config& cfg) {
    foxglove::setLogLevel(foxglove::LogLevel::Info);
    ctx = foxglove::Context::create();

    // Server 回调组装（细节隔离在 detail/server_builder.hpp）
    detail::ServerCallbackArgs cb_args{
        .client_count = client_count,
        .param_store = param_store,
        .param_mutex = param_mutex,
        .param_callback = param_callback,
        .server = server,
    };
    auto options = detail::BuildServerOptions(cfg, ctx, cb_args);

    auto server_res = foxglove::WebSocketServer::create(std::move(options));
    if (!server_res.has_value()) {
      spdlog::error("[FoxgloveSink] Failed to create WebSocket server: {}",
                    foxglove::strerror(server_res.error()));
      throw std::runtime_error("FoxgloveSink: WebSocket server creation failed");
    }
    server = std::make_unique<foxglove::WebSocketServer>(std::move(server_res.value()));

    // 初始化八个子发布器（共享同一 Context）
    image_pub = std::make_unique<detail::ImagePublisher>(ctx, cfg.use_jpeg, cfg.jpeg_quality,
                                                         cfg.publish_width, cfg.publish_height);
    detection_pub = std::make_unique<detail::DetectionPublisher>(ctx);
    pnp_viz = std::make_unique<detail::PnpVisualizer>(ctx);
    tv_debug_pub = std::make_unique<detail::TraditionalVisionDebugPublisher>(ctx);
    hud_status_pub = std::make_unique<detail::HudStatusPublisher>(ctx);
    pnp_viz->SetArmorDims(cfg.armor_small_half_w, cfg.armor_big_half_w, cfg.armor_half_h);
    tf_pub = std::make_unique<detail::TfPublisher>(ctx);
    thread_monitor = std::make_unique<detail::ThreadMonitor>(ctx);
    gimbal_pub = std::make_unique<detail::GimbalPublisher>(ctx);
    tracking_viz = std::make_unique<detail::TrackingVisualizer>(ctx);
  }
};

// ============================================================================
// FoxgloveSink public interface
// ============================================================================

FoxgloveSink::FoxgloveSink() : FoxgloveSink(Config{}) {}
FoxgloveSink::FoxgloveSink(const Config& cfg) : impl_(std::make_unique<Impl>(cfg)) {}
FoxgloveSink::~FoxgloveSink() = default;
FoxgloveSink::FoxgloveSink(FoxgloveSink&&) noexcept = default;
FoxgloveSink& FoxgloveSink::operator=(FoxgloveSink&&) noexcept = default;

void FoxgloveSink::Start() {
  if (impl_->server && !impl_->is_running) {
    impl_->is_running = true;
    spdlog::info("[FoxgloveSink] WebSocket server started on port {}", 8765);
  }
}

// ── 传统视觉调试图 ────────────────────────────────────────────────────────────

void FoxgloveSink::PublishTraditionalVisionDebug(
    const cv::Mat& diff, const cv::Mat& binary, const cv::Rect2i& roi_rect,
    const cv::Size& frame_size, const cv::Mat& raw_frame,
    const TraditionalVisionLightVisParams& light_vis_params, int64_t ts_ns) {
  // 零客户端时跳过调试图发布
  if (!impl_->HasClients()) {
    return;
  }
  detail::TraditionalVisionDebugPublisher::LightVisParams publisher_light_params;
  publisher_light_params.min_light_ratio = light_vis_params.min_light_ratio;
  publisher_light_params.max_light_ratio = light_vis_params.max_light_ratio;
  publisher_light_params.max_light_angle = light_vis_params.max_light_angle;
  publisher_light_params.min_area = light_vis_params.min_area;

  impl_->tv_debug_pub->Publish(diff, binary, roi_rect, frame_size, raw_frame,
                               publisher_light_params, detail::ResolveTs(ts_ns));
}

// ── HUD 状态 JSON 同步 ────────────────────────────────────────────────────

namespace {
void PublishHudStatusHelper(detail::HudStatusPublisher* pub, double fps, int detection_count,
                            bool tracking, bool serial_alive, const std::string& enemy_color,
                            double target_yaw_deg, double target_pitch_deg,
                            double target_distance_m, uint64_t timestamp_ns) {
  detail::HudStatusPublisher::HudStatus status;
  status.fps = fps;
  status.detection_count = detection_count;
  status.tracking = tracking;
  status.serial_alive = serial_alive;
  status.enemy_color = enemy_color;
  status.target_yaw_deg = target_yaw_deg;
  status.target_pitch_deg = target_pitch_deg;
  status.target_distance_m = target_distance_m;
  status.timestamp_us = timestamp_ns / 1000;  // 纳秒转微秒

  pub->Publish(status, timestamp_ns);
}
}  // namespace

void FoxgloveSink::PublishHudStatus(double fps, int detection_count, bool tracking,
                                    bool serial_alive, const std::string& enemy_color,
                                    double target_yaw_deg, double target_pitch_deg,
                                    double target_distance_m, int64_t ts_ns) {
  const uint64_t kTimestampNs = detail::ResolveTs(ts_ns);
  PublishHudStatusHelper(impl_->hud_status_pub.get(), fps, detection_count, tracking, serial_alive,
                         enemy_color, target_yaw_deg, target_pitch_deg, target_distance_m,
                         kTimestampNs);
}

void FoxgloveSink::Stop() {
  if (impl_->is_running && impl_->server) {
    impl_->server->stop();
    impl_->is_running = false;
    spdlog::info("[FoxgloveSink] WebSocket server stopped");
  }
}

bool FoxgloveSink::HasClients() const noexcept {
  return impl_->HasClients();
}

// ── 图像 ─────────────────────────────────────────────────────────────────────

void FoxgloveSink::PublishImage(const cv::Mat& img, const std::string& topic,
                                const std::string& frame_id, int64_t ts_ns) {
  // 零客户端时提前返回：图像 clone + memcpy(~1.5MB/帧) 会占用约 2ms，
  // 在 120fps Pipeline 中累积为不可忽甥的延迟，无接收方时完全没有意么。
  if (!impl_->HasClients()) {
    return;
  }
  impl_->image_pub->Publish(img, topic, frame_id, detail::ResolveTs(ts_ns));
}

// ── 检测结果 ──────────────────────────────────────────────────────────────────

void FoxgloveSink::PublishDetections(const std::vector<mv::Detection>& dets, int64_t ts_ns) {
  impl_->detection_pub->Publish(dets, detail::ResolveTs(ts_ns));
}

// ── PnP 专项调试 ──────────────────────────────────────────────────────────────

void FoxgloveSink::PublishPnpResult(const std::vector<mv::Detection>& dets, const cv::Mat& frame,
                                    int64_t ts_ns) {
  // 零客户端时跳过调试图像绘制和编码
  if (!impl_->HasClients())
    return;
  impl_->pnp_viz->Publish(dets, frame, detail::ResolveTs(ts_ns));
}

// ── TF 坐标系 ─────────────────────────────────────────────────────────────────

void FoxgloveSink::PublishTransform(const std::string& parent, const std::string& child,
                                    const Eigen::Matrix4d& transform, int64_t ts_ns) {
  impl_->tf_pub->Publish(parent, child, transform, detail::ResolveTs(ts_ns));
}

// ── 线程健康 ──────────────────────────────────────────────────────────────────

void FoxgloveSink::PublishThreadMetrics(const std::vector<ThreadMetrics>& metrics, int64_t ts_ns) {
  impl_->thread_monitor->Publish(metrics, detail::ResolveTs(ts_ns));
}

// ── 云台控制指令 ──────────────────────────────────────────────────────────────

void FoxgloveSink::PublishGimbalControl(const mv::GimbalControl& ctrl, int64_t ts_ns) {
  impl_->gimbal_pub->Publish(ctrl, detail::ResolveTs(ts_ns));
}

// ── 参数双向调节 ──────────────────────────────────────────────────────────────

void FoxgloveSink::SetParameterCallback(ParameterCallback callback) {
  impl_->param_callback = std::move(callback);
}

void FoxgloveSink::UpdateParameters(const nlohmann::json& params) {
  if (!impl_->server) {
    return;
  }

  std::lock_guard<std::mutex> lock(impl_->param_mutex);
  std::vector<foxglove::Parameter> to_publish;

  for (const auto& [key, val] : params.items()) {
    foxglove::Parameter param(key);

    if (val.is_boolean()) {
      param = foxglove::Parameter(key, val.get<bool>());
    } else if (val.is_number_integer()) {
      param = foxglove::Parameter(key, val.get<int64_t>());
    } else if (val.is_number_float()) {
      param = foxglove::Parameter(key, val.get<double>());
    } else if (val.is_string()) {
      param = foxglove::Parameter(key, val.get<std::string>());
    } else if (val.is_array() && !val.empty()) {
      if (val[0].is_number()) {
        param = foxglove::Parameter(key, val.get<std::vector<double>>());
      } else {
        param = foxglove::Parameter(key, val.dump());  // JSON 字符串
      }
    } else if (val.is_object()) {
      param = foxglove::Parameter(key, val.dump());
    }

    impl_->param_store.insert_or_assign(key, std::move(param));
    to_publish.push_back(impl_->param_store.at(key).clone());
  }

  if (!to_publish.empty()) {
    impl_->server->publishParameterValues(std::move(to_publish));
  }
}

nlohmann::json FoxgloveSink::GetParameter(const std::string& name) const {
  std::lock_guard<std::mutex> lock(impl_->param_mutex);
  auto iterator = impl_->param_store.find(name);
  if (iterator == impl_->param_store.end()) {
    return nullptr;
  }
  const auto& param = iterator->second;
  if (param.is<bool>()) {
    return param.get<bool>();
  }
  if (param.is<int64_t>()) {
    return param.get<int64_t>();
  }
  if (param.is<double>()) {
    return param.get<double>();
  }
  if (param.is<std::string>()) {
    return param.get<std::string>();
  }
  if (param.is<std::vector<double>>()) {
    return param.get<std::vector<double>>();
  }
  if (param.is<std::vector<int64_t>>()) {
    return param.get<std::vector<int64_t>>();
  }
  return nullptr;
}

// ── 通用 JSON 发布 ────────────────────────────────────────────────────────────

void FoxgloveSink::PublishJson(const std::string& topic, const nlohmann::json& data,
                               int64_t ts_ns) {
  const uint64_t timestamp_ns = detail::ResolveTs(ts_ns);
  std::lock_guard<std::mutex> lock(impl_->json_chs_mtx);

  if (impl_->json_chs.find(topic) == impl_->json_chs.end()) {
    auto res = foxglove::RawChannel::create(topic, "json", {}, impl_->ctx);
    if (!res.has_value()) {
      spdlog::warn("[FoxgloveSink] PublishJson: failed to create channel '{}': {}", topic,
                   foxglove::strerror(res.error()));
      return;
    }
    impl_->json_chs.emplace(topic, std::move(res.value()));
  }

  const std::string json_str = data.dump();
  impl_->json_chs.at(topic).log(reinterpret_cast<const std::byte*>(json_str.data()),
                                json_str.size(), timestamp_ns);
}

// ── 预测跟踪 3D 可视化 ────────────────────────────────────────────────────────

void FoxgloveSink::PublishTrackingVisuals(const mv::TrackTarget& target,
                                          const Eigen::Vector3d& aim_xyz,
                                          const std::string& frame_id, int64_t ts_ns) {
  impl_->tracking_viz->Publish(target, aim_xyz, frame_id, detail::ResolveTs(ts_ns));
}

}  // namespace mv::tool
