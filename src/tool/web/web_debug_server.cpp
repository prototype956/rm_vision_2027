#include "tool/web/web_debug_server.hpp"

#include "core/logger.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector_config.hpp"
#include "modules/armor_light_detector/armor_light_detector_config.hpp"
#include "tool/web/latest_preview.hpp"
#include "tool/web/web_assets.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>

namespace mv::tool::web {
namespace {

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;
using Json = nlohmann::json;

constexpr std::size_t K_MAX_REQUEST_BODY_BYTES = std::size_t{64} * 1024;

void RequireKeys(const Json& object, std::initializer_list<const char*> keys,
                 const std::string& context) {
  if (!object.is_object()) {
    throw std::invalid_argument(context + " must be an object");
  }
  std::set<std::string> allowed;
  for (const char* key : keys) {
    allowed.emplace(key);
    if (!object.contains(key)) {
      throw std::invalid_argument(context + " is missing required field '" + key + "'");
    }
  }
  for (const auto& [key, value] : object.items()) {
    static_cast<void>(value);
    if (!allowed.contains(key)) {
      std::string message = context;
      message += " contains unknown field '";
      message += key;
      message += "'";
      throw std::invalid_argument(message);
    }
  }
}

std::string ColorName(modules::ArmorColor color) {
  return color == modules::ArmorColor::RED ? "red" : "blue";
}

Json FrontendConfigJson(const runtime::VisionFrontendTuningConfig& config) {
  return {
      {"detector",
       {{"enemy_color", ColorName(config.detector.enemy_color)},
        {"confidence_threshold", config.detector.confidence_threshold},
        {"nms_iou_threshold", config.detector.nms_iou_threshold}}},
      {"light_detector",
       {{"enabled", config.light_detector.enabled},
        {"threshold",
         {{"fixed", config.light_detector.fixed_binary_threshold},
          {"network_reference_offset", config.light_detector.network_reference_offset},
          {"minimum", config.light_detector.minimum_binary_threshold},
          {"maximum", config.light_detector.maximum_binary_threshold}}},
        {"geometry",
         {{"minimum_contour_points", config.light_detector.minimum_contour_points},
          {"minimum_contour_area_px2", config.light_detector.minimum_contour_area_px2},
          {"minimum_length_px", config.light_detector.minimum_length_px},
          {"minimum_width_length_ratio", config.light_detector.minimum_width_length_ratio},
          {"maximum_width_length_ratio", config.light_detector.maximum_width_length_ratio},
          {"maximum_tilt_rad", config.light_detector.maximum_tilt_rad}}},
        {"color", {{"minimum_channel_difference", config.light_detector.minimum_color_difference}}},
        {"maximum_candidates", config.light_detector.maximum_candidates}}},
      {"corner_refiner",
       {{"enabled", config.corner_refiner.enabled},
        {"pass_optimize_lightbar_width", config.corner_refiner.pass_optimize_lightbar_width},
        {"normalize_max_brightness", config.corner_refiner.normalize_max_brightness},
        {"lightbar_min_mean_brightness", config.corner_refiner.lightbar_min_mean_brightness},
        {"padding_scale", config.corner_refiner.padding_scale},
        {"search_start_ratio", config.corner_refiner.search_start_ratio},
        {"search_end_ratio", config.corner_refiner.search_end_ratio}}}};
}

runtime::VisionFrontendTuningConfig ParseFrontendConfigJson(
    const Json& value, const runtime::VisionPipelineConfig& startup,
    const std::filesystem::path& project_root) {
  RequireKeys(value, {"detector", "light_detector", "corner_refiner"}, "config");
  const auto& detector = value.at("detector");
  RequireKeys(detector, {"enemy_color", "confidence_threshold", "nms_iou_threshold"},
              "config.detector");
  YAML::Node detector_yaml;
  detector_yaml["schema_version"] = modules::ARMOR_DETECTOR_CONFIG_SCHEMA_VERSION;
  detector_yaml["backend"] =
      std::string(modules::ArmorInferenceBackendName(startup.detector.backend));
  detector_yaml["model_path"] = startup.detector.model_path.string();
  detector_yaml["device"] = startup.detector.device;
  detector_yaml["enemy_color"] = detector.at("enemy_color").get<std::string>();
  detector_yaml["confidence_threshold"] = detector.at("confidence_threshold").get<float>();
  detector_yaml["nms_iou_threshold"] = detector.at("nms_iou_threshold").get<float>();
  const auto DETECTOR_CONFIG = modules::ParseArmorDetectorConfig(detector_yaml, project_root);

  const auto& light = value.at("light_detector");
  RequireKeys(light, {"enabled", "threshold", "geometry", "color", "maximum_candidates"},
              "config.light_detector");
  RequireKeys(light.at("threshold"), {"fixed", "network_reference_offset", "minimum", "maximum"},
              "config.light_detector.threshold");
  RequireKeys(light.at("geometry"),
              {"minimum_contour_points", "minimum_contour_area_px2", "minimum_length_px",
               "minimum_width_length_ratio", "maximum_width_length_ratio", "maximum_tilt_rad"},
              "config.light_detector.geometry");
  RequireKeys(light.at("color"), {"minimum_channel_difference"}, "config.light_detector.color");
  YAML::Node light_yaml;
  light_yaml["schema_version"] = modules::ARMOR_LIGHT_DETECTOR_CONFIG_SCHEMA_VERSION;
  light_yaml["enabled"] = light.at("enabled").get<bool>();
  light_yaml["threshold"]["fixed"] = light.at("threshold").at("fixed").get<int>();
  light_yaml["threshold"]["network_reference_offset"] =
      light.at("threshold").at("network_reference_offset").get<int>();
  light_yaml["threshold"]["minimum"] = light.at("threshold").at("minimum").get<int>();
  light_yaml["threshold"]["maximum"] = light.at("threshold").at("maximum").get<int>();
  light_yaml["geometry"]["minimum_contour_points"] =
      light.at("geometry").at("minimum_contour_points").get<int>();
  light_yaml["geometry"]["minimum_contour_area_px2"] =
      light.at("geometry").at("minimum_contour_area_px2").get<double>();
  light_yaml["geometry"]["minimum_length_px"] =
      light.at("geometry").at("minimum_length_px").get<double>();
  light_yaml["geometry"]["minimum_width_length_ratio"] =
      light.at("geometry").at("minimum_width_length_ratio").get<double>();
  light_yaml["geometry"]["maximum_width_length_ratio"] =
      light.at("geometry").at("maximum_width_length_ratio").get<double>();
  light_yaml["geometry"]["maximum_tilt_rad"] =
      light.at("geometry").at("maximum_tilt_rad").get<double>();
  light_yaml["color"]["minimum_channel_difference"] =
      light.at("color").at("minimum_channel_difference").get<double>();
  light_yaml["maximum_candidates"] = light.at("maximum_candidates").get<int>();
  const auto LIGHT_CONFIG = modules::ParseArmorLightDetectorConfig(light_yaml);

  const auto& refiner = value.at("corner_refiner");
  RequireKeys(
      refiner,
      {"enabled", "pass_optimize_lightbar_width", "normalize_max_brightness",
       "lightbar_min_mean_brightness", "padding_scale", "search_start_ratio", "search_end_ratio"},
      "config.corner_refiner");
  YAML::Node refiner_yaml;
  refiner_yaml["schema_version"] = 1;
  refiner_yaml["enabled"] = refiner.at("enabled").get<bool>();
  refiner_yaml["pass_optimize_lightbar_width"] =
      refiner.at("pass_optimize_lightbar_width").get<int>();
  refiner_yaml["normalize_max_brightness"] = refiner.at("normalize_max_brightness").get<double>();
  refiner_yaml["lightbar_min_mean_brightness"] =
      refiner.at("lightbar_min_mean_brightness").get<double>();
  refiner_yaml["padding_scale"] = refiner.at("padding_scale").get<double>();
  refiner_yaml["search_start_ratio"] = refiner.at("search_start_ratio").get<double>();
  refiner_yaml["search_end_ratio"] = refiner.at("search_end_ratio").get<double>();
  const auto REFINER_CONFIG = modules::ParseArmorCornerRefinerConfig(refiner_yaml);

  return {.detector = {.enemy_color = DETECTOR_CONFIG.enemy_color,
                       .confidence_threshold = DETECTOR_CONFIG.confidence_threshold,
                       .nms_iou_threshold = DETECTOR_CONFIG.nms_iou_threshold},
          .corner_refiner = REFINER_CONFIG,
          .light_detector = LIGHT_CONFIG};
}

Json FrontendSchemaJson() {
  Json values = Json::array();
  const auto ADD = [&values](std::string path, std::string group, std::string label,
                             std::string description, std::string type,
                             const Json& options = Json::object()) {
    Json item{{"path", std::move(path)},
              {"group", std::move(group)},
              {"label", std::move(label)},
              {"description", std::move(description)},
              {"type", std::move(type)}};
    item.update(options);
    values.push_back(std::move(item));
  };
  ADD("detector.enemy_color", "装甲检测器", "敌方颜色", "保留的装甲颜色", "enum",
      {{"values", {"red", "blue"}}});
  ADD("detector.confidence_threshold", "装甲检测器", "置信度阈值", "YOLO objectness 门限", "number",
      {{"min", 0.0}, {"max", 1.0}, {"step", 0.01}});
  ADD("detector.nms_iou_threshold", "装甲检测器", "NMS IoU", "重叠框抑制阈值", "number",
      {{"min", 0.0}, {"max", 1.0}, {"step", 0.01}});
  ADD("light_detector.enabled", "独立灯条", "启用", "启用全图独立灯条检测", "boolean");
  ADD("light_detector.threshold.fixed", "独立灯条", "固定阈值", "没有网络参考时的灰度阈值",
      "integer", {{"min", 0}, {"max", 255}, {"step", 1}});
  ADD("light_detector.threshold.network_reference_offset", "独立灯条", "参考偏移",
      "相对网络灯条亮度的阈值偏移", "integer", {{"min", 0}, {"step", 1}});
  ADD("light_detector.threshold.minimum", "独立灯条", "最小阈值", "自适应阈值下界", "integer",
      {{"min", 0}, {"max", 255}, {"step", 1}});
  ADD("light_detector.threshold.maximum", "独立灯条", "最大阈值", "自适应阈值上界", "integer",
      {{"min", 0}, {"max", 255}, {"step", 1}});
  ADD("light_detector.geometry.minimum_contour_points", "独立灯条", "最少轮廓点", "轮廓几何筛选",
      "integer", {{"min", 3}, {"step", 1}});
  ADD("light_detector.geometry.minimum_contour_area_px2", "独立灯条", "最小轮廓面积",
      "轮廓面积下界", "number", {{"min", 0}, {"step", 0.5}, {"unit", "px²"}});
  ADD("light_detector.geometry.minimum_length_px", "独立灯条", "最小长度", "灯条长度下界", "number",
      {{"min", 0.0}, {"step", 0.5}, {"unit", "px"}});
  ADD("light_detector.geometry.minimum_width_length_ratio", "独立灯条", "最小宽长比",
      "灯条宽长比下界", "number", {{"min", 0}, {"max", 1}, {"step", 0.01}});
  ADD("light_detector.geometry.maximum_width_length_ratio", "独立灯条", "最大宽长比",
      "灯条宽长比上界", "number", {{"min", 0}, {"max", 1}, {"step", 0.01}});
  ADD("light_detector.geometry.maximum_tilt_rad", "独立灯条", "最大倾角", "灯条倾角上界", "number",
      {{"min", 0.0}, {"max", 1.570796}, {"step", 0.01}, {"unit", "rad"}});
  ADD("light_detector.color.minimum_channel_difference", "独立灯条", "最小颜色差",
      "敌我颜色通道差下界", "number", {{"min", 0}, {"step", 1}});
  ADD("light_detector.maximum_candidates", "独立灯条", "最大候选数", "每帧候选容量上限", "integer",
      {{"min", 1}, {"step", 1}});
  ADD("corner_refiner.enabled", "角点精修", "启用", "启用灰度矩角点精修", "boolean");
  ADD("corner_refiner.pass_optimize_lightbar_width", "角点精修", "跳过宽度", "过窄灯条跳过精修",
      "integer", {{"min", 0}, {"step", 1}, {"unit", "px"}});
  ADD("corner_refiner.normalize_max_brightness", "角点精修", "归一化亮度", "灰度矩归一化上界",
      "number", {{"min", 0.0}, {"step", 1}});
  ADD("corner_refiner.lightbar_min_mean_brightness", "角点精修", "最小平均亮度",
      "灯条 ROI 接受门限", "number", {{"min", 0}, {"step", 1}});
  ADD("corner_refiner.padding_scale", "角点精修", "ROI 扩张比例", "灯条搜索区域扩张比例", "number",
      {{"min", 0}, {"step", 0.01}});
  ADD("corner_refiner.search_start_ratio", "角点精修", "搜索起点比例", "相对灯条长度", "number",
      {{"min", 0.0}, {"step", 0.01}});
  ADD("corner_refiner.search_end_ratio", "角点精修", "搜索终点比例", "必须大于搜索起点", "number",
      {{"min", 0.02}, {"step", 0.01}});
  return {{"parameters", std::move(values)}};
}

bool ConstantTimeEqual(std::string_view first, std::string_view second) noexcept {
  const std::size_t SIZE = std::max(first.size(), second.size());
  std::size_t difference = first.size() ^ second.size();
  for (std::size_t index = 0; index < SIZE; ++index) {
    const unsigned char LEFT = index < first.size() ? first[index] : 0;
    const unsigned char RIGHT = index < second.size() ? second[index] : 0;
    difference |= LEFT ^ RIGHT;
  }
  return difference == 0;
}

http::response<http::string_body> TextResponse(http::status status, std::string body,
                                               std::string_view content_type) {
  http::response<http::string_body> response{status, 11};
  response.set(http::field::server, "MiracleVision-WebDebug");
  response.set(http::field::content_type,
               beast::string_view(content_type.data(), content_type.size()));
  response.set(http::field::cache_control, "no-store");
  response.keep_alive(false);
  response.body() = std::move(body);
  response.prepare_payload();
  return response;
}

http::response<http::string_body> JsonResponse(http::status status, const Json& value) {
  return TextResponse(status, value.dump(), "application/json; charset=utf-8");
}

}  // namespace

class WebDebugServer::Impl {
 public:
  struct VisionStatus {
    bool has_frame{false};
    std::uint64_t sequence{0};
    int width{0};
    int height{0};
    std::size_t detections{0};
    std::size_t candidates{0};
    double detector_ms{0.0};
    int lightbar_threshold{0};
    std::size_t lightbar_contours{0};
    std::size_t lightbar_kept{0};
    double lightbar_ms{0.0};
    std::chrono::steady_clock::time_point receive_time{};
  };

  class Session final : public std::enable_shared_from_this<Session> {
   public:
    Session(tcp::socket socket, Impl& owner) : socket_(std::move(socket)), owner_(owner) {
      parser_.body_limit(K_MAX_REQUEST_BODY_BYTES);
    }

    void Run() {
      http::async_read(socket_, buffer_, parser_,
                       beast::bind_front_handler(&Session::OnRead, shared_from_this()));
    }

   private:
    void OnRead(beast::error_code error, std::size_t) {
      if (error) {
        if (error == http::error::body_limit) {
          response_ = std::make_shared<http::response<http::string_body>>(JsonResponse(
              http::status::payload_too_large, {{"message", "request body is too large"}}));
          http::async_write(socket_, *response_,
                            beast::bind_front_handler(&Session::OnWrite, shared_from_this()));
          return;
        }
        if (error != http::error::end_of_stream) {
          owner_.error_count_.fetch_add(1, std::memory_order_relaxed);
        }
        return;
      }
      try {
        response_ = std::make_shared<http::response<http::string_body>>(
            owner_.HandleRequest(parser_.release()));
      } catch (const std::exception& exception) {
        owner_.error_count_.fetch_add(1, std::memory_order_relaxed);
        response_ = std::make_shared<http::response<http::string_body>>(
            JsonResponse(http::status::internal_server_error, {{"message", exception.what()}}));
      } catch (...) {
        owner_.error_count_.fetch_add(1, std::memory_order_relaxed);
        response_ = std::make_shared<http::response<http::string_body>>(JsonResponse(
            http::status::internal_server_error, {{"message", "unknown server error"}}));
      }
      http::async_write(socket_, *response_,
                        beast::bind_front_handler(&Session::OnWrite, shared_from_this()));
    }

    void OnWrite(beast::error_code, std::size_t) {
      beast::error_code ignored;
      socket_.shutdown(tcp::socket::shutdown_send, ignored);
    }

    tcp::socket socket_;
    beast::flat_buffer buffer_;
    http::request_parser<http::string_body> parser_;
    std::shared_ptr<http::response<http::string_body>> response_;
    Impl& owner_;
  };

  Impl(Config input_config, runtime::VisionTuningMailbox& input_mailbox,
       runtime::VisionPipelineConfig input_startup, std::filesystem::path input_project_root)
      : config_(std::move(input_config)),
        mailbox_(input_mailbox),
        startup_(std::move(input_startup)),
        project_root_(std::move(input_project_root)),
        preview_(config_.preview),
        acceptor_(io_context_) {}

  bool Start() noexcept {
    if (running_.load(std::memory_order_acquire)) {
      return true;
    }
    try {
      if (config_.auth.password.empty()) {
        MV_LOG_WARN("WebDebug", "disabled because the configured password is empty");
        return false;
      }
      const auto ADDRESS = asio::ip::make_address(config_.server.host);
      const tcp::endpoint ENDPOINT(ADDRESS, config_.server.port);
      beast::error_code error;
      acceptor_.open(ENDPOINT.protocol(), error);
      if (error)
        throw beast::system_error(error);
      acceptor_.set_option(asio::socket_base::reuse_address(true), error);
      if (error)
        throw beast::system_error(error);
      acceptor_.bind(ENDPOINT, error);
      if (error)
        throw beast::system_error(error);
      acceptor_.listen(asio::socket_base::max_listen_connections, error);
      if (error)
        throw beast::system_error(error);
      bound_port_.store(acceptor_.local_endpoint().port(), std::memory_order_release);
      preview_.Start();
      running_.store(true, std::memory_order_release);
      DoAccept();
      io_thread_ = std::thread([this] { io_context_.run(); });
      MV_LOG_INFO("WebDebug", "listening on http://{}:{}", config_.server.host,
                  bound_port_.load(std::memory_order_acquire));
      return true;
    } catch (const std::exception& error) {
      error_count_.fetch_add(1, std::memory_order_relaxed);
      MV_LOG_WARN("WebDebug", "disabled after startup failure: {}", error.what());
      Stop();
      return false;
    } catch (...) {
      error_count_.fetch_add(1, std::memory_order_relaxed);
      MV_LOG_WARN("WebDebug", "disabled after unknown startup failure");
      Stop();
      return false;
    }
  }

  void Stop() noexcept {
    running_.store(false, std::memory_order_release);
    beast::error_code error;
    acceptor_.cancel(error);
    acceptor_.close(error);
    io_context_.stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
    preview_.Stop();
  }

  void DoAccept() {
    acceptor_.async_accept(asio::make_strand(io_context_),
                           [this](beast::error_code error, tcp::socket socket) {
                             if (!error) {
                               std::make_shared<Session>(std::move(socket), *this)->Run();
                             } else if (error != asio::error::operation_aborted) {
                               error_count_.fetch_add(1, std::memory_order_relaxed);
                             }
                             if (running_.load(std::memory_order_acquire)) {
                               DoAccept();
                             }
                           });
  }

  bool Authorized(const http::request<http::string_body>& request) const noexcept {
    const auto VALUE = request[http::field::authorization];
    constexpr std::string_view PREFIX = "Bearer ";
    const std::string_view HEADER(VALUE.data(), VALUE.size());
    return HEADER.starts_with(PREFIX) &&
           ConstantTimeEqual(HEADER.substr(PREFIX.size()), config_.auth.password);
  }

  http::response<http::string_body> HandleRequest(http::request<http::string_body> request) {
    const std::string TARGET(request.target());
    if (TARGET == "/" || TARGET == "/index.html") {
      if (request.method() != http::verb::get) {
        return JsonResponse(http::status::method_not_allowed, {{"message", "method not allowed"}});
      }
      return TextResponse(http::status::ok, std::string(IndexHtml()), "text/html; charset=utf-8");
    }
    if (!Authorized(request)) {
      auto response = JsonResponse(http::status::unauthorized, {{"message", "unauthorized"}});
      response.set(http::field::www_authenticate, "Bearer");
      return response;
    }

    if (TARGET == "/api/v1/status") {
      if (request.method() != http::verb::get)
        return MethodNotAllowed();
      return JsonResponse(http::status::ok, StatusJson());
    }
    if (TARGET == "/api/v1/frontend-schema") {
      if (request.method() != http::verb::get)
        return MethodNotAllowed();
      return JsonResponse(http::status::ok, FrontendSchemaJson());
    }
    if (TARGET == "/api/v1/frontend-config") {
      if (request.method() == http::verb::get) {
        const auto SNAPSHOT = mailbox_.Snapshot();
        return JsonResponse(http::status::ok,
                            {{"revision", SNAPSHOT.active_revision},
                             {"startup", FrontendConfigJson(SNAPSHOT.startup_config)},
                             {"active", FrontendConfigJson(SNAPSHOT.active_config)},
                             {"pending", SNAPSHOT.pending}});
      }
      if (request.method() == http::verb::put) {
        return SubmitConfig(request.body());
      }
      return MethodNotAllowed();
    }
    if (TARGET == "/api/v1/preview.jpg") {
      if (request.method() != http::verb::get)
        return MethodNotAllowed();
      const auto PREVIEW = preview_.Snapshot();
      if (!PREVIEW.jpeg) {
        return JsonResponse(http::status::service_unavailable,
                            {{"message", "preview is not available yet"}});
      }
      return TextResponse(http::status::ok, *PREVIEW.jpeg, "image/jpeg");
    }
    return JsonResponse(http::status::not_found, {{"message", "not found"}});
  }

  http::response<http::string_body> MethodNotAllowed() const {
    return JsonResponse(http::status::method_not_allowed, {{"message", "method not allowed"}});
  }

  http::response<http::string_body> SubmitConfig(const std::string& body) {
    try {
      const Json REQUEST = Json::parse(body);
      RequireKeys(REQUEST, {"base_revision", "config"}, "request");
      const auto BASE_REVISION = REQUEST.at("base_revision").get<std::uint64_t>();
      const auto CONFIG = ParseFrontendConfigJson(REQUEST.at("config"), startup_, project_root_);
      const auto RESULT = mailbox_.Submit(BASE_REVISION, CONFIG);
      if (RESULT.status == runtime::VisionTuningSubmitStatus::REVISION_CONFLICT) {
        return JsonResponse(http::status::conflict, {{"message", "base_revision is stale"}});
      }
      if (RESULT.status == runtime::VisionTuningSubmitStatus::PENDING_REQUEST) {
        return JsonResponse(http::status::conflict,
                            {{"message", "another tuning request is pending"}});
      }
      return JsonResponse(http::status::accepted, {{"request_id", RESULT.request_id},
                                                   {"target_revision", RESULT.target_revision}});
    } catch (const nlohmann::json::exception& error) {
      return JsonResponse(http::status::unprocessable_entity, {{"message", error.what()}});
    } catch (const std::exception& error) {
      return JsonResponse(http::status::unprocessable_entity, {{"message", error.what()}});
    }
  }

  Json StatusJson() const {
    VisionStatus vision;
    {
      std::lock_guard lock(status_mutex_);
      vision = vision_status_;
    }
    double frame_age_ms = 0.0;
    if (vision.has_frame) {
      frame_age_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                               vision.receive_time)
                         .count();
    }
    const auto PREVIEW = preview_.Snapshot();
    const auto TUNING = mailbox_.Snapshot();
    const Json LAST_APPLIED_SEQUENCE =
        TUNING.last_applied_sequence ? Json(*TUNING.last_applied_sequence) : Json(nullptr);
    return {{"running", running_.load(std::memory_order_acquire)},
            {"error_count", error_count_.load(std::memory_order_relaxed)},
            {"vision",
             {{"has_frame", vision.has_frame},
              {"sequence", vision.sequence},
              {"width", vision.width},
              {"height", vision.height},
              {"frame_age_ms", frame_age_ms},
              {"detections", vision.detections},
              {"candidates", vision.candidates},
              {"detector_ms", vision.detector_ms},
              {"lightbar_threshold", vision.lightbar_threshold},
              {"lightbar_contours", vision.lightbar_contours},
              {"lightbar_kept", vision.lightbar_kept},
              {"lightbar_ms", vision.lightbar_ms}}},
            {"preview",
             {{"available", static_cast<bool>(PREVIEW.jpeg)},
              {"sequence", PREVIEW.source_sequence},
              {"overwritten_frames", PREVIEW.overwritten_frames},
              {"encoding_errors", PREVIEW.encoding_errors}}},
            {"tuning",
             {{"revision", TUNING.active_revision},
              {"pending", TUNING.pending},
              {"request_id", TUNING.pending_request_id},
              {"target_revision", TUNING.target_revision},
              {"last_applied_sequence", LAST_APPLIED_SEQUENCE},
              {"last_error", TUNING.last_error}}}};
  }

 private:
  friend class WebDebugServer;

  Config config_;
  runtime::VisionTuningMailbox& mailbox_;
  runtime::VisionPipelineConfig startup_;
  std::filesystem::path project_root_;
  LatestPreview preview_;
  asio::io_context io_context_{1};
  tcp::acceptor acceptor_;
  std::thread io_thread_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint16_t> bound_port_{0};
  std::atomic<std::uint64_t> error_count_{0};
  mutable std::mutex status_mutex_;
  VisionStatus vision_status_;
};

WebDebugServer::WebDebugServer(Config config, runtime::VisionTuningMailbox& tuning_mailbox,
                               runtime::VisionPipelineConfig startup_config,
                               std::filesystem::path project_root)
    : impl_(std::make_unique<Impl>(std::move(config), tuning_mailbox, std::move(startup_config),
                                   std::move(project_root))) {}

WebDebugServer::~WebDebugServer() {
  Stop();
}

bool WebDebugServer::Start() noexcept {
  return impl_->Start();
}

void WebDebugServer::Stop() noexcept {
  impl_->Stop();
}

bool WebDebugServer::IsRunning() const noexcept {
  return impl_->running_.load(std::memory_order_acquire);
}

std::uint16_t WebDebugServer::BoundPort() const noexcept {
  return impl_->bound_port_.load(std::memory_order_acquire);
}

void WebDebugServer::PublishVision(
    const frame::FramePacket& packet, const runtime::VisionFrameOutput& output,
    const runtime::VisionFrameDiagnostics& diagnostics,
    const std::optional<simulation_evaluation::SimulationEvaluationResult>&) noexcept {
  try {
    {
      std::lock_guard lock(impl_->status_mutex_);
      impl_->vision_status_ = {.has_frame = true,
                               .sequence = packet.capture.stamp.sequence,
                               .width = packet.capture.image.cols,
                               .height = packet.capture.image.rows,
                               .detections = output.detections.size(),
                               .candidates = diagnostics.detector.threshold_candidates,
                               .detector_ms = diagnostics.detector.total_ms,
                               .lightbar_threshold = diagnostics.lightbars.binary_threshold,
                               .lightbar_contours = diagnostics.lightbars.contours,
                               .lightbar_kept = diagnostics.lightbars.kept_candidates,
                               .lightbar_ms = diagnostics.lightbars.elapsed_ms,
                               .receive_time = packet.capture.stamp.receive_steady_time};
    }
    impl_->preview_.Push(packet, output, diagnostics);
  } catch (...) {
    impl_->error_count_.fetch_add(1, std::memory_order_relaxed);
  }
}

void WebDebugServer::PublishControl(const runtime::ControlCycleOutput&,
                                    const runtime::ControlCycleDiagnostics&) noexcept {}

runtime::RuntimeDiagnosticsHealth WebDebugServer::SnapshotHealth() const noexcept {
  const auto PREVIEW = impl_->preview_.Snapshot();
  return {
      .available = IsRunning(),
      .error_count = impl_->error_count_.load(std::memory_order_relaxed) + PREVIEW.encoding_errors};
}

}  // namespace mv::tool::web
