#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_light_detector/armor_light_detector.hpp"
#include "runtime/vision_tuning.hpp"
#include "tool/web/web_debug_config.hpp"
#include "tool/web/web_debug_server.hpp"

#include <array>
#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

namespace {

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;
using Json = nlohmann::json;
using namespace std::chrono_literals;

bool Check(bool condition, std::string_view message) {
  if (!condition)
    std::cerr << "FAILED: " << message << '\n';
  return condition;
}

http::response<http::string_body> Request(std::uint16_t port, http::verb method,
                                          beast::string_view target, std::string_view password = {},
                                          std::string body = {}) {
  asio::io_context context;
  tcp::resolver resolver(context);
  beast::tcp_stream stream(context);
  stream.connect(resolver.resolve("127.0.0.1", std::to_string(port)));
  http::request<http::string_body> request{method, target, 11};
  request.set(http::field::host, "127.0.0.1");
  if (!password.empty())
    request.set(http::field::authorization, "Bearer " + std::string(password));
  if (!body.empty()) {
    request.set(http::field::content_type, "application/json");
    request.body() = std::move(body);
    request.prepare_payload();
  }
  http::write(stream, request);
  beast::flat_buffer buffer;
  http::response<http::string_body> response;
  http::read(stream, buffer, response);
  return response;
}

mv::runtime::VisionPipelineConfig MakeStartupConfig() {
  mv::runtime::VisionPipelineConfig config;
  config.detector.model_path =
      std::filesystem::path(PROJECT_ROOT_PATH) / "src/modules/armor_detector/models/0526.onnx";
  config.detector.device = "GPU";
  config.detector.enemy_color = mv::modules::ArmorColor::BLUE;
  config.detector.confidence_threshold = 0.65F;
  config.detector.nms_iou_threshold = 0.45F;
  return config;
}

mv::runtime::VisionFrontendTuningConfig MakeTuningConfig(
    const mv::runtime::VisionPipelineConfig& config) {
  return {.detector = {.enemy_color = config.detector.enemy_color,
                       .confidence_threshold = config.detector.confidence_threshold,
                       .nms_iou_threshold = config.detector.nms_iou_threshold},
          .corner_refiner = config.corner_refiner,
          .light_detector = config.light_detector};
}

bool TestConfigParsing() {
  const auto VALID = mv::tool::web::ParseConfig(YAML::Load(R"(
schema_version: 1
enabled: true
server: {host: 127.0.0.1, port: 8080}
auth: {password: rmvision2027}
preview: {max_fps: 8, jpeg_quality: 75}
)"));
  bool valid = Check(VALID.server.port == 8080 && VALID.auth.password == "rmvision2027" &&
                         VALID.preview.max_fps == 8.0,
                     "valid Web config must preserve values");
  try {
    static_cast<void>(mv::tool::web::ParseConfig(YAML::Load(R"(
schema_version: 1
enabled: true
server: {host: 127.0.0.1, port: 0}
auth: {password: rmvision2027}
preview: {max_fps: 8, jpeg_quality: 75}
)")));
    valid &= Check(false, "zero port must be rejected");
  } catch (const std::exception&) {
  }

  const auto REJECTS = [](std::string_view yaml) {
    try {
      static_cast<void>(mv::tool::web::ParseConfig(YAML::Load(std::string(yaml))));
      return false;
    } catch (const std::exception&) {
      return true;
    }
  };
  valid &= Check(REJECTS(R"(
schema_version: 1
enabled: true
server: {host: 127.0.0.1, port: 8080, extra: true}
auth: {password: rmvision2027}
preview: {max_fps: 8, jpeg_quality: 75}
)"),
                 "unknown Web config fields must be rejected");
  valid &= Check(REJECTS(R"(
schema_version: 1
enabled: true
server: {host: 127.0.0.1, port: 8080}
auth: {password: rmvision2027}
preview: {max_fps: 31, jpeg_quality: 75}
)"),
                 "preview FPS above 30 must be rejected");
  valid &= Check(REJECTS(R"(
schema_version: 1
enabled: true
server: {host: 127.0.0.1, port: 8080}
auth: {password: rmvision2027}
preview: {max_fps: 8, jpeg_quality: 0}
)"),
                 "invalid JPEG quality must be rejected");

  valid &= Check(REJECTS(R"(
schema_version: 1
enabled: true
server: {host: 127.0.0.1, port: 8080}
auth: {}
preview: {max_fps: 8, jpeg_quality: 75}
)"),
                 "missing password must be rejected");
  valid &= Check(REJECTS(R"(
schema_version: 1
enabled: true
server: {host: 127.0.0.1, port: 8080}
auth: {password: ""}
preview: {max_fps: 8, jpeg_quality: 75}
)"),
                 "empty password must be rejected");
  valid &= Check(REJECTS(R"(
schema_version: 1
enabled: true
server: {host: 127.0.0.1, port: 8080}
auth: {token_env: MV_WEB_DEBUG_TOKEN}
preview: {max_fps: 8, jpeg_quality: 75}
)"),
                 "legacy token_env field must be rejected");

  const auto STARTUP = MakeStartupConfig();
  mv::runtime::VisionTuningMailbox mailbox(MakeTuningConfig(STARTUP));
  mv::tool::web::Config empty_password_config;
  empty_password_config.server.host = "127.0.0.1";
  empty_password_config.server.port = 0;
  empty_password_config.auth.password.clear();
  mv::tool::web::WebDebugServer empty_password_server(empty_password_config, mailbox, STARTUP,
                                                      PROJECT_ROOT_PATH);
  valid &= Check(!empty_password_server.Start(), "empty password must disable only the Web server");
  return valid;
}

bool TestModuleConfigUpdates() {
  mv::modules::ArmorCornerRefinerConfig refiner_config;
  mv::modules::ArmorCornerRefiner refiner(refiner_config);
  refiner_config.enabled = false;
  refiner.UpdateConfig(refiner_config);
  const std::array<cv::Point2f, 4> CORNERS{cv::Point2f{10.0F, 10.0F}, cv::Point2f{30.0F, 10.0F},
                                           cv::Point2f{30.0F, 30.0F}, cv::Point2f{10.0F, 30.0F}};
  const auto REFINEMENT = refiner.Refine(cv::Mat(), CORNERS);

  mv::modules::ArmorLightDetectorConfig light_config;
  mv::modules::ArmorLightDetector light_detector(light_config, mv::modules::ArmorColor::RED);
  light_config.enabled = false;
  light_detector.UpdateConfig(light_config, mv::modules::ArmorColor::BLUE);
  const auto LIGHTS = light_detector.Detect(cv::Mat(), cv::Mat(), {}, {});
  return Check(REFINEMENT.diagnostics.status == mv::modules::CornerRefinementStatus::DISABLED,
               "corner refiner update must affect the following call") &&
         Check(!LIGHTS.diagnostics.enabled && LIGHTS.diagnostics.rejection_reason == "disabled",
               "light detector update must affect the following call");
}

bool TestServerAndTransactions() {
  constexpr char PASSWORD[] = "rmvision2027";
  const auto STARTUP = MakeStartupConfig();
  mv::runtime::VisionTuningMailbox mailbox(MakeTuningConfig(STARTUP));
  mv::tool::web::Config config;
  config.server.host = "127.0.0.1";
  config.server.port = 0;
  config.auth.password = PASSWORD;
  config.preview.max_fps = 30.0;
  mv::tool::web::WebDebugServer server(config, mailbox, STARTUP, PROJECT_ROOT_PATH);
  if (!Check(server.Start(), "loopback Web server must start"))
    return false;
  const auto PORT = server.BoundPort();
  const auto LOGIN_PAGE = Request(PORT, http::verb::get, "/");
  const auto AUTHORIZED_STATUS = Request(PORT, http::verb::get, "/api/v1/status", PASSWORD);

  bool valid =
      Check(LOGIN_PAGE.result() == http::status::ok, "login page must be public") &&
      Check(Request(PORT, http::verb::get, "/api/v1/status").result() == http::status::unauthorized,
            "API without password must return 401") &&
      Check(Request(PORT, http::verb::get, "/api/v1/status", "wrong").result() ==
                http::status::unauthorized,
            "API with wrong password must return 401") &&
      Check(AUTHORIZED_STATUS.result() == http::status::ok,
            "API with correct password must succeed") &&
      Check(LOGIN_PAGE.body().find(PASSWORD) == std::string::npos &&
                AUTHORIZED_STATUS.body().find(PASSWORD) == std::string::npos,
            "password must not be returned by public or status responses");

  const auto CONFIG_RESPONSE = Request(PORT, http::verb::get, "/api/v1/frontend-config", PASSWORD);
  Json config_json = Json::parse(CONFIG_RESPONSE.body());
  Json candidate = config_json.at("active");
  candidate["detector"]["confidence_threshold"] = 0.72;
  const Json REQUEST_JSON{{"base_revision", 1}, {"config", candidate}};
  const auto ACCEPTED =
      Request(PORT, http::verb::put, "/api/v1/frontend-config", PASSWORD, REQUEST_JSON.dump());
  valid &= Check(ACCEPTED.result() == http::status::accepted,
                 "valid tuning transaction must return 202");
  valid &=
      Check(Request(PORT, http::verb::put, "/api/v1/frontend-config", PASSWORD, REQUEST_JSON.dump())
                    .result() == http::status::conflict,
            "second transaction must conflict while one is pending");

  const auto PENDING = mailbox.TryTakePending();
  valid &= Check(PENDING && PENDING->target_revision == 2,
                 "vision thread must receive target revision 2");
  if (PENDING)
    mailbox.MarkApplied({.request_id = PENDING->request_id, .source_sequence = 42});
  const Json STATUS =
      Json::parse(Request(PORT, http::verb::get, "/api/v1/status", PASSWORD).body());
  valid &=
      Check(STATUS["tuning"]["revision"] == 2 && STATUS["tuning"]["last_applied_sequence"] == 42,
            "applied revision and frame sequence must be visible");
  valid &=
      Check(Request(PORT, http::verb::put, "/api/v1/frontend-config", PASSWORD, REQUEST_JSON.dump())
                    .result() == http::status::conflict,
            "stale base revision must return 409");

  candidate["detector"]["confidence_threshold"] = 2.0;
  const Json INVALID_REQUEST{{"base_revision", 2}, {"config", candidate}};
  valid &= Check(
      Request(PORT, http::verb::put, "/api/v1/frontend-config", PASSWORD, INVALID_REQUEST.dump())
              .result() == http::status::unprocessable_entity,
      "invalid detector threshold must return 422");
  valid &= Check(Request(PORT, http::verb::put, "/api/v1/frontend-config", PASSWORD,
                         std::string(std::size_t{70} * 1024, 'x'))
                         .result() == http::status::payload_too_large,
                 "oversized request must return 413");

  mv::frame::FramePacket packet;
  packet.capture.image = cv::Mat(120, 160, CV_8UC3, cv::Scalar(8, 12, 16));
  packet.capture.stamp.sequence = 77;
  packet.capture.stamp.receive_steady_time = std::chrono::steady_clock::now();
  mv::runtime::VisionFrameOutput output;
  mv::runtime::VisionFrameDiagnostics diagnostics;
  diagnostics.detector.total_ms = 3.2;
  diagnostics.detector.threshold_candidates = 4;
  diagnostics.lightbars.binary_threshold = 120;
  server.PublishVision(packet, output, diagnostics, std::nullopt);
  http::response<http::string_body> preview;
  for (int attempt = 0; attempt < 50; ++attempt) {
    preview = Request(PORT, http::verb::get, "/api/v1/preview.jpg", PASSWORD);
    if (preview.result() == http::status::ok)
      break;
    std::this_thread::sleep_for(10ms);
  }
  const std::vector<unsigned char> BYTES(preview.body().begin(), preview.body().end());
  valid &=
      Check(preview.result() == http::status::ok && !cv::imdecode(BYTES, cv::IMREAD_COLOR).empty(),
            "published frame must become a decodable JPEG");

  server.Stop();
  server.Stop();
  return valid;
}

}  // namespace

int main() {
  try {
    if (!TestConfigParsing() || !TestModuleConfigUpdates() || !TestServerAndTransactions())
      return 1;
    std::cout << "Web debug acceptance passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAILED with exception: " << error.what() << '\n';
    return 1;
  }
}
