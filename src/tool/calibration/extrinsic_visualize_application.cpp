#include "tool/calibration/extrinsic_visualize_application.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "geometry/rigid_transform.hpp"
#if defined(MV_EXTRINSIC_HAS_FOXGLOVE) && MV_EXTRINSIC_HAS_FOXGLOVE
#include "tool/foxglove/foxglove_config.hpp"
#include "tool/foxglove/runtime/foxglove_session.hpp"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <filesystem>
#if defined(MV_EXTRINSIC_HAS_FOXGLOVE) && MV_EXTRINSIC_HAS_FOXGLOVE
#include <fmt/format.h>
#include <foxglove/schemas.hpp>
#endif
#include <yaml-cpp/yaml.h>

namespace mv::tool::calibration {
#if defined(MV_EXTRINSIC_HAS_FOXGLOVE) && MV_EXTRINSIC_HAS_FOXGLOVE
namespace {

constexpr char K_CONTEXT[] = "extrinsic visualization";
constexpr char K_TRANSFORMS_TOPIC[] = "/calibration/extrinsic/transforms";
constexpr char K_SCENE_TOPIC[] = "/calibration/extrinsic/scene";
constexpr double K_AXIS_LENGTH = 0.18;
constexpr auto K_FRAME_PERIOD = std::chrono::milliseconds(750);
constexpr double K_ROTATION_EPSILON = 1.0e-5;
volatile std::sig_atomic_t g_stop_requested = 0;

void HandleStopSignal(int) noexcept {
  g_stop_requested = 1;
}

template <typename T>
T Require(const YAML::Node& node, const std::string& key) {
  return ConfigLoader::Require<T>(node, key, K_CONTEXT);
}

std::vector<double> ReadVector(const YAML::Node& node, const std::string& key, std::size_t size) {
  const auto VALUE = node[key];
  if (!VALUE || !VALUE.IsSequence() || VALUE.size() != size) {
    throw ConfigError(std::string(K_CONTEXT) + "." + key + " must contain " + std::to_string(size) +
                      " values");
  }
  std::vector<double> result;
  result.reserve(size);
  for (const auto& item : VALUE) {
    const double NUMBER = item.as<double>();
    if (!std::isfinite(NUMBER)) {
      throw ConfigError(std::string(K_CONTEXT) + "." + key + " contains a non-finite value");
    }
    result.push_back(NUMBER);
  }
  return result;
}

geometry::RigidTransform ReadTransform(const YAML::Node& node) {
  ConfigLoader::RequireMap(node, K_CONTEXT);
  // 解算结果同时保存旋转矩阵；回放以规范化四元数为准，矩阵仅作为冗余校验字段保留。
  ConfigLoader::RejectUnknownKeys(node, {"translation", "rotation_xyzw", "rotation_matrix"},
                                  K_CONTEXT);
  const auto TRANSLATION = ReadVector(node, "translation", 3);
  const auto ROTATION = ReadVector(node, "rotation_xyzw", 4);
  geometry::RigidTransform result;
  result.translation = geometry::Vector3(TRANSLATION[0], TRANSLATION[1], TRANSLATION[2]);
  result.rotation = geometry::Quaternion(ROTATION[3], ROTATION[0], ROTATION[1], ROTATION[2]);
  if (!result.translation.allFinite() || !result.rotation.coeffs().allFinite() ||
      std::abs(result.rotation.norm() - 1.0) > K_ROTATION_EPSILON) {
    throw ConfigError(std::string(K_CONTEXT) + " contains an invalid transform");
  }
  result.rotation.normalize();
  return result;
}

struct Sample {
  std::size_t id{0};
  std::uint64_t capture_timestamp_ns{0};
  geometry::RigidTransform world_t_gimbal;
  geometry::RigidTransform gimbal_t_camera_truth;
  double rotation_error_deg{0.0};
  double translation_error_m{0.0};
};

struct VisualizationData {
  geometry::RigidTransform estimate;
  std::vector<Sample> samples;
  double rotation_mean_deg{0.0};
  double translation_mean_m{0.0};
};

VisualizationData LoadSolution(const std::filesystem::path& solution_dir,
                               const ExtrinsicCaptureSettings& settings) {
  const auto ROOT = std::filesystem::absolute(solution_dir).lexically_normal();
  const auto EXTRINSICS_PATH = ROOT / "extrinsics.yaml";
  const auto REPORT_PATH = ROOT / "report.yaml";
  if (!std::filesystem::is_regular_file(EXTRINSICS_PATH) ||
      !std::filesystem::is_regular_file(REPORT_PATH)) {
    throw ConfigError("solution must contain extrinsics.yaml and report.yaml: " + ROOT.string());
  }

  const auto EXTRINSICS = YAML::LoadFile(EXTRINSICS_PATH.string());
  ConfigLoader::RequireMap(EXTRINSICS, K_CONTEXT);
  ConfigLoader::RejectUnknownKeys(EXTRINSICS,
                                  {"schema_version", "translation_unit", "transform_convention",
                                   "target", "source_session", "gimbal_t_camera_optical"},
                                  K_CONTEXT);
  if (Require<int>(EXTRINSICS, "schema_version") != 1 ||
      Require<std::string>(EXTRINSICS, "translation_unit") != "m" ||
      Require<std::string>(EXTRINSICS, "transform_convention") != "parent_t_child" ||
      Require<std::string>(EXTRINSICS, "target") != "gimbal_t_camera_optical") {
    throw ConfigError("unsupported extrinsic result metadata");
  }
  VisualizationData result;
  result.estimate = ReadTransform(EXTRINSICS["gimbal_t_camera_optical"]);

  const auto REPORT = YAML::LoadFile(REPORT_PATH.string());
  ConfigLoader::RequireMap(REPORT, K_CONTEXT);
  ConfigLoader::RejectUnknownKeys(
      REPORT,
      {"schema_version", "status", "source_session", "board", "camera", "sample_counts", "samples",
       "estimated_gimbal_t_camera_optical", "truth_reference_sample_id",
       "truth_reference_gimbal_t_camera_optical", "error_statistics", "error"},
      K_CONTEXT);
  if (Require<int>(REPORT, "schema_version") != 1 ||
      Require<std::string>(REPORT, "status") != "solved") {
    throw ConfigError("report is not a solved schema version 1 result");
  }
  const auto BOARD = REPORT["board"];
  ConfigLoader::RequireMap(BOARD, K_CONTEXT);
  if (Require<int>(BOARD, "pattern_cols") != settings.board_columns ||
      Require<int>(BOARD, "pattern_rows") != settings.board_rows ||
      std::abs(Require<double>(BOARD, "square_size_mm") - settings.square_size_mm) > 1.0e-9) {
    throw ConfigError("report board parameters do not match calibration configuration");
  }

  const auto STATISTICS = REPORT["error_statistics"];
  ConfigLoader::RequireMap(STATISTICS, K_CONTEXT);
  result.rotation_mean_deg = Require<double>(STATISTICS, "rotation_mean_deg");
  result.translation_mean_m = Require<double>(STATISTICS, "translation_mean_m");
  const auto SAMPLES = REPORT["samples"];
  if (!SAMPLES || !SAMPLES.IsSequence()) {
    throw ConfigError("report.samples must be a sequence");
  }
  for (const auto& item : SAMPLES) {
    if (!item.IsMap() || !item["detected"].as<bool>() || !item["accepted"].as<bool>()) {
      continue;
    }
    ConfigLoader::RejectUnknownKeys(
        item,
        {"id", "image", "frame_sequence", "simulation_round_id", "capture_timestamp_ns", "detected",
         "accepted", "pnp_reprojection_rms_px", "world_t_gimbal", "gimbal_t_camera_optical_truth",
         "board_t_camera", "rotation_error_deg", "translation_error_m"},
        K_CONTEXT);
    Sample sample;
    sample.id = Require<std::size_t>(item, "id");
    sample.capture_timestamp_ns = Require<std::uint64_t>(item, "capture_timestamp_ns");
    sample.world_t_gimbal = ReadTransform(item["world_t_gimbal"]);
    sample.gimbal_t_camera_truth = ReadTransform(item["gimbal_t_camera_optical_truth"]);
    sample.rotation_error_deg = Require<double>(item, "rotation_error_deg");
    sample.translation_error_m = Require<double>(item, "translation_error_m");
    if (sample.id == 0 || sample.capture_timestamp_ns == 0 ||
        !std::isfinite(sample.rotation_error_deg) || !std::isfinite(sample.translation_error_m)) {
      throw ConfigError("report contains an invalid accepted sample");
    }
    result.samples.push_back(std::move(sample));
  }
  if (result.samples.size() < static_cast<std::size_t>(settings.solve_min_samples)) {
    throw ConfigError("report contains too few accepted samples");
  }
  std::sort(result.samples.begin(), result.samples.end(), [](const Sample& lhs, const Sample& rhs) {
    return lhs.capture_timestamp_ns < rhs.capture_timestamp_ns;
  });
  return result;
}

::foxglove::schemas::Timestamp ToTimestamp(std::uint64_t timestamp_ns) {
  return {.sec = static_cast<std::uint32_t>(timestamp_ns / 1'000'000'000ULL),
          .nsec = static_cast<std::uint32_t>(timestamp_ns % 1'000'000'000ULL)};
}

::foxglove::schemas::Vector3 ToVector(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

::foxglove::schemas::Point3 ToPoint(const geometry::Vector3& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z()};
}

::foxglove::schemas::Quaternion ToQuaternion(const geometry::Quaternion& value) {
  return {.x = value.x(), .y = value.y(), .z = value.z(), .w = value.w()};
}

::foxglove::schemas::FrameTransform MakeTransform(std::string parent, std::string child,
                                                  const geometry::RigidTransform& value,
                                                  const ::foxglove::schemas::Timestamp& timestamp) {
  return {.timestamp = timestamp,
          .parent_frame_id = std::move(parent),
          .child_frame_id = std::move(child),
          .translation = ToVector(value.translation),
          .rotation = ToQuaternion(value.rotation)};
}

void AddAxes(::foxglove::schemas::SceneEntity& entity, const geometry::RigidTransform& pose,
             double alpha) {
  const auto ORIGIN = pose.translation;
  const std::array<geometry::Vector3, 3> DIRECTIONS = {
      geometry::TransformVector(pose, geometry::Vector3::UnitX()) * K_AXIS_LENGTH,
      geometry::TransformVector(pose, geometry::Vector3::UnitY()) * K_AXIS_LENGTH,
      geometry::TransformVector(pose, geometry::Vector3::UnitZ()) * K_AXIS_LENGTH};
  const std::array<::foxglove::schemas::Color, 3> COLORS = {
      ::foxglove::schemas::Color{.r = 1.0, .a = alpha},
      ::foxglove::schemas::Color{.g = 1.0, .a = alpha},
      ::foxglove::schemas::Color{.b = 1.0, .a = alpha}};
  ::foxglove::schemas::LinePrimitive axes;
  axes.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
  axes.thickness = 0.012;
  axes.points.reserve(6);
  axes.colors.reserve(6);
  for (std::size_t index = 0; index < DIRECTIONS.size(); ++index) {
    axes.points.push_back(ToPoint(ORIGIN));
    axes.points.push_back(ToPoint(ORIGIN + DIRECTIONS[index]));
    axes.colors.push_back(COLORS[index]);
    axes.colors.push_back(COLORS[index]);
  }
  entity.lines.push_back(std::move(axes));
}

void AddLabel(::foxglove::schemas::SceneEntity& entity, const geometry::Vector3& position,
              std::string text, const ::foxglove::schemas::Color& color) {
  ::foxglove::schemas::TextPrimitive label;
  label.pose = ::foxglove::schemas::Pose{.position = ToVector(position),
                                         .orientation = ::foxglove::schemas::Quaternion{.w = 1.0}};
  label.billboard = true;
  label.font_size = 11.0;
  label.scale_invariant = true;
  label.color = color;
  label.text = std::move(text);
  entity.texts.push_back(std::move(label));
}

::foxglove::schemas::FrameTransforms EncodeTransforms(
    const Sample& sample, const geometry::RigidTransform& estimate,
    const ::foxglove::schemas::Timestamp& timestamp) {
  ::foxglove::schemas::FrameTransforms message;
  message.transforms = {
      MakeTransform("world", "gimbal_truth", sample.world_t_gimbal, timestamp),
      MakeTransform("gimbal_truth", "camera_optical_truth", sample.gimbal_t_camera_truth,
                    timestamp),
      MakeTransform("world", "gimbal_estimated", sample.world_t_gimbal, timestamp),
      MakeTransform("gimbal_estimated", "camera_optical_estimated", estimate, timestamp)};
  return message;
}

::foxglove::schemas::SceneUpdate EncodeScene(const Sample& sample,
                                             const geometry::RigidTransform& estimate,
                                             const ::foxglove::schemas::Timestamp& timestamp,
                                             double rotation_mean_deg, double translation_mean_m) {
  const auto WORLD_T_CAMERA_TRUTH =
      geometry::Compose(sample.world_t_gimbal, sample.gimbal_t_camera_truth);
  const auto WORLD_T_CAMERA_ESTIMATE = geometry::Compose(sample.world_t_gimbal, estimate);
  ::foxglove::schemas::SceneUpdate update;
  ::foxglove::schemas::SceneEntity entity;
  entity.timestamp = timestamp;
  entity.frame_id = "world";
  entity.id = "extrinsic_calibration_frames";
  entity.lifetime = {.sec = 2, .nsec = 0};
  entity.metadata = {
      {.key = "sample_id", .value = std::to_string(sample.id)},
      {.key = "rotation_error_deg", .value = fmt::format("{:.6f}", sample.rotation_error_deg)},
      {.key = "translation_error_m", .value = fmt::format("{:.6f}", sample.translation_error_m)}};

  geometry::RigidTransform world{};
  AddAxes(entity, world, 0.95);
  AddAxes(entity, sample.world_t_gimbal, 0.95);
  AddAxes(entity, WORLD_T_CAMERA_TRUTH, 0.45);
  AddAxes(entity, WORLD_T_CAMERA_ESTIMATE, 1.0);
  AddLabel(entity, WORLD_T_CAMERA_TRUTH.translation + geometry::Vector3(0.0, 0.0, 0.08),
           fmt::format("truth camera #{}", sample.id), {.r = 0.7, .g = 0.7, .b = 1.0, .a = 1.0});
  AddLabel(entity, WORLD_T_CAMERA_ESTIMATE.translation + geometry::Vector3(0.0, 0.0, 0.12),
           fmt::format("estimated camera #{}", sample.id),
           {.r = 1.0, .g = 0.8, .b = 0.1, .a = 1.0});
  AddLabel(entity, geometry::Vector3(0.0, 0.0, 0.12),
           fmt::format(
               "world | sample {} | rot {:.3f} deg | trans {:.3f} mm | mean {:.3f} deg / {:.3f} mm",
               sample.id, sample.rotation_error_deg, sample.translation_error_m * 1000.0,
               rotation_mean_deg, translation_mean_m * 1000.0),
           {.r = 1.0, .g = 1.0, .b = 1.0, .a = 1.0});

  ::foxglove::schemas::LinePrimitive error_line;
  error_line.type = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
  error_line.thickness = 0.018;
  error_line.color = ::foxglove::schemas::Color{.r = 1.0, .g = 0.2, .b = 0.8, .a = 1.0};
  error_line.points = {ToPoint(WORLD_T_CAMERA_ESTIMATE.translation),
                       ToPoint(WORLD_T_CAMERA_TRUTH.translation)};
  entity.lines.push_back(std::move(error_line));
  update.entities.push_back(std::move(entity));
  return update;
}

}  // namespace

ExtrinsicVisualizeApplication::ExtrinsicVisualizeApplication(ExtrinsicCaptureSettings settings)
    : settings_(std::move(settings)) {}

int ExtrinsicVisualizeApplication::Run(const std::filesystem::path& solution_dir, bool once) {
  try {
    const auto DATA = LoadSolution(solution_dir, settings_);
    const std::filesystem::path CONFIG_ROOT = CONFIG_FILE_PATH;
    const auto CONFIG_PATH = CONFIG_ROOT / "tool/foxglove.yaml";
    const auto FOXGLOVE_ROOT = ConfigLoader::LoadFile(CONFIG_PATH);
    auto foxglove_config = foxglove::ParseConfig(FOXGLOVE_ROOT, CONFIG_PATH);
    foxglove_config.recording.enabled = false;
    foxglove::runtime::FoxgloveSession session(foxglove_config);
    if (!session.LiveConfigured()) {
      throw std::runtime_error("Foxglove live server is disabled by tool/foxglove.yaml");
    }
    auto transforms_result = ::foxglove::schemas::FrameTransformsChannel::create(
        K_TRANSFORMS_TOPIC, session.LiveContext());
    if (!transforms_result.has_value()) {
      throw std::runtime_error("create extrinsic transforms channel: " +
                               std::string(::foxglove::strerror(transforms_result.error())));
    }
    auto scene_result =
        ::foxglove::schemas::SceneUpdateChannel::create(K_SCENE_TOPIC, session.LiveContext());
    if (!scene_result.has_value()) {
      throw std::runtime_error("create extrinsic scene channel: " +
                               std::string(::foxglove::strerror(scene_result.error())));
    }
    auto transforms = std::make_unique<::foxglove::schemas::FrameTransformsChannel>(
        std::move(transforms_result).value());
    auto scene =
        std::make_unique<::foxglove::schemas::SceneUpdateChannel>(std::move(scene_result).value());
    session.RegisterLiveChannel(transforms->id());
    session.RegisterLiveChannel(scene->id());
    session.Start();
    if (!session.LiveActive()) {
      throw std::runtime_error("Foxglove WebSocket server did not start");
    }
    MV_LOG_INFO("ExtrinsicVisualization", "connect Foxglove to ws://{}:{}; transforms={} scene={}",
                foxglove_config.server.host, foxglove_config.server.port, K_TRANSFORMS_TOPIC,
                K_SCENE_TOPIC);
    std::signal(SIGINT, HandleStopSignal);
    std::signal(SIGTERM, HandleStopSignal);
    std::size_t index = 0;
    while (!g_stop_requested) {
      const auto& sample = DATA.samples[index];
      const auto TIMESTAMP = ToTimestamp(sample.capture_timestamp_ns);
      const auto TRANSFORMS = EncodeTransforms(sample, DATA.estimate, TIMESTAMP);
      const auto SCENE = EncodeScene(sample, DATA.estimate, TIMESTAMP, DATA.rotation_mean_deg,
                                     DATA.translation_mean_m);
      {
        std::lock_guard lock(session.PublishMutex());
        const auto TRANSFORM_ERROR = transforms->log(TRANSFORMS, sample.capture_timestamp_ns);
        const auto SCENE_ERROR = scene->log(SCENE, sample.capture_timestamp_ns);
        if (TRANSFORM_ERROR != ::foxglove::FoxgloveError::Ok ||
            SCENE_ERROR != ::foxglove::FoxgloveError::Ok) {
          throw std::runtime_error(
              "publish extrinsic visualization: " +
              std::string(::foxglove::strerror(TRANSFORM_ERROR != ::foxglove::FoxgloveError::Ok
                                                   ? TRANSFORM_ERROR
                                                   : SCENE_ERROR)));
        }
      }
      if (once && index + 1 == DATA.samples.size()) {
        break;
      }
      index = (index + 1) % DATA.samples.size();
      std::this_thread::sleep_for(K_FRAME_PERIOD);
    }
    transforms->close();
    scene->close();
    session.Stop();
    return g_stop_requested ? 6 : 0;
  } catch (const std::exception& error) {
    MV_LOG_ERROR("ExtrinsicVisualization", "{}", error.what());
    return g_stop_requested ? 6 : 5;
  }
}

#else

ExtrinsicVisualizeApplication::ExtrinsicVisualizeApplication(ExtrinsicCaptureSettings settings)
    : settings_(std::move(settings)) {}

int ExtrinsicVisualizeApplication::Run(const std::filesystem::path&, bool) {
  MV_LOG_ERROR("ExtrinsicVisualization", "Foxglove support was not built");
  return 5;
}

#endif

}  // namespace mv::tool::calibration
