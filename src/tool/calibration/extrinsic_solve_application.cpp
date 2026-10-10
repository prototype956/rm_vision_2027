#include "tool/calibration/extrinsic_solve_application.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "tool/debug/debug_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <fmt/format.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace mv::tool::calibration {
namespace {

constexpr char K_WINDOW_NAME[] = "MiracleVision Extrinsic Calibration Review";
constexpr char K_CONFIG_CONTEXT[] = "extrinsic calibration session";
constexpr double K_ROTATION_EPSILON = 1.0e-6;
constexpr double K_PI = 3.14159265358979323846;
volatile std::sig_atomic_t g_stop_requested = 0;

void HandleStopSignal(int) noexcept {
  g_stop_requested = 1;
}

class SolveCancelled final : public std::runtime_error {
 public:
  explicit SolveCancelled(const std::string& message) : std::runtime_error(message) {}
};

struct SessionSample {
  std::size_t id{0};
  std::filesystem::path image_path;
  std::uint64_t frame_sequence{0};
  std::uint64_t capture_timestamp_ns{0};
  std::uint64_t simulation_round_id{0};
  frame::CameraModel camera;
  geometry::RigidTransform world_t_gimbal;
  geometry::RigidTransform gimbal_t_camera_optical;
};

struct SessionData {
  std::filesystem::path session_dir;
  std::string status;
  std::vector<SessionSample> samples;
};

struct ObservationRecord {
  SessionSample sample;
  bool detected{false};
  bool accepted{false};
  std::string failure;
  std::optional<geometry::RigidTransform> board_t_camera;
  double pnp_reprojection_rms_px{0.0};
  double rotation_error_deg{0.0};
  double translation_error_m{0.0};
};

struct ErrorStats {
  double rotation_mean_deg{0.0};
  double rotation_rms_deg{0.0};
  double rotation_max_deg{0.0};
  double translation_mean_m{0.0};
  double translation_rms_m{0.0};
  double translation_max_m{0.0};
};

struct SolveReportData {
  std::string status{"solving"};
  std::string error;
  std::filesystem::path session_dir;
  std::vector<ObservationRecord> records;
  std::optional<geometry::RigidTransform> estimate;
  std::optional<geometry::RigidTransform> truth_reference;
  std::size_t truth_reference_id{0};
  ErrorStats error_stats;
};

std::string SessionTimestamp() {
  const auto NOW = std::chrono::system_clock::now();
  const std::time_t TIME = std::chrono::system_clock::to_time_t(NOW);
  std::tm local_time{};
  localtime_r(&TIME, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::filesystem::path CreateSolutionDirectory(const std::filesystem::path& solutions_root) {
  std::filesystem::create_directories(solutions_root);
  const std::string BASE_NAME = SessionTimestamp();
  for (int suffix = 0; suffix < 1000; ++suffix) {
    const std::string NAME = suffix == 0 ? BASE_NAME : fmt::format("{}_{:02d}", BASE_NAME, suffix);
    auto directory = solutions_root / NAME;
    std::error_code error;
    if (std::filesystem::create_directory(directory, error))
      return directory;
    if (error && error != std::errc::file_exists) {
      throw std::filesystem::filesystem_error("cannot create extrinsic solution directory",
                                              directory, error);
    }
  }
  throw std::runtime_error("cannot allocate a unique extrinsic solution directory");
}

void WriteEmitterAtomically(const std::filesystem::path& path, const YAML::Emitter& emitter) {
  if (!emitter.good())
    throw std::runtime_error("cannot serialize extrinsic result: " + emitter.GetLastError());
  std::filesystem::create_directories(path.parent_path());
  const auto TEMP_PATH = path.string() + ".tmp";
  {
    std::ofstream output(TEMP_PATH, std::ios::trunc);
    if (!output)
      throw std::runtime_error("cannot open output file: " + TEMP_PATH);
    output << emitter.c_str() << '\n';
    output.close();
    if (!output)
      throw std::runtime_error("cannot write output file: " + TEMP_PATH);
  }
  std::error_code error;
  std::filesystem::rename(TEMP_PATH, path, error);
  if (error) {
    std::filesystem::remove(TEMP_PATH);
    throw std::runtime_error("cannot replace output file '" + path.string() +
                             "': " + error.message());
  }
}

void DrawText(cv::Mat& image, const std::vector<std::string>& lines, const cv::Scalar& color) {
  int y = 28;
  for (const auto& line : lines) {
    cv::putText(image, line, {14, y}, cv::FONT_HERSHEY_SIMPLEX, 0.58, {0, 0, 0}, 3, cv::LINE_AA);
    cv::putText(image, line, {14, y}, cv::FONT_HERSHEY_SIMPLEX, 0.58, color, 1, cv::LINE_AA);
    y += 25;
  }
}

template <typename T>
T RequireNode(const YAML::Node& node, const std::string& key) {
  return ConfigLoader::Require<T>(node, key, K_CONFIG_CONTEXT);
}

std::vector<double> RequireSequence(const YAML::Node& node, const std::string& key,
                                    std::size_t expected_size) {
  const auto VALUE = node[key];
  if (!VALUE || !VALUE.IsSequence() || VALUE.size() != expected_size) {
    throw ConfigError(std::string(K_CONFIG_CONTEXT) + "." + key + " must be a sequence of size " +
                      std::to_string(expected_size));
  }
  std::vector<double> result;
  result.reserve(expected_size);
  try {
    for (const auto& item : VALUE)
      result.push_back(item.as<double>());
  } catch (const std::exception& error) {
    throw ConfigError(std::string(K_CONFIG_CONTEXT) + "." + key +
                      " contains an invalid number: " + error.what());
  }
  return result;
}

bool IsFiniteTransform(const geometry::RigidTransform& value) {
  return value.translation.allFinite() && value.rotation.coeffs().allFinite() &&
         std::abs(value.rotation.norm() - 1.0) <= K_ROTATION_EPSILON;
}

geometry::RigidTransform ReadTransform(const YAML::Node& node, const std::string& key) {
  const auto TRANSFORM = node[key];
  ConfigLoader::RequireMap(TRANSFORM, std::string(K_CONFIG_CONTEXT) + "." + key);
  ConfigLoader::RejectUnknownKeys(TRANSFORM, {"translation", "rotation_xyzw"},
                                  std::string(K_CONFIG_CONTEXT) + "." + key);
  const auto TRANSLATION = RequireSequence(TRANSFORM, "translation", 3);
  const auto ROTATION = RequireSequence(TRANSFORM, "rotation_xyzw", 4);
  geometry::RigidTransform result;
  result.translation = geometry::Vector3(TRANSLATION[0], TRANSLATION[1], TRANSLATION[2]);
  result.rotation = geometry::Quaternion(ROTATION[3], ROTATION[0], ROTATION[1], ROTATION[2]);
  if (!IsFiniteTransform(result))
    throw ConfigError(std::string(K_CONFIG_CONTEXT) + "." + key + " contains an invalid transform");
  return result;
}

frame::CameraModel ReadCamera(const YAML::Node& node) {
  ConfigLoader::RequireMap(node, std::string(K_CONFIG_CONTEXT) + ".camera");
  ConfigLoader::RejectUnknownKeys(node, {"width", "height", "fx", "fy", "cx", "cy", "distortion"},
                                  std::string(K_CONFIG_CONTEXT) + ".camera");
  const auto DISTORTION = RequireSequence(node, "distortion", 5);
  frame::CameraModel result;
  result.width = RequireNode<std::uint32_t>(node, "width");
  result.height = RequireNode<std::uint32_t>(node, "height");
  result.fx = RequireNode<double>(node, "fx");
  result.fy = RequireNode<double>(node, "fy");
  result.cx = RequireNode<double>(node, "cx");
  result.cy = RequireNode<double>(node, "cy");
  std::copy(DISTORTION.begin(), DISTORTION.end(), result.distortion.begin());
  if (result.width == 0 || result.height == 0 || !std::isfinite(result.fx) ||
      !std::isfinite(result.fy) || !std::isfinite(result.cx) || !std::isfinite(result.cy) ||
      result.fx <= 0.0 || result.fy <= 0.0 ||
      !std::all_of(result.distortion.begin(), result.distortion.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw ConfigError(std::string(K_CONFIG_CONTEXT) +
                      ".camera contains invalid calibration values");
  }
  return result;
}

bool SameCamera(const frame::CameraModel& lhs, const frame::CameraModel& rhs) {
  constexpr double EPSILON = 1.0e-9;
  return lhs.width == rhs.width && lhs.height == rhs.height &&
         std::abs(lhs.fx - rhs.fx) <= EPSILON && std::abs(lhs.fy - rhs.fy) <= EPSILON &&
         std::abs(lhs.cx - rhs.cx) <= EPSILON && std::abs(lhs.cy - rhs.cy) <= EPSILON &&
         std::equal(lhs.distortion.begin(), lhs.distortion.end(), rhs.distortion.begin(),
                    [](double left, double right) { return std::abs(left - right) <= EPSILON; });
}

bool IsSafeRelativePath(const std::filesystem::path& path) {
  if (path.empty() || path.is_absolute())
    return false;
  const auto NORMALIZED = path.lexically_normal();
  const auto TEXT = NORMALIZED.generic_string();
  return TEXT != ".." && !TEXT.starts_with("../");
}

SessionData LoadSession(const std::filesystem::path& input_session_dir,
                        const ExtrinsicCaptureSettings& settings) {
  const auto SESSION_DIR = std::filesystem::absolute(input_session_dir).lexically_normal();
  const auto SESSION_PATH = SESSION_DIR / "session.yaml";
  if (!std::filesystem::is_regular_file(SESSION_PATH))
    throw ConfigError("extrinsic calibration session.yaml not found: " + SESSION_PATH.string());

  YAML::Node root;
  try {
    root = YAML::LoadFile(SESSION_PATH.string());
  } catch (const std::exception& error) {
    throw ConfigError("cannot load extrinsic calibration session: " + std::string(error.what()));
  }
  ConfigLoader::RequireMap(root, K_CONFIG_CONTEXT);
  ConfigLoader::RejectUnknownKeys(root,
                                  {"schema_version", "status", "error", "translation_unit",
                                   "transform_convention", "source", "board", "samples"},
                                  K_CONFIG_CONTEXT);
  if (RequireNode<int>(root, "schema_version") != 1)
    throw ConfigError(std::string(K_CONFIG_CONTEXT) + " schema_version must be 1");
  const auto STATUS = RequireNode<std::string>(root, "status");
  if (STATUS == "collecting")
    throw ConfigError("cannot solve a session that is still collecting");
  if (STATUS != "captured" && STATUS != "aborted")
    throw ConfigError("unsupported extrinsic calibration session status: " + STATUS);
  if (RequireNode<std::string>(root, "translation_unit") != "m" ||
      RequireNode<std::string>(root, "transform_convention") != "parent_t_child") {
    throw ConfigError("session transform units or convention are unsupported");
  }
  const auto SOURCE = root["source"];
  ConfigLoader::RequireMap(SOURCE, std::string(K_CONFIG_CONTEXT) + ".source");
  ConfigLoader::RejectUnknownKeys(SOURCE, {"backend", "device_name"},
                                  std::string(K_CONFIG_CONTEXT) + ".source");
  if (RequireNode<std::string>(SOURCE, "backend") != "talos" ||
      RequireNode<std::string>(SOURCE, "device_name").empty()) {
    throw ConfigError("session source metadata is incomplete or unsupported");
  }

  const auto BOARD = root["board"];
  ConfigLoader::RequireMap(BOARD, std::string(K_CONFIG_CONTEXT) + ".board");
  const int PATTERN_COLS = RequireNode<int>(BOARD, "pattern_cols");
  const int PATTERN_ROWS = RequireNode<int>(BOARD, "pattern_rows");
  const double SQUARE_SIZE_MM = RequireNode<double>(BOARD, "square_size_mm");
  if (PATTERN_COLS != settings.board_columns || PATTERN_ROWS != settings.board_rows ||
      std::abs(SQUARE_SIZE_MM - settings.square_size_mm) > 1.0e-9) {
    throw ConfigError("session board parameters do not match extrinsic calibration config");
  }

  const auto SAMPLES = root["samples"];
  if (!SAMPLES || !SAMPLES.IsSequence())
    throw ConfigError(std::string(K_CONFIG_CONTEXT) + ".samples must be a sequence");

  SessionData result;
  result.session_dir = SESSION_DIR;
  result.status = STATUS;
  for (const auto& item : SAMPLES) {
    if (!item.IsMap() || !item["active"] || !item["active"].as<bool>())
      continue;
    ConfigLoader::RejectUnknownKeys(
        item,
        {"id", "active", "image", "frame_sequence", "simulation_round_id", "capture_timestamp_ns",
         "camera", "world_t_gimbal", "gimbal_t_camera_optical"},
        std::string(K_CONFIG_CONTEXT) + ".samples[]");
    SessionSample sample;
    sample.id = RequireNode<std::size_t>(item, "id");
    const auto IMAGE = std::filesystem::path(RequireNode<std::string>(item, "image"));
    if (!IsSafeRelativePath(IMAGE))
      throw ConfigError("active sample image path must be relative to the session directory");
    sample.image_path = IMAGE.lexically_normal();
    sample.frame_sequence = RequireNode<std::uint64_t>(item, "frame_sequence");
    sample.simulation_round_id = RequireNode<std::uint64_t>(item, "simulation_round_id");
    sample.capture_timestamp_ns = RequireNode<std::uint64_t>(item, "capture_timestamp_ns");
    if (sample.id == 0 || sample.frame_sequence == 0 || sample.capture_timestamp_ns == 0)
      throw ConfigError("active sample has an invalid id, frame sequence or timestamp");
    sample.camera = ReadCamera(item["camera"]);
    sample.world_t_gimbal = ReadTransform(item, "world_t_gimbal");
    sample.gimbal_t_camera_optical = ReadTransform(item, "gimbal_t_camera_optical");
    if (sample.camera.width != static_cast<std::uint32_t>(settings.expected_width) ||
        sample.camera.height != static_cast<std::uint32_t>(settings.expected_height)) {
      throw ConfigError("active sample camera dimensions do not match expected dimensions");
    }
    const auto IMAGE_PATH = SESSION_DIR / sample.image_path;
    if (!std::filesystem::is_regular_file(IMAGE_PATH))
      throw ConfigError("active sample image not found: " + IMAGE_PATH.string());
    result.samples.push_back(std::move(sample));
  }

  std::sort(result.samples.begin(), result.samples.end(),
            [](const SessionSample& lhs, const SessionSample& rhs) {
              return lhs.image_path.generic_string() < rhs.image_path.generic_string();
            });
  if (result.samples.size() < static_cast<std::size_t>(settings.solve_min_samples))
    throw ConfigError("active sample count is below the configured minimum");
  const auto& reference_camera = result.samples.front().camera;
  const auto ROUND_ID = result.samples.front().simulation_round_id;
  for (const auto& sample : result.samples) {
    if (!SameCamera(sample.camera, reference_camera))
      throw ConfigError("active samples contain inconsistent camera models");
    if (sample.simulation_round_id != ROUND_ID)
      throw ConfigError("active samples contain multiple simulation rounds");
  }
  return result;
}

cv::Mat CameraMatrix(const frame::CameraModel& camera) {
  return (cv::Mat_<double>(3, 3) << camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0,
          1.0);
}

cv::Mat Distortion(const frame::CameraModel& camera) {
  return (cv::Mat_<double>(1, 5) << camera.distortion[0], camera.distortion[1],
          camera.distortion[2], camera.distortion[3], camera.distortion[4]);
}

cv::Mat RotationMatrix(const geometry::Quaternion& quaternion) {
  cv::Mat result;
  const Eigen::Matrix3d MATRIX = quaternion.toRotationMatrix();
  cv::eigen2cv(MATRIX, result);
  return result;
}

cv::Mat TranslationVector(const geometry::Vector3& translation) {
  return (cv::Mat_<double>(3, 1) << translation.x(), translation.y(), translation.z());
}

struct OpenCvPose {
  cv::Mat rotation;
  cv::Mat translation;
};

geometry::RigidTransform TransformFromOpenCv(const OpenCvPose& pose) {
  Eigen::Matrix3d matrix;
  cv::cv2eigen(pose.rotation, matrix);
  Eigen::Vector3d vector;
  cv::cv2eigen(pose.translation, vector);
  geometry::RigidTransform result;
  result.translation = vector;
  result.rotation = geometry::Quaternion(matrix).normalized();
  if (!IsFiniteTransform(result))
    throw std::runtime_error("OpenCV returned an invalid hand-eye transform");
  return result;
}

std::vector<cv::Point3f> MakeBoardPoints(const ExtrinsicCaptureSettings& settings) {
  const float SQUARE_SIZE_M = static_cast<float>(settings.square_size_mm / 1000.0);
  std::vector<cv::Point3f> result;
  result.reserve(static_cast<std::size_t>(settings.board_columns) *
                 static_cast<std::size_t>(settings.board_rows));
  for (int row = 0; row < settings.board_rows; ++row) {
    for (int column = 0; column < settings.board_columns; ++column) {
      result.emplace_back(static_cast<float>(column) * SQUARE_SIZE_M,
                          static_cast<float>(row) * SQUARE_SIZE_M, 0.0F);
    }
  }
  return result;
}

double ReprojectionRms(const std::vector<cv::Point2f>& observed,
                       const std::vector<cv::Point2f>& projected) {
  if (observed.size() != projected.size() || observed.empty())
    return std::numeric_limits<double>::infinity();
  double squared_error = 0.0;
  for (std::size_t index = 0; index < observed.size(); ++index) {
    const cv::Point2f ERROR = observed[index] - projected[index];
    squared_error +=
        static_cast<double>(ERROR.x) * ERROR.x + static_cast<double>(ERROR.y) * ERROR.y;
  }
  return std::sqrt(squared_error / static_cast<double>(observed.size()));
}

double RotationErrorDegrees(const geometry::Quaternion& estimated,
                            const geometry::Quaternion& truth) {
  const Eigen::Matrix3d ERROR = (estimated * truth.conjugate()).normalized().toRotationMatrix();
  const double COS_ANGLE = std::clamp((ERROR.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(COS_ANGLE) * 180.0 / K_PI;
}

ErrorStats CalculateErrorStats(const std::vector<ObservationRecord>& records) {
  ErrorStats result;
  std::size_t count = 0;
  double rotation_squared = 0.0;
  double translation_squared = 0.0;
  for (const auto& record : records) {
    if (!record.accepted)
      continue;
    ++count;
    result.rotation_mean_deg += record.rotation_error_deg;
    result.translation_mean_m += record.translation_error_m;
    rotation_squared += record.rotation_error_deg * record.rotation_error_deg;
    translation_squared += record.translation_error_m * record.translation_error_m;
    result.rotation_max_deg = std::max(result.rotation_max_deg, record.rotation_error_deg);
    result.translation_max_m = std::max(result.translation_max_m, record.translation_error_m);
  }
  if (count > 0) {
    const double COUNT = static_cast<double>(count);
    result.rotation_mean_deg /= COUNT;
    result.translation_mean_m /= COUNT;
    result.rotation_rms_deg = std::sqrt(rotation_squared / COUNT);
    result.translation_rms_m = std::sqrt(translation_squared / COUNT);
  }
  return result;
}

void EmitVector3(YAML::Emitter& output, const geometry::Vector3& value) {
  output << YAML::Flow << YAML::BeginSeq << value.x() << value.y() << value.z() << YAML::EndSeq;
}

void EmitQuaternion(YAML::Emitter& output, const geometry::Quaternion& value) {
  output << YAML::Flow << YAML::BeginSeq << value.x() << value.y() << value.z() << value.w()
         << YAML::EndSeq;
}

void EmitRotationMatrix(YAML::Emitter& output, const geometry::Quaternion& value) {
  const Eigen::Matrix3d MATRIX = value.toRotationMatrix();
  output << YAML::Flow << YAML::BeginSeq;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column)
      output << MATRIX(row, column);
  }
  output << YAML::EndSeq;
}

void EmitTransform(YAML::Emitter& output, const geometry::RigidTransform& value) {
  output << YAML::BeginMap << YAML::Key << "translation" << YAML::Value;
  EmitVector3(output, value.translation);
  output << YAML::Key << "rotation_xyzw" << YAML::Value;
  EmitQuaternion(output, value.rotation);
  output << YAML::Key << "rotation_matrix" << YAML::Value;
  EmitRotationMatrix(output, value.rotation);
  output << YAML::EndMap;
}

void WriteExtrinsics(const std::filesystem::path& path, const std::string& source_session,
                     const geometry::RigidTransform& estimate) {
  YAML::Emitter output;
  output.SetDoublePrecision(15);
  output << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value << 1 << YAML::Key
         << "translation_unit" << YAML::Value << "m" << YAML::Key << "transform_convention"
         << YAML::Value << "parent_t_child" << YAML::Key << "target" << YAML::Value
         << "gimbal_t_camera_optical" << YAML::Key << "source_session" << YAML::Value
         << source_session << YAML::Key << "gimbal_t_camera_optical" << YAML::Value;
  EmitTransform(output, estimate);
  output << YAML::EndMap;
  WriteEmitterAtomically(path, output);
}

void WriteReport(const std::filesystem::path& path, const ExtrinsicCaptureSettings& settings,
                 const SolveReportData& report) {
  YAML::Emitter output;
  output.SetDoublePrecision(15);
  output << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value << 1 << YAML::Key
         << "status" << YAML::Value << report.status << YAML::Key << "source_session" << YAML::Value
         << report.session_dir.string();
  if (!report.error.empty())
    output << YAML::Key << "error" << YAML::Value << report.error;
  output << YAML::Key << "board" << YAML::Value << YAML::BeginMap << YAML::Key << "pattern_cols"
         << YAML::Value << settings.board_columns << YAML::Key << "pattern_rows" << YAML::Value
         << settings.board_rows << YAML::Key << "square_size_mm" << YAML::Value
         << settings.square_size_mm << YAML::EndMap;

  if (!report.records.empty()) {
    const auto& camera = report.records.front().sample.camera;
    output << YAML::Key << "camera" << YAML::Value << YAML::BeginMap << YAML::Key << "width"
           << YAML::Value << camera.width << YAML::Key << "height" << YAML::Value << camera.height
           << YAML::Key << "fx" << YAML::Value << camera.fx << YAML::Key << "fy" << YAML::Value
           << camera.fy << YAML::Key << "cx" << YAML::Value << camera.cx << YAML::Key << "cy"
           << YAML::Value << camera.cy << YAML::Key << "distortion" << YAML::Value << YAML::Flow
           << YAML::BeginSeq;
    for (const double VALUE : camera.distortion)
      output << VALUE;
    output << YAML::EndSeq << YAML::EndMap;
  }

  std::size_t accepted_count = 0;
  std::size_t detected_count = 0;
  for (const auto& record : report.records) {
    detected_count += record.detected ? 1 : 0;
    accepted_count += record.accepted ? 1 : 0;
  }
  output << YAML::Key << "sample_counts" << YAML::Value << YAML::BeginMap << YAML::Key << "active"
         << YAML::Value << report.records.size() << YAML::Key << "detected" << YAML::Value
         << detected_count << YAML::Key << "accepted" << YAML::Value << accepted_count
         << YAML::EndMap;

  output << YAML::Key << "samples" << YAML::Value << YAML::BeginSeq;
  for (const auto& record : report.records) {
    const auto& sample = record.sample;
    output << YAML::BeginMap << YAML::Key << "id" << YAML::Value << sample.id << YAML::Key
           << "image" << YAML::Value << sample.image_path.generic_string() << YAML::Key
           << "frame_sequence" << YAML::Value << sample.frame_sequence << YAML::Key
           << "simulation_round_id" << YAML::Value << sample.simulation_round_id << YAML::Key
           << "capture_timestamp_ns" << YAML::Value << sample.capture_timestamp_ns << YAML::Key
           << "detected" << YAML::Value << record.detected << YAML::Key << "accepted" << YAML::Value
           << record.accepted << YAML::Key << "pnp_reprojection_rms_px" << YAML::Value
           << record.pnp_reprojection_rms_px;
    if (!record.failure.empty())
      output << YAML::Key << "failure" << YAML::Value << record.failure;
    output << YAML::Key << "world_t_gimbal" << YAML::Value;
    EmitTransform(output, sample.world_t_gimbal);
    output << YAML::Key << "gimbal_t_camera_optical_truth" << YAML::Value;
    EmitTransform(output, sample.gimbal_t_camera_optical);
    if (record.accepted) {
      if (record.board_t_camera) {
        output << YAML::Key << "board_t_camera" << YAML::Value;
        EmitTransform(output, *record.board_t_camera);
      }
      output << YAML::Key << "rotation_error_deg" << YAML::Value << record.rotation_error_deg
             << YAML::Key << "translation_error_m" << YAML::Value << record.translation_error_m;
    }
    output << YAML::EndMap;
  }
  output << YAML::EndSeq;

  if (report.estimate) {
    output << YAML::Key << "estimated_gimbal_t_camera_optical" << YAML::Value;
    EmitTransform(output, *report.estimate);
  }
  if (report.truth_reference) {
    output << YAML::Key << "truth_reference_sample_id" << YAML::Value << report.truth_reference_id
           << YAML::Key << "truth_reference_gimbal_t_camera_optical" << YAML::Value;
    EmitTransform(output, *report.truth_reference);
  }
  output << YAML::Key << "error_statistics" << YAML::Value << YAML::BeginMap << YAML::Key
         << "rotation_mean_deg" << YAML::Value << report.error_stats.rotation_mean_deg << YAML::Key
         << "rotation_rms_deg" << YAML::Value << report.error_stats.rotation_rms_deg << YAML::Key
         << "rotation_max_deg" << YAML::Value << report.error_stats.rotation_max_deg << YAML::Key
         << "translation_mean_m" << YAML::Value << report.error_stats.translation_mean_m
         << YAML::Key << "translation_rms_m" << YAML::Value << report.error_stats.translation_rms_m
         << YAML::Key << "translation_max_m" << YAML::Value << report.error_stats.translation_max_m
         << YAML::EndMap << YAML::EndMap;
  WriteEmitterAtomically(path, output);
}

}  // namespace

ExtrinsicSolveApplication::ExtrinsicSolveApplication(ExtrinsicCaptureSettings settings)
    : settings_(std::move(settings)) {}

int ExtrinsicSolveApplication::Run(const std::filesystem::path& input_session_dir) {
  g_stop_requested = 0;
  std::signal(SIGINT, HandleStopSignal);
  std::signal(SIGTERM, HandleStopSignal);

  SessionData session;
  std::filesystem::path result_dir;
  try {
    session = LoadSession(input_session_dir, settings_);
    result_dir = CreateSolutionDirectory(session.session_dir / "solutions");
  } catch (const std::exception& error) {
    const auto SESSION_DIR = std::filesystem::absolute(input_session_dir).lexically_normal();
    const auto FAILURE_DIR = CreateSolutionDirectory(SESSION_DIR / "solutions");
    SolveReportData report;
    report.session_dir = SESSION_DIR;
    report.status = "failed";
    report.error = error.what();
    WriteReport(FAILURE_DIR / "report.yaml", settings_, report);
    return 5;
  }
  const auto& session_data = session;
  const auto& result_directory = result_dir;
  SolveReportData report;
  report.session_dir = session_data.session_dir;
  report.records.reserve(session_data.samples.size());
  for (const auto& sample : session_data.samples) {
    ObservationRecord record;
    record.sample = sample;
    report.records.push_back(std::move(record));
  }

  try {
    const auto BOARD_POINTS = MakeBoardPoints(settings_);
    std::vector<cv::Mat> r_gimbal_world;
    std::vector<cv::Mat> t_gimbal_world;
    std::vector<cv::Mat> r_board_camera;
    std::vector<cv::Mat> t_board_camera;
    r_gimbal_world.reserve(report.records.size());
    t_gimbal_world.reserve(report.records.size());
    r_board_camera.reserve(report.records.size());
    t_board_camera.reserve(report.records.size());

    tool::DebugWindow review_window(K_WINDOW_NAME, tool::WindowMode::NORMAL);
    for (auto& record : report.records) {
      if (g_stop_requested != 0)
        throw SolveCancelled("extrinsic solve interrupted by signal");
      const auto IMAGE_PATH = session_data.session_dir / record.sample.image_path;
      const auto IMAGE = cv::imread(IMAGE_PATH.string(), cv::IMREAD_COLOR);
      if (IMAGE.empty() || IMAGE.cols != settings_.expected_width ||
          IMAGE.rows != settings_.expected_height) {
        throw std::runtime_error("unreadable image or size mismatch: " + IMAGE_PATH.string());
      }

      std::vector<cv::Point2f> corners;
      record.detected = cv::findChessboardCornersSB(
          IMAGE, {settings_.board_columns, settings_.board_rows}, corners,
          cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
      if (!record.detected)
        record.failure = "chessboard was not detected";

      cv::Mat preview = IMAGE.clone();
      cv::drawChessboardCorners(preview, {settings_.board_columns, settings_.board_rows}, corners,
                                record.detected);
      DrawText(preview,
               {fmt::format("sample {}  {}/{}", record.sample.id, report.records.size(),
                            report.records.size()),
                record.detected ? "Board found" : "Board NOT found - will be skipped",
                "Any key: confirm   Q/Esc: cancel"},
               record.detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 180, 255));
      review_window.Show(preview);

      bool confirmed = false;
      while (g_stop_requested == 0) {
        const auto EVENT = review_window.Poll(30);
        if (EVENT.exit_requested)
          throw SolveCancelled("extrinsic solve review cancelled");
        if (EVENT.key >= 0) {
          confirmed = true;
          break;
        }
      }
      if (!confirmed)
        throw SolveCancelled("extrinsic solve interrupted by signal");
      if (!record.detected)
        continue;

      cv::Mat rvec;
      cv::Mat tvec;
      const auto CAMERA_MATRIX = CameraMatrix(record.sample.camera);
      const auto DISTORTION = Distortion(record.sample.camera);
      if (!cv::solvePnP(BOARD_POINTS, corners, CAMERA_MATRIX, DISTORTION, rvec, tvec, false,
                        cv::SOLVEPNP_IPPE)) {
        record.failure = "solvePnP returned false";
        continue;
      }
      cv::Mat rotation;
      cv::Rodrigues(rvec, rotation);
      std::vector<cv::Point2f> projected;
      cv::projectPoints(BOARD_POINTS, rvec, tvec, CAMERA_MATRIX, DISTORTION, projected);
      record.pnp_reprojection_rms_px = ReprojectionRms(corners, projected);
      if (!std::isfinite(record.pnp_reprojection_rms_px)) {
        record.failure = "solvePnP produced an invalid reprojection error";
        continue;
      }
      record.board_t_camera = TransformFromOpenCv({.rotation = rotation, .translation = tvec});
      record.accepted = true;
      r_gimbal_world.push_back(RotationMatrix(record.sample.world_t_gimbal.rotation));
      t_gimbal_world.push_back(TranslationVector(record.sample.world_t_gimbal.translation));
      r_board_camera.push_back(rotation);
      t_board_camera.push_back(tvec);
    }
    review_window.Close();

    if (r_board_camera.size() < static_cast<std::size_t>(settings_.solve_min_samples)) {
      report.status = "failed";
      report.error = "valid chessboard observations are below the configured minimum";
      WriteReport(result_directory / "report.yaml", settings_, report);
      return 5;
    }

    cv::Mat rotation_camera_gimbal;
    cv::Mat translation_camera_gimbal;
    cv::calibrateHandEye(r_gimbal_world, t_gimbal_world, r_board_camera, t_board_camera,
                         rotation_camera_gimbal, translation_camera_gimbal,
                         cv::CALIB_HAND_EYE_TSAI);
    const auto ESTIMATE = TransformFromOpenCv(
        {.rotation = rotation_camera_gimbal, .translation = translation_camera_gimbal});
    report.estimate = ESTIMATE;

    for (auto& record : report.records) {
      if (!record.accepted)
        continue;
      record.rotation_error_deg =
          RotationErrorDegrees(ESTIMATE.rotation, record.sample.gimbal_t_camera_optical.rotation);
      record.translation_error_m =
          (ESTIMATE.translation - record.sample.gimbal_t_camera_optical.translation).norm();
    }
    const auto REFERENCE =
        std::find_if(report.records.begin(), report.records.end(),
                     [](const ObservationRecord& record) { return record.accepted; });
    report.truth_reference = REFERENCE->sample.gimbal_t_camera_optical;
    report.truth_reference_id = REFERENCE->sample.id;
    report.error_stats = CalculateErrorStats(report.records);
    report.status = "solved";
    WriteExtrinsics(result_directory / "extrinsics.yaml", session_data.session_dir.string(),
                    ESTIMATE);
    WriteReport(result_directory / "report.yaml", settings_, report);
    MV_LOG_INFO("ExtrinsicCalibration",
                "solved {} samples; rotation RMS={:.6f} deg, "
                "translation RMS={:.6f} m; result={}",
                r_board_camera.size(), report.error_stats.rotation_rms_deg,
                report.error_stats.translation_rms_m, result_directory.string());
    return 0;
  } catch (const SolveCancelled& error) {
    report.status = "aborted";
    report.error = error.what();
    WriteReport(result_directory / "report.yaml", settings_, report);
    return 4;
  } catch (const std::exception& error) {
    report.status = "failed";
    report.error = error.what();
    WriteReport(result_directory / "report.yaml", settings_, report);
    MV_LOG_ERROR("ExtrinsicCalibration", "solve failed: {}", error.what());
    return 5;
  }
}

}  // namespace mv::tool::calibration
