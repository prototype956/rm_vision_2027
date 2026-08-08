#include "tool/calibration/calibration.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::tool::calibration {
namespace {

constexpr char K_CONFIG_CONTEXT[] = "camera calibration config";

double LengthRatio(double first, double second) {
  const double SMALLER = std::min(first, second);
  return SMALLER > 0.0 ? std::max(first, second) / SMALLER : 1.0;
}

double LaplacianVariance(const cv::Mat& laplacian) {
  cv::Scalar mean;
  cv::Scalar deviation;
  cv::meanStdDev(laplacian, mean, deviation);
  return deviation[0] * deviation[0];
}

std::string CurrentLocalTime() {
  const std::time_t NOW = std::time(nullptr);
  std::tm local_time{};
  localtime_r(&NOW, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%dT%H:%M:%S%z");
  return stream.str();
}

void WriteEmitterAtomically(const std::filesystem::path& path, const YAML::Emitter& emitter) {
  std::filesystem::create_directories(path.parent_path());
  const auto TEMP_PATH = path.string() + ".tmp";
  {
    std::ofstream output(TEMP_PATH, std::ios::trunc);
    if (!output) {
      throw std::runtime_error("cannot open output file: " + TEMP_PATH);
    }
    output << emitter.c_str() << '\n';
    if (!output) {
      throw std::runtime_error("cannot write output file: " + TEMP_PATH);
    }
  }
  std::error_code error;
  // 正常路径用同目录 rename 原子替换；不支持覆盖的文件系统再退化为先删后改名。
  std::filesystem::rename(TEMP_PATH, path, error);
  if (error) {
    std::filesystem::remove(path, error);
    error.clear();
    std::filesystem::rename(TEMP_PATH, path, error);
  }
  if (error) {
    std::filesystem::remove(TEMP_PATH);
    throw std::runtime_error("cannot replace output file '" + path.string() +
                             "': " + error.message());
  }
}

void EmitCameraInfo(YAML::Emitter& output, const hal::CameraInfo& info) {
  output << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "device_name" << YAML::Value << info.device_name;
  output << YAML::Key << "sensor_width" << YAML::Value << info.sensor_width;
  output << YAML::Key << "sensor_height" << YAML::Value << info.sensor_height;
  output << YAML::Key << "output_width" << YAML::Value << info.output_width;
  output << YAML::Key << "output_height" << YAML::Value << info.output_height;
  output << YAML::Key << "roi_offset_x" << YAML::Value << info.roi_offset_x;
  output << YAML::Key << "roi_offset_y" << YAML::Value << info.roi_offset_y;
  output << YAML::Key << "exposure_us" << YAML::Value << info.exposure_us;
  output << YAML::EndMap;
}

void EmitBoard(YAML::Emitter& output, const CalibrationSettings& settings) {
  output << YAML::Key << "board" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "columns" << YAML::Value << settings.board_columns;
  output << YAML::Key << "rows" << YAML::Value << settings.board_rows;
  output << YAML::Key << "square_size_mm" << YAML::Value << settings.square_size_mm;
  output << YAML::EndMap;
}

void EmitQualityCriteria(YAML::Emitter& output, const CalibrationSettings& settings) {
  output << YAML::Key << "quality_criteria" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "min_samples" << YAML::Value << settings.min_samples;
  output << YAML::Key << "min_sharpness" << YAML::Value << settings.min_sharpness;
  output << YAML::Key << "max_rms_px" << YAML::Value << settings.max_rms_px;
  output << YAML::Key << "max_view_rms_px" << YAML::Value << settings.max_view_rms_px;
  output << YAML::Key << "required_grid_cells" << YAML::Value << 9;
  output << YAML::Key << "min_area_ratio" << YAML::Value << settings.min_area_ratio;
  output << YAML::Key << "min_tilted_views_per_axis" << YAML::Value << settings.min_tilted_views;
  output << YAML::Key << "min_tilt_ratio" << YAML::Value << settings.min_tilt_ratio;
  output << YAML::EndMap;
}

void EmitCoverage(YAML::Emitter& output, const CoverageMetrics& coverage) {
  output << YAML::Key << "coverage" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "occupied_grid_cells" << YAML::Value << coverage.occupied_grid_cells;
  output << YAML::Key << "grid_cells" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (const bool OCCUPIED : coverage.grid_cells)
    output << OCCUPIED;
  output << YAML::EndSeq;
  output << YAML::Key << "projected_area_ratio" << YAML::Value << coverage.area_ratio;
  output << YAML::Key << "horizontal_tilted_views" << YAML::Value
         << coverage.horizontal_tilted_views;
  output << YAML::Key << "vertical_tilted_views" << YAML::Value << coverage.vertical_tilted_views;
  output << YAML::EndMap;
}

std::vector<cv::Point3f> MakeBoardPoints(const CalibrationSettings& settings) {
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<std::size_t>(settings.board_columns) *
                 static_cast<std::size_t>(settings.board_rows));
  for (int row = 0; row < settings.board_rows; ++row) {
    for (int column = 0; column < settings.board_columns; ++column) {
      points.emplace_back(static_cast<float>(column * settings.square_size_mm),
                          static_cast<float>(row * settings.square_size_mm), 0.0F);
    }
  }
  return points;
}

}  // namespace

CalibrationSettings ParseCalibrationSettings(const YAML::Node& root,
                                             const std::filesystem::path& project_root) {
  ConfigLoader::RejectUnknownKeys(
      root, {"schema_version", "board", "capture", "quality", "output_dir"}, K_CONFIG_CONTEXT);

  const auto BOARD = root["board"];
  ConfigLoader::RequireMap(BOARD, "camera calibration config.board");
  ConfigLoader::RejectUnknownKeys(BOARD, {"columns", "rows", "square_size_mm"},
                                  "camera calibration config.board");

  const auto CAPTURE = root["capture"];
  ConfigLoader::RequireMap(CAPTURE, "camera calibration config.capture");
  ConfigLoader::RejectUnknownKeys(CAPTURE, {"min_samples", "min_sharpness"},
                                  "camera calibration config.capture");

  const auto QUALITY = root["quality"];
  ConfigLoader::RequireMap(QUALITY, "camera calibration config.quality");
  ConfigLoader::RejectUnknownKeys(
      QUALITY,
      {"max_rms_px", "max_view_rms_px", "min_area_ratio", "min_tilted_views", "min_tilt_ratio"},
      "camera calibration config.quality");

  CalibrationSettings settings;
  settings.board_columns =
      ConfigLoader::Require<int>(BOARD, "columns", "camera calibration config.board");
  settings.board_rows =
      ConfigLoader::Require<int>(BOARD, "rows", "camera calibration config.board");
  settings.square_size_mm =
      ConfigLoader::Require<double>(BOARD, "square_size_mm", "camera calibration config.board");
  settings.min_samples =
      ConfigLoader::Require<int>(CAPTURE, "min_samples", "camera calibration config.capture");
  settings.min_sharpness =
      ConfigLoader::Require<double>(CAPTURE, "min_sharpness", "camera calibration config.capture");
  settings.max_rms_px =
      ConfigLoader::Require<double>(QUALITY, "max_rms_px", "camera calibration config.quality");
  settings.max_view_rms_px = ConfigLoader::Require<double>(QUALITY, "max_view_rms_px",
                                                           "camera calibration config.quality");
  settings.min_area_ratio =
      ConfigLoader::Require<double>(QUALITY, "min_area_ratio", "camera calibration config.quality");
  settings.min_tilted_views =
      ConfigLoader::Require<int>(QUALITY, "min_tilted_views", "camera calibration config.quality");
  settings.min_tilt_ratio =
      ConfigLoader::Require<double>(QUALITY, "min_tilt_ratio", "camera calibration config.quality");
  const auto OUTPUT_DIR = ConfigLoader::Require<std::string>(root, "output_dir", K_CONFIG_CONTEXT);
  settings.output_dir = ConfigLoader::ResolvePath(project_root, OUTPUT_DIR);

  if (settings.board_columns < 2 || settings.board_rows < 2 || settings.square_size_mm <= 0.0 ||
      settings.min_samples < 3 || settings.min_sharpness < 0.0 || settings.max_rms_px <= 0.0 ||
      settings.max_view_rms_px <= 0.0 || settings.min_area_ratio < 1.0 ||
      settings.min_tilted_views < 1 || settings.min_tilt_ratio <= 1.0 || OUTPUT_DIR.empty()) {
    throw ConfigError("camera calibration config contains an invalid setting");
  }
  return settings;
}

CameraCalibrator::CameraCalibrator(CalibrationSettings settings, cv::Size image_size)
    : settings_(std::move(settings)), image_size_(image_size) {
  if (image_size_.width <= 0 || image_size_.height <= 0) {
    throw std::invalid_argument("calibration image size must be positive");
  }
}

FrameObservation CameraCalibrator::Observe(const cv::Mat& bgr_image) const {
  if (bgr_image.empty() || bgr_image.size() != image_size_ || bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("calibration frame must match configured BGR8 image size");
  }

  cv::Mat gray;
  cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
  cv::Mat laplacian;
  cv::Laplacian(gray, laplacian, CV_64F);

  FrameObservation observation;
  observation.sharpness = LaplacianVariance(laplacian);
  observation.sharp_enough = observation.sharpness >= settings_.min_sharpness;
  observation.found = cv::findChessboardCornersSB(
      gray, {settings_.board_columns, settings_.board_rows}, observation.corners,
      cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
  if (!observation.found) {
    observation.corners.clear();
    return observation;
  }

  // 棋盘识别后只在目标区域重新计算清晰度，避免背景纹理抬高整帧方差。
  const cv::Rect BOARD_REGION =
      cv::boundingRect(observation.corners) & cv::Rect(0, 0, image_size_.width, image_size_.height);
  if (BOARD_REGION.area() > 0) {
    observation.sharpness = LaplacianVariance(laplacian(BOARD_REGION));
    observation.sharp_enough = observation.sharpness >= settings_.min_sharpness;
  }

  const int COLUMNS = settings_.board_columns;
  const int ROWS = settings_.board_rows;
  const auto& top_left = observation.corners.front();
  const auto& top_right = observation.corners[static_cast<std::size_t>(COLUMNS - 1)];
  const auto& bottom_left =
      observation.corners[static_cast<std::size_t>(ROWS - 1) * static_cast<std::size_t>(COLUMNS)];
  const auto& bottom_right = observation.corners.back();
  const std::vector<cv::Point2f> OUTER_CORNERS = {top_left, top_right, bottom_right, bottom_left};
  observation.projected_area = std::abs(cv::contourArea(OUTER_CORNERS));

  // 透视投影会压缩远侧边，对边长度比可低成本区分横纵两个方向的倾斜姿态。
  const double TOP_LENGTH = cv::norm(top_right - top_left);
  const double BOTTOM_LENGTH = cv::norm(bottom_right - bottom_left);
  const double LEFT_LENGTH = cv::norm(bottom_left - top_left);
  const double RIGHT_LENGTH = cv::norm(bottom_right - top_right);
  observation.horizontal_tilt_ratio = LengthRatio(LEFT_LENGTH, RIGHT_LENGTH);
  observation.vertical_tilt_ratio = LengthRatio(TOP_LENGTH, BOTTOM_LENGTH);
  return observation;
}

bool CameraCalibrator::IsLikelyDuplicate(const FrameObservation& observation) const {
  if (!observation.found || observation.corners.empty())
    return false;
  const cv::Point2f OBSERVATION_CENTER =
      (observation.corners.front() + observation.corners.back()) * 0.5F;
  const double IMAGE_DIAGONAL = std::hypot(image_size_.width, image_size_.height);
  // 同时约束归一化位置、尺度和双向倾斜，提示近似姿态但不阻止用户主动采集。
  for (const auto& sample : samples_) {
    if (!sample.active || sample.corners.empty() || sample.projected_area <= 0.0)
      continue;
    const cv::Point2f SAMPLE_CENTER = (sample.corners.front() + sample.corners.back()) * 0.5F;
    const double CENTER_DISTANCE = cv::norm(OBSERVATION_CENTER - SAMPLE_CENTER) / IMAGE_DIAGONAL;
    const double AREA_RATIO = LengthRatio(observation.projected_area, sample.projected_area);
    const double HORIZONTAL_DELTA =
        std::abs(observation.horizontal_tilt_ratio - sample.horizontal_tilt_ratio);
    const double VERTICAL_DELTA =
        std::abs(observation.vertical_tilt_ratio - sample.vertical_tilt_ratio);
    if (CENTER_DISTANCE < 0.05 && AREA_RATIO < 1.10 && HORIZONTAL_DELTA < 0.05 &&
        VERTICAL_DELTA < 0.05) {
      return true;
    }
  }
  return false;
}

bool CameraCalibrator::AddSample(const FrameObservation& observation,
                                 const std::filesystem::path& image_path) {
  const std::size_t EXPECTED_CORNERS = static_cast<std::size_t>(settings_.board_columns) *
                                       static_cast<std::size_t>(settings_.board_rows);
  if (!observation.found || !observation.sharp_enough ||
      observation.corners.size() != EXPECTED_CORNERS || image_path.empty()) {
    return false;
  }
  samples_.push_back({.id = next_sample_id_++,
                      .image_path = image_path,
                      .active = true,
                      .sharpness = observation.sharpness,
                      .projected_area = observation.projected_area,
                      .horizontal_tilt_ratio = observation.horizontal_tilt_ratio,
                      .vertical_tilt_ratio = observation.vertical_tilt_ratio,
                      .corners = observation.corners});
  return true;
}

std::optional<std::size_t> CameraCalibrator::UndoLastSample() {
  for (auto sample = samples_.rbegin(); sample != samples_.rend(); ++sample) {
    if (sample->active) {
      sample->active = false;
      return sample->id;
    }
  }
  return std::nullopt;
}

CoverageMetrics CameraCalibrator::AnalyzeCoverage(
    const std::vector<const CalibrationSample*>& samples) const {
  CoverageMetrics metrics;
  double minimum_area = std::numeric_limits<double>::max();
  double maximum_area = 0.0;
  for (const auto* sample : samples) {
    minimum_area = std::min(minimum_area, sample->projected_area);
    maximum_area = std::max(maximum_area, sample->projected_area);
    if (sample->horizontal_tilt_ratio >= settings_.min_tilt_ratio) {
      ++metrics.horizontal_tilted_views;
    }
    if (sample->vertical_tilt_ratio >= settings_.min_tilt_ratio) {
      ++metrics.vertical_tilted_views;
    }
    // 使用全部内角点而非棋盘中心统计覆盖，要求观测信息延伸到画面每个区域。
    for (const auto& corner : sample->corners) {
      const int COLUMN = std::clamp(static_cast<int>(static_cast<double>(corner.x) * 3.0 /
                                                     static_cast<double>(image_size_.width)),
                                    0, 2);
      const int ROW = std::clamp(static_cast<int>(static_cast<double>(corner.y) * 3.0 /
                                                  static_cast<double>(image_size_.height)),
                                 0, 2);
      const auto INDEX = static_cast<std::size_t>(ROW) * 3U + static_cast<std::size_t>(COLUMN);
      metrics.grid_cells[INDEX] = true;
    }
  }
  metrics.occupied_grid_cells =
      static_cast<int>(std::count(metrics.grid_cells.begin(), metrics.grid_cells.end(), true));
  if (minimum_area > 0.0 && minimum_area != std::numeric_limits<double>::max()) {
    metrics.area_ratio = maximum_area / minimum_area;
  }
  return metrics;
}

CalibrationResult CameraCalibrator::Solve() const {
  CalibrationResult result;
  std::vector<const CalibrationSample*> active_samples;
  for (const auto& sample : samples_) {
    if (sample.active)
      active_samples.push_back(&sample);
  }
  result.coverage = AnalyzeCoverage(active_samples);

  // 覆盖条件即使不满足也继续求解，便于操作者同时看到误差和待补采的姿态。
  if (active_samples.size() < static_cast<std::size_t>(settings_.min_samples)) {
    result.failures.push_back("active sample count is below min_samples");
  }
  if (result.coverage.occupied_grid_cells < 9) {
    result.failures.push_back("chessboard corners do not cover all 3x3 image cells");
  }
  if (result.coverage.area_ratio < settings_.min_area_ratio) {
    result.failures.push_back("projected board area ratio is below min_area_ratio");
  }
  if (result.coverage.horizontal_tilted_views < settings_.min_tilted_views) {
    result.failures.push_back("not enough horizontally tilted views");
  }
  if (result.coverage.vertical_tilted_views < settings_.min_tilted_views) {
    result.failures.push_back("not enough vertically tilted views");
  }
  if (active_samples.size() < 3) {
    return result;
  }

  const auto BOARD_POINTS = MakeBoardPoints(settings_);
  std::vector<std::vector<cv::Point3f>> object_points(active_samples.size(), BOARD_POINTS);
  std::vector<std::vector<cv::Point2f>> image_points;
  image_points.reserve(active_samples.size());
  for (const auto* sample : active_samples) {
    image_points.push_back(sample->corners);
    result.active_sample_ids.push_back(sample->id);
  }

  std::vector<cv::Mat> rotation_vectors;
  std::vector<cv::Mat> translation_vectors;
  result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  result.distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
  try {
    result.rms_px = cv::calibrateCamera(object_points, image_points, image_size_,
                                        result.camera_matrix, result.distortion_coefficients,
                                        rotation_vectors, translation_vectors, 0,
                                        {cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                                         std::numeric_limits<double>::epsilon()});
  } catch (const cv::Exception& error) {
    result.failures.push_back("OpenCV calibration failed: " + std::string(error.what()));
    return result;
  }
  result.solved = true;
  // OpenCV 可能返回列向量或更多模型系数，输出格式固定为 plumb_bob 五参数行向量。
  result.distortion_coefficients =
      result.distortion_coefficients.reshape(1, 1).colRange(0, 5).clone();

  // 单独重投影每个视图，定位会被全局 RMS 掩盖的离群样本。
  for (std::size_t index = 0; index < active_samples.size(); ++index) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(BOARD_POINTS, rotation_vectors[index], translation_vectors[index],
                      result.camera_matrix, result.distortion_coefficients, projected);
    const double VIEW_RMS = cv::norm(active_samples[index]->corners, projected, cv::NORM_L2) /
                            std::sqrt(static_cast<double>(projected.size()));
    result.per_view_rms_px.push_back(VIEW_RMS);
    if (VIEW_RMS > result.max_view_rms_px) {
      result.max_view_rms_px = VIEW_RMS;
      result.worst_sample_id = active_samples[index]->id;
    }
  }
  if (result.rms_px > settings_.max_rms_px) {
    result.failures.push_back("global RMS exceeds max_rms_px");
  }
  if (result.max_view_rms_px > settings_.max_view_rms_px) {
    result.failures.push_back("a per-view RMS exceeds max_view_rms_px");
  }
  result.accepted = result.failures.empty();
  return result;
}

const CalibrationSettings& CameraCalibrator::Settings() const noexcept {
  return settings_;
}

const std::vector<CalibrationSample>& CameraCalibrator::Samples() const noexcept {
  return samples_;
}

std::size_t CameraCalibrator::ActiveSampleCount() const noexcept {
  return static_cast<std::size_t>(
      std::count_if(samples_.begin(), samples_.end(),
                    [](const CalibrationSample& sample) { return sample.active; }));
}

void WriteSession(const std::filesystem::path& path, const CalibrationSettings& settings,
                  const hal::CameraInfo& camera_info, const std::vector<CalibrationSample>& samples,
                  const std::optional<CalibrationResult>& result, const std::string& status) {
  YAML::Emitter output;
  output.SetDoublePrecision(12);
  output << YAML::BeginMap;
  output << YAML::Key << "schema_version" << YAML::Value << 1;
  output << YAML::Key << "updated_at" << YAML::Value << CurrentLocalTime();
  output << YAML::Key << "status" << YAML::Value << status;
  EmitCameraInfo(output, camera_info);
  EmitBoard(output, settings);
  EmitQualityCriteria(output, settings);
  output << YAML::Key << "samples" << YAML::Value << YAML::BeginSeq;
  for (const auto& sample : samples) {
    output << YAML::BeginMap;
    output << YAML::Key << "id" << YAML::Value << sample.id;
    output << YAML::Key << "active" << YAML::Value << sample.active;
    output << YAML::Key << "image" << YAML::Value << sample.image_path.generic_string();
    output << YAML::Key << "sharpness" << YAML::Value << sample.sharpness;
    output << YAML::Key << "projected_area" << YAML::Value << sample.projected_area;
    output << YAML::Key << "horizontal_tilt_ratio" << YAML::Value << sample.horizontal_tilt_ratio;
    output << YAML::Key << "vertical_tilt_ratio" << YAML::Value << sample.vertical_tilt_ratio;
    output << YAML::Key << "corners" << YAML::Value << YAML::BeginSeq;
    for (const auto& corner : sample.corners) {
      output << YAML::Flow << YAML::BeginSeq << corner.x << corner.y << YAML::EndSeq;
    }
    output << YAML::EndSeq << YAML::EndMap;
  }
  output << YAML::EndSeq;
  if (result) {
    output << YAML::Key << "result" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "solved" << YAML::Value << result->solved;
    output << YAML::Key << "accepted" << YAML::Value << result->accepted;
    output << YAML::Key << "rms_px" << YAML::Value << result->rms_px;
    output << YAML::Key << "max_view_rms_px" << YAML::Value << result->max_view_rms_px;
    output << YAML::Key << "worst_sample_id" << YAML::Value << result->worst_sample_id;
    EmitCoverage(output, result->coverage);
    output << YAML::Key << "per_view_errors" << YAML::Value << YAML::BeginSeq;
    for (std::size_t index = 0; index < result->per_view_rms_px.size(); ++index) {
      output << YAML::BeginMap << YAML::Key << "sample_id" << YAML::Value
             << result->active_sample_ids[index] << YAML::Key << "rms_px" << YAML::Value
             << result->per_view_rms_px[index] << YAML::EndMap;
    }
    output << YAML::EndSeq;
    output << YAML::Key << "failures" << YAML::Value << YAML::BeginSeq;
    for (const auto& failure : result->failures)
      output << failure;
    output << YAML::EndSeq << YAML::EndMap;
  }
  output << YAML::EndMap;
  WriteEmitterAtomically(path, output);
}

void WriteIntrinsics(const std::filesystem::path& path, const CalibrationSettings& settings,
                     const hal::CameraInfo& camera_info, const CalibrationResult& result) {
  if (!result.accepted || result.camera_matrix.empty() ||
      result.distortion_coefficients.total() != 5) {
    throw std::invalid_argument("only an accepted five-parameter calibration can be written");
  }
  YAML::Emitter output;
  output.SetDoublePrecision(15);
  output << YAML::BeginMap;
  output << YAML::Key << "schema_version" << YAML::Value << 1;
  output << YAML::Key << "generated_at" << YAML::Value << CurrentLocalTime();
  output << YAML::Key << "camera_model" << YAML::Value << "pinhole";
  EmitCameraInfo(output, camera_info);
  output << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      output << result.camera_matrix.at<double>(row, column);
    }
  }
  output << YAML::EndSeq;
  output << YAML::Key << "distortion_model" << YAML::Value << "plumb_bob";
  output << YAML::Key << "distortion_coefficients" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int index = 0; index < 5; ++index) {
    output << result.distortion_coefficients.at<double>(0, index);
  }
  output << YAML::EndSeq;
  EmitBoard(output, settings);
  EmitQualityCriteria(output, settings);
  output << YAML::Key << "calibration" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "sample_count" << YAML::Value << result.active_sample_ids.size();
  output << YAML::Key << "rms_px" << YAML::Value << result.rms_px;
  output << YAML::Key << "max_view_rms_px" << YAML::Value << result.max_view_rms_px;
  output << YAML::Key << "worst_sample_id" << YAML::Value << result.worst_sample_id;
  output << YAML::EndMap << YAML::EndMap;
  WriteEmitterAtomically(path, output);
}

}  // namespace mv::tool::calibration
