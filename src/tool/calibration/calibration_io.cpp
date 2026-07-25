/**
 * @file calibration_io.cpp
 * @brief 标定工具公共 I/O 与图案检测实现
 *
 * 【为什么单独拆分 calibration_io？】
 *   采集、内参标定、手眼标定三个可执行都依赖同一套配置读写和图案检测逻辑。
 *   若逻辑分散在各自 .cpp 中，后续改字段名（如 calibration 节点）时容易遗漏。
 *   统一收敛后可以保证：
 *   1) `vision.yaml` 写回策略一致；
 *   2) 棋盘格/圆点板识别行为一致；
 *   3) 单位约定（mm/m）在模块边界上清晰可控。
 */

#include "tool/calibration/calibration_io.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>

#include <fmt/core.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::tool::calibration {

// ============================================================================
// 内部工具函数
// ============================================================================

namespace {

std::string ToLower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return text;
}

cv::Mat VectorToMat3x3(const std::vector<double>& vals) {
  cv::Mat matrix(3, 3, CV_64F);
  std::memcpy(matrix.data, vals.data(), sizeof(double) * 9);
  return matrix;
}

cv::Mat VectorToRowMat(const std::vector<double>& vals) {
  cv::Mat matrix(1, static_cast<int>(vals.size()), CV_64F);
  std::memcpy(matrix.data, vals.data(), sizeof(double) * vals.size());
  return matrix;
}

}  // namespace

// ============================================================================
// 图案工具
// ============================================================================

PatternType ParsePatternType(const std::string& text) {
  auto normalized = ToLower(text);
  if (normalized == "circles" || normalized == "circle" || normalized == "circles_grid") {
    return PatternType::CIRCLES_GRID;
  }
  return PatternType::CHESSBOARD;
}

std::string PatternTypeToString(PatternType type) {
  return type == PatternType::CIRCLES_GRID ? "circles" : "chessboard";
}

std::vector<cv::Point3f> BuildPatternObjectPoints(const PatternConfig& cfg) {
  std::vector<cv::Point3f> object_points;
  object_points.reserve(static_cast<size_t>(cfg.cols) * static_cast<size_t>(cfg.rows));
  for (int row = 0; row < cfg.rows; ++row) {
    for (int col = 0; col < cfg.cols; ++col) {
      object_points.emplace_back(static_cast<float>(col * cfg.spacing_mm),
                                 static_cast<float>(row * cfg.spacing_mm), 0.0F);
    }
  }
  return object_points;
}

bool DetectPatternCorners(const cv::Mat& image, const PatternConfig& cfg,
                          std::vector<cv::Point2f>* corners, cv::Mat* debug_image) {
  if (!corners || image.empty()) {
    return false;
  }

  corners->clear();
  auto pattern_size = cv::Size(cfg.cols, cfg.rows);

  bool found_success = false;
  if (cfg.type == PatternType::CHESSBOARD) {
    cv::Mat gray;
    if (image.channels() == 3) {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = image;
    }

    found_success = cv::findChessboardCorners(
        gray, pattern_size, *corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (found_success) {
      cv::cornerSubPix(
          gray, *corners, cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
    }
  } else {
    // 圆点板默认使用对称圆点模式。
    found_success = cv::findCirclesGrid(image, pattern_size, *corners, cv::CALIB_CB_SYMMETRIC_GRID);
  }

  if (debug_image) {
    *debug_image = image.clone();
    cv::drawChessboardCorners(*debug_image, pattern_size, *corners, found_success);
  }

  return found_success;
}

// ============================================================================
// YAML 读写
// ============================================================================

std::vector<double> MatToRowMajorVector(const cv::Mat& mat) {
  cv::Mat m64;
  mat.convertTo(m64, CV_64F);
  cv::Mat row = m64.reshape(1, 1);
  return {row.ptr<double>(), row.ptr<double>() + row.cols};
}

bool LoadVisionYaml(const std::string& vision_yaml_path, YAML::Node* root) {
  if (!root) {
    return false;
  }
  try {
    *root = YAML::LoadFile(vision_yaml_path);
    return true;
  } catch (const std::exception& e) {
    fmt::print(stderr, "[calib] load vision yaml failed: {} ({})\n", vision_yaml_path, e.what());
    return false;
  }
}

bool UpdateVisionCalibration(const std::string& vision_yaml_path,
                             const VisionCalibrationUpdate& update) {
  YAML::Node root;
  if (!LoadVisionYaml(vision_yaml_path, &root)) {
    return false;
  }

  YAML::Node calibration_node = root["calibration"];
  if (!calibration_node) {
    calibration_node = root["calibration"] = YAML::Node(YAML::NodeType::Map);
  }

  // 增量写回：仅覆盖提供了值的字段，避免误改其他 calibration 子项。

  if (update.camera_matrix) {
    calibration_node["camera_matrix"] = MatToRowMajorVector(*update.camera_matrix);
  }
  if (update.distort_coeffs) {
    calibration_node["distort_coeffs"] = MatToRowMajorVector(*update.distort_coeffs);
  }
  if (update.r_camera_to_gimbal) {
    calibration_node["R_camera_to_gimbal"] = MatToRowMajorVector(*update.r_camera_to_gimbal);
  }
  if (update.t_camera_to_gimbal) {
    calibration_node["t_camera_to_gimbal"] = MatToRowMajorVector(*update.t_camera_to_gimbal);
  }
  if (update.r_gimbal_to_imu) {
    calibration_node["R_gimbal_to_imu"] = MatToRowMajorVector(*update.r_gimbal_to_imu);
  }

  try {
    std::ofstream out(vision_yaml_path);
    out << root;
    out.close();
    return true;
  } catch (const std::exception& e) {
    fmt::print(stderr, "[calib] write vision yaml failed: {} ({})\n", vision_yaml_path, e.what());
    return false;
  }
}

bool ReadCalibrationNode(const YAML::Node& root, CalibrationData* data) {
  if (data == nullptr || !root || !root["calibration"]) {
    fmt::print(stderr, "[calib] missing calibration node\n");
    return false;
  }

  auto calibration_node = root["calibration"];

  if (!calibration_node["camera_matrix"]) {
    fmt::print(stderr, "[calib] missing calibration.camera_matrix\n");
    return false;
  }
  auto camera_matrix_values = calibration_node["camera_matrix"].as<std::vector<double>>();
  if (camera_matrix_values.size() != 9) {
    fmt::print(stderr, "[calib] camera_matrix size must be 9, got {}\n",
               camera_matrix_values.size());
    return false;
  }
  data->camera_matrix = VectorToMat3x3(camera_matrix_values);

  if (calibration_node["distort_coeffs"]) {
    auto distort_coeff_values = calibration_node["distort_coeffs"].as<std::vector<double>>();
    data->distort_coeffs = VectorToRowMat(distort_coeff_values);
  } else {
    data->distort_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  }

  data->r_gimbal_to_imu = Eigen::Matrix3d::Identity();
  if (calibration_node["R_gimbal_to_imu"]) {
    auto r_gimbal_to_imu_values = calibration_node["R_gimbal_to_imu"].as<std::vector<double>>();
    if (r_gimbal_to_imu_values.size() == 9) {
      data->r_gimbal_to_imu = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          r_gimbal_to_imu_values.data());
    }
  }

  return true;
}

}  // namespace mv::tool::calibration
