/**
 * @file validate_calibration.cpp
 * @brief 标定结果校验工具（det/正交性/重投影统计）
 *
 * 【职责边界】
 *   - 读取 vision.yaml 的 calibration 节点，校验外参矩阵几何合法性；
 *   - 在标定样本集上计算重投影误差统计；
 *   - 输出 PASS/FAIL，供 CI 或现场流程做闭环验收。
 */

#include "tool/calibration/calibration_io.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <filesystem>
#include <fmt/core.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

namespace {

const std::string K_KEYS =
    "{help h usage ? |                        | Print help.}"
    "{@input-folder  | data/calib_capture     | Folder with calibration images.}"
    "{vision-path v  | src/config/vision.yaml | EX vision.yaml path.}"
    "{pattern-type p | chessboard             | chessboard or circles.}"
    "{cols           | 10                     | Pattern columns.}"
    "{rows           | 7                      | Pattern rows.}"
    "{spacing-mm     | 40.0                   | Pattern center spacing in mm.}"
    "{max-reproj-px  | 1.5                    | Max allowed mean reprojection error (px).}"
    "{det-eps        | 0.05                   | Allowed |det(R)-1| tolerance.}"
    "{ortho-eps      | 0.05                   | Allowed ||R^T*R-I||_F tolerance.}";

std::vector<std::string> CollectImagePaths(const std::string& folder) {
  std::vector<std::string> paths;
  for (const auto& entry : std::filesystem::directory_iterator(folder)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    auto ext = entry.path().extension().string();
    if (ext == ".jpg" || ext == ".png" || ext == ".jpeg" || ext == ".bmp") {
      paths.push_back(entry.path().string());
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

double MeanValue(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  double sum = std::accumulate(values.begin(), values.end(), 0.0);
  return sum / static_cast<double>(values.size());
}

double MedianValue(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  size_t mid = values.size() / 2;
  if (values.size() % 2 == 0) {
    return 0.5 * (values[mid - 1] + values[mid]);
  }
  return values[mid];
}

bool TryReadRcameraToGimbal(const YAML::Node& root, Eigen::Matrix3d* rotation) {
  if (rotation == nullptr || !root || !root["calibration"] ||
      !root["calibration"]["R_camera_to_gimbal"]) {
    return false;
  }
  auto values = root["calibration"]["R_camera_to_gimbal"].as<std::vector<double>>();
  if (values.size() != 9) {
    return false;
  }
  *rotation = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(values.data());
  return true;
}

double ComputePerFrameReprojectionError(const std::vector<cv::Point3f>& object_points,
                                        const std::vector<cv::Point2f>& image_points,
                                        const cv::Mat& camera_matrix,
                                        const cv::Mat& distort_coeffs) {
  cv::Mat rvec;
  cv::Mat tvec;
  bool pnp_success = cv::solvePnP(object_points, image_points, camera_matrix, distort_coeffs, rvec,
                                  tvec, false, cv::SOLVEPNP_IPPE);
  if (!pnp_success) {
    return -1.0;
  }

  std::vector<cv::Point2f> reproj;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, distort_coeffs, reproj);
  if (reproj.size() != image_points.size() || reproj.empty()) {
    return -1.0;
  }

  double error_sum = 0.0;
  for (size_t index = 0; index < reproj.size(); ++index) {
    error_sum += cv::norm(reproj[index] - image_points[index]);
  }
  return error_sum / static_cast<double>(reproj.size());
}

}  // namespace

int main(int argc, char* argv[]) {
  cv::CommandLineParser cli(argc, argv, K_KEYS);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto input_folder = cli.get<std::string>(0);
  auto vision_path = cli.get<std::string>("vision-path");
  auto max_reproj_px = cli.get<double>("max-reproj-px");
  auto det_eps = cli.get<double>("det-eps");
  auto ortho_eps = cli.get<double>("ortho-eps");

  mv::tool::calibration::PatternConfig pattern_cfg;
  pattern_cfg.type = mv::tool::calibration::ParsePatternType(cli.get<std::string>("pattern-type"));
  pattern_cfg.cols = cli.get<int>("cols");
  pattern_cfg.rows = cli.get<int>("rows");
  pattern_cfg.spacing_mm = cli.get<double>("spacing-mm");

  YAML::Node root;
  if (!mv::tool::calibration::LoadVisionYaml(vision_path, &root)) {
    return 1;
  }

  mv::tool::calibration::CalibrationData calibration_data;
  if (!mv::tool::calibration::ReadCalibrationNode(root, &calibration_data)) {
    return 1;
  }

  bool matrix_check_pass = true;
  Eigen::Matrix3d r_camera_to_gimbal = Eigen::Matrix3d::Identity();
  if (TryReadRcameraToGimbal(root, &r_camera_to_gimbal)) {
    double det_r = r_camera_to_gimbal.determinant();
    double det_err = std::abs(det_r - 1.0);
    double ortho_err =
        (r_camera_to_gimbal.transpose() * r_camera_to_gimbal - Eigen::Matrix3d::Identity()).norm();

    fmt::print("[validate] det(R_camera_to_gimbal)={:.6f}, |det-1|={:.6f}\n", det_r, det_err);
    fmt::print("[validate] orthogonality ||R^T*R-I||_F={:.6f}\n", ortho_err);

    if (det_err > det_eps || ortho_err > ortho_eps) {
      matrix_check_pass = false;
    }
  } else {
    fmt::print("[validate] R_camera_to_gimbal missing, skip matrix geometry checks\n");
  }

  auto image_paths = CollectImagePaths(input_folder);
  if (image_paths.empty()) {
    fmt::print(stderr, "[validate] no images in {}\n", input_folder);
    return 1;
  }

  auto object_template = mv::tool::calibration::BuildPatternObjectPoints(pattern_cfg);
  std::vector<double> per_frame_errors;

  for (const auto& path : image_paths) {
    cv::Mat image = cv::imread(path);
    if (image.empty()) {
      continue;
    }

    std::vector<cv::Point2f> corners;
    bool detect_success =
        mv::tool::calibration::DetectPatternCorners(image, pattern_cfg, &corners, nullptr);
    if (!detect_success) {
      fmt::print("[skip] {} (pattern not found)\n", path);
      continue;
    }

    double frame_error = ComputePerFrameReprojectionError(
        object_template, corners, calibration_data.camera_matrix, calibration_data.distort_coeffs);
    if (frame_error < 0.0) {
      fmt::print("[skip] {} (PnP/reprojection failed)\n", path);
      continue;
    }
    per_frame_errors.push_back(frame_error);
  }

  if (per_frame_errors.empty()) {
    fmt::print(stderr, "[validate] no valid frames for reprojection stats\n");
    return 1;
  }

  double mean_error = MeanValue(per_frame_errors);
  double median_error = MedianValue(per_frame_errors);
  double max_error = *std::max_element(per_frame_errors.begin(), per_frame_errors.end());

  fmt::print("[validate] reprojection mean={:.4f}px median={:.4f}px max={:.4f}px frames={}\n",
             mean_error, median_error, max_error, per_frame_errors.size());

  bool reproj_pass = mean_error <= max_reproj_px;
  bool all_pass = matrix_check_pass && reproj_pass;

  fmt::print("[validate] RESULT: {}\n", all_pass ? "PASS" : "FAIL");
  return all_pass ? 0 : 2;
}
