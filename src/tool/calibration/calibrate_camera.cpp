/**
 * @file calibrate_camera.cpp
 * @brief 相机内参标定工具（兼容棋盘格与圆点板）
 *
 * 【实现约定】
 *   - 使用 OpenCV calibrateCamera 输出 K 与畸变系数；
 *   - 写回配置时仅更新 EX 的 calibration.camera_matrix / distort_coeffs 字段，
 *     其余节点保持不变。
 *
 * 【输出含义】
 *   - `camera_matrix`：当前输入分辨率下的内参矩阵；
 *   - `distort_coeffs`：OpenCV 畸变系数，默认允许 k1/k2/p1/p2（k3 可按参数固定）。
 */

#include "tool/calibration/calibration_io.hpp"

#include <algorithm>
#include <cfloat>
#include <string>
#include <vector>

#include <filesystem>
#include <fmt/core.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
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
    "{fix-k3         | true                   | Use CALIB_FIX_K3.}"
    "{show           | false                  | Show detection image per frame.}"
    "{write          | true                   | Write camera_matrix/distort_coeffs to EX "
    "calibration node.}";

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

double ComputeMeanReprojectionError(const std::vector<std::vector<cv::Point3f>>& object_points,
                                    const std::vector<std::vector<cv::Point2f>>& image_points,
                                    const std::vector<cv::Mat>& rvecs,
                                    const std::vector<cv::Mat>& tvecs, const cv::Mat& camera_matrix,
                                    const cv::Mat& dist_coeffs) {
  // 使用逐点欧氏距离平均值，作为快速可读的重投影误差指标。
  double error_sum = 0.0;
  size_t total = 0;
  for (size_t i = 0; i < object_points.size(); ++i) {
    std::vector<cv::Point2f> reproj;
    cv::projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, reproj);
    total += reproj.size();
    for (size_t j = 0; j < reproj.size(); ++j) {
      error_sum += cv::norm(image_points[i][j] - reproj[j]);
    }
  }
  return total == 0 ? 0.0 : (error_sum / static_cast<double>(total));
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
  auto fix_k3 = cli.get<bool>("fix-k3");
  auto show = cli.get<bool>("show");
  auto write = cli.get<bool>("write");

  mv::tool::calibration::PatternConfig pattern_cfg;
  pattern_cfg.type = mv::tool::calibration::ParsePatternType(cli.get<std::string>("pattern-type"));
  pattern_cfg.cols = cli.get<int>("cols");
  pattern_cfg.rows = cli.get<int>("rows");
  pattern_cfg.spacing_mm = cli.get<double>("spacing-mm");

  auto image_paths = CollectImagePaths(input_folder);
  if (image_paths.empty()) {
    fmt::print(stderr, "[calib-camera] no images in {}\n", input_folder);
    return 1;
  }

  auto object_template = mv::tool::calibration::BuildPatternObjectPoints(pattern_cfg);
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;
  cv::Size image_size;

  for (const auto& path : image_paths) {
    cv::Mat img = cv::imread(path);
    if (img.empty()) {
      continue;
    }
    image_size = img.size();

    std::vector<cv::Point2f> corners;
    cv::Mat debug;
    auto detect_success =
        mv::tool::calibration::DetectPatternCorners(img, pattern_cfg, &corners, &debug);
    fmt::print("[{}] {}\n", detect_success ? "ok" : "skip", path);
    if (!detect_success) {
      continue;
    }

    object_points.push_back(object_template);
    image_points.push_back(corners);

    if (show) {
      cv::imshow("mv-calib-camera", debug);
      cv::waitKey(100);
    }
  }

  if (object_points.size() < 5) {
    fmt::print(stderr, "[calib-camera] too few valid images: {}\n", object_points.size());
    return 1;
  }

  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  int flags = 0;
  if (fix_k3) {
    flags |= cv::CALIB_FIX_K3;
  }

  cv::calibrateCamera(
      object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs, flags,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON));

  auto err = ComputeMeanReprojectionError(object_points, image_points, rvecs, tvecs, camera_matrix,
                                          dist_coeffs);

  fmt::print("\n[calib-camera] pattern={} valid_images={} reproj_error={:.4f}px\n",
             mv::tool::calibration::PatternTypeToString(pattern_cfg.type), object_points.size(),
             err);
  fmt::print(
      "camera_matrix: {}\n",
      fmt::format(
          "{}", YAML::Dump(YAML::Node(mv::tool::calibration::MatToRowMajorVector(camera_matrix)))));
  fmt::print(
      "distort_coeffs: {}\n",
      fmt::format("{}",
                  YAML::Dump(YAML::Node(mv::tool::calibration::MatToRowMajorVector(dist_coeffs)))));

  if (write) {
    mv::tool::calibration::VisionCalibrationUpdate update{};
    update.camera_matrix = &camera_matrix;
    update.distort_coeffs = &dist_coeffs;
    auto write_success = mv::tool::calibration::UpdateVisionCalibration(vision_path, update);
    if (!write_success) {
      return 1;
    }
    fmt::print("[calib-camera] wrote calibration intrinsics to {}\n", vision_path);
  }

  return 0;
}
