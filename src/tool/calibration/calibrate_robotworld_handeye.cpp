/**
 * @file calibrate_robotworld_handeye.cpp
 * @brief RobotWorld-HandEye 外参标定主流程工具
 *
 * 【实现约定】
 *   - 读取 EX calibration 内参后执行 solvePnP + calibrateRobotWorldHandEye；
 *   - 输出并写回 R_camera_to_gimbal / t_camera_to_gimbal（单位 m）；
 *   - 仅更新 EX 的 calibration 字段，不引入额外配置源。
 *
 * 【坐标链路】
 *   图像角点 + 内参 -> solvePnP 得到 board 在 camera 下位姿，
 *   再结合云台姿态（四元数）进入 calibrateRobotWorldHandEye，最终反解出
 *   camera->gimbal 外参，供 PnP 解算模块直接消费。
 */

#include "tool/calibration/calibration_io.hpp"

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <filesystem>
#include <fmt/core.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

namespace {

const std::string K_KEYS =
    "{help h usage ? |                        | Print help.}"
    "{@input-folder  | data/calib_capture     | Folder containing i.jpg and i.txt (w x y z).}"
    "{vision-path v  | src/config/vision.yaml | EX vision.yaml path.}"
    "{pattern-type p | chessboard             | chessboard or circles.}"
    "{cols           | 10                     | Pattern columns.}"
    "{rows           | 7                      | Pattern rows.}"
    "{spacing-mm     | 40.0                   | Pattern center spacing in mm.}"
    "{write          | true                   | Write R/t to EX calibration node.}";

std::vector<int> CollectIndices(const std::string& folder) {
  std::vector<int> ids;
  for (const auto& entry : std::filesystem::directory_iterator(folder)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    auto ext = entry.path().extension().string();
    if (ext != ".jpg" && ext != ".png" && ext != ".jpeg" && ext != ".bmp") {
      continue;
    }
    auto stem = entry.path().stem().string();
    try {
      ids.push_back(std::stoi(stem));
    } catch (...) {
      continue;
    }
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  return ids;
}

bool ReadQuaternionWxyz(const std::string& path, Eigen::Quaterniond* quaternion) {
  if (quaternion == nullptr) {
    return false;
  }
  std::ifstream input(path);
  if (!input.is_open()) {
    return false;
  }
  double quat_w = 1.0;
  double quat_x = 0.0;
  double quat_y = 0.0;
  double quat_z = 0.0;
  input >> quat_w >> quat_x >> quat_y >> quat_z;
  if (!input.good()) {
    return false;
  }
  *quaternion = Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);
  return true;
}

cv::Mat ToCvMat3x3(const Eigen::Matrix3d& rotation_matrix) {
  cv::Mat out;
  cv::eigen2cv(rotation_matrix, out);
  return out;
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
  auto write = cli.get<bool>("write");

  mv::tool::calibration::PatternConfig pattern_cfg;
  pattern_cfg.type = mv::tool::calibration::ParsePatternType(cli.get<std::string>("pattern-type"));
  pattern_cfg.cols = cli.get<int>("cols");
  pattern_cfg.rows = cli.get<int>("rows");
  pattern_cfg.spacing_mm = cli.get<double>("spacing-mm");

  YAML::Node vision_root;
  if (!mv::tool::calibration::LoadVisionYaml(vision_path, &vision_root)) {
    return 1;
  }

  mv::tool::calibration::CalibrationData calibration_data;
  if (!mv::tool::calibration::ReadCalibrationNode(vision_root, &calibration_data)) {
    return 1;
  }
  auto camera_matrix = calibration_data.camera_matrix;
  auto distort_coeffs = calibration_data.distort_coeffs;
  auto r_gimbal_to_imu = calibration_data.r_gimbal_to_imu;

  auto object_points = mv::tool::calibration::BuildPatternObjectPoints(pattern_cfg);
  auto ids = CollectIndices(input_folder);
  if (ids.empty()) {
    fmt::print(stderr, "[calib-rwhandeye] no image indices in {}\n", input_folder);
    return 1;
  }

  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  std::vector<cv::Mat> r_world_to_gimbal_list;
  std::vector<cv::Mat> t_world_to_gimbal_list;

  for (int sample_id : ids) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, sample_id);
    auto txt_path = fmt::format("{}/{}.txt", input_folder, sample_id);

    cv::Mat image = cv::imread(img_path);
    if (image.empty()) {
      continue;
    }

    Eigen::Quaterniond quaternion;
    if (!ReadQuaternionWxyz(txt_path, &quaternion)) {
      fmt::print("[skip] {} (missing or invalid quaternion)\n", txt_path);
      continue;
    }

    std::vector<cv::Point2f> corners;
    auto found_success =
        mv::tool::calibration::DetectPatternCorners(image, pattern_cfg, &corners, nullptr);
    if (!found_success) {
      fmt::print("[skip] {} (pattern not found)\n", img_path);
      continue;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    // 平面标定板优先使用 IPPE，能稳定提供平面目标位姿。
    auto pnp_success = cv::solvePnP(object_points, corners, camera_matrix, distort_coeffs, rvec,
                                    tvec, false, cv::SOLVEPNP_IPPE);
    if (!pnp_success) {
      fmt::print("[skip] {} (solvePnP failed)\n", img_path);
      continue;
    }

    auto r_imubody_to_imuabs = quaternion.toRotationMatrix();
    auto r_gimbal_to_world = r_gimbal_to_imu.transpose() * r_imubody_to_imuabs * r_gimbal_to_imu;
    auto r_world_to_gimbal = r_gimbal_to_world.transpose();

    r_world_to_gimbal_list.push_back(ToCvMat3x3(r_world_to_gimbal));
    t_world_to_gimbal_list.push_back((cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0));
    rvecs.push_back(rvec);
    tvecs.push_back(tvec);

    fmt::print("[ok] sample {}\n", sample_id);
  }

  if (rvecs.size() < 6) {
    fmt::print(stderr, "[calib-rwhandeye] valid samples too few: {} (need >= 6)\n", rvecs.size());
    return 1;
  }

  cv::Mat r_world_to_board;
  cv::Mat t_world_to_board;
  cv::Mat r_gimbal_to_camera;
  cv::Mat t_gimbal_to_camera;

  cv::calibrateRobotWorldHandEye(rvecs, tvecs, r_world_to_gimbal_list, t_world_to_gimbal_list,
                                 r_world_to_board, t_world_to_board, r_gimbal_to_camera,
                                 t_gimbal_to_camera);

  // 输入模板点单位是 mm，因此平移结果也是 mm，这里统一换算到 m。
  t_gimbal_to_camera /= 1000.0;

  cv::Mat r_camera_to_gimbal;
  cv::transpose(r_gimbal_to_camera, r_camera_to_gimbal);
  cv::Mat t_camera_to_gimbal = -r_camera_to_gimbal * t_gimbal_to_camera;

  auto det_r = cv::determinant(r_camera_to_gimbal);
  fmt::print("\n[calib-rwhandeye] samples={} det(R_camera_to_gimbal)={:.6f}\n", rvecs.size(),
             det_r);
  fmt::print(
      "R_camera_to_gimbal: {}\n",
      YAML::Dump(YAML::Node(mv::tool::calibration::MatToRowMajorVector(r_camera_to_gimbal))));
  fmt::print(
      "t_camera_to_gimbal(m): {}\n",
      YAML::Dump(YAML::Node(mv::tool::calibration::MatToRowMajorVector(t_camera_to_gimbal))));

  if (write) {
    cv::Mat r_gimbal_to_imu_cv;
    cv::eigen2cv(r_gimbal_to_imu, r_gimbal_to_imu_cv);
    mv::tool::calibration::VisionCalibrationUpdate update{};
    update.r_camera_to_gimbal = &r_camera_to_gimbal;
    update.t_camera_to_gimbal = &t_camera_to_gimbal;
    update.r_gimbal_to_imu = &r_gimbal_to_imu_cv;
    auto write_success = mv::tool::calibration::UpdateVisionCalibration(vision_path, update);
    if (!write_success) {
      return 1;
    }
    fmt::print("[calib-rwhandeye] wrote extrinsics to {}\n", vision_path);
  }

  return 0;
}
