#include "tool/calibration/handeye_calibration.hpp"

#include "core/config.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/SVD>
#include <opencv2/calib3d.hpp>

namespace mv::tool::calibration {
namespace {
cv::Mat Matrix(const Eigen::Matrix3d& rotation) {
  cv::Mat result(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) result.at<double>(row, col) = rotation(row, col);
  }
  return result;
}
Eigen::Matrix3d Rotation(const cv::Mat& matrix) {
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) result(row, col) = matrix.at<double>(row, col);
  }
  return result;
}
std::vector<double> Numbers(const YAML::Node& root, const char* key, std::size_t count) {
  auto values = ConfigLoader::Require<std::vector<double>>(root, key, "handeye intrinsics");
  if (values.size() != count) throw ConfigError(std::string(key) + ": wrong length");
  for (double value : values) {
    if (!std::isfinite(value)) throw ConfigError(std::string(key) + ": non-finite value");
  }
  return values;
}
}  // namespace

std::vector<double> HandeyeMatrixValues(const cv::Mat& matrix) {
  return {matrix.begin<double>(), matrix.end<double>()};
}

HandeyeIntrinsics ParseHandeyeIntrinsics(const YAML::Node& root) {
  ConfigLoader::RejectUnknownKeys(root,
      {"schema_version", "width", "height", "roi_offset_x", "roi_offset_y", "camera_matrix",
       "distortion"}, "handeye intrinsics");
  if (ConfigLoader::Require<int>(root, "schema_version", "intrinsics") != 1) {
    throw ConfigError("intrinsics schema_version must be 1");
  }
  HandeyeIntrinsics result;
  result.size = {root["width"].as<int>(), root["height"].as<int>()};
  result.roi_x = root["roi_offset_x"].as<int>();
  result.roi_y = root["roi_offset_y"].as<int>();
  const auto K = Numbers(root, "camera_matrix", 9);
  const auto D = Numbers(root, "distortion", 5);
  if (result.size.width <= 0 || result.size.height <= 0 || result.roi_x < 0 || result.roi_y < 0 ||
      K[0] <= 0 || K[4] <= 0 || K[1] != 0 || K[3] != 0 || K[6] != 0 || K[7] != 0 || K[8] != 1) {
    throw ConfigError("invalid camera size, ROI or pinhole intrinsics");
  }
  result.matrix = cv::Mat(K).reshape(1, 3).clone();
  result.distortion = cv::Mat(D).clone();
  return result;
}

HandeyeCalibrator::HandeyeCalibrator(CalibrationSettings settings, HandeyeIntrinsics intrinsics,
                                   geometry::GimbalExtrinsics extrinsics)
    : settings_(std::move(settings)), intrinsics_(std::move(intrinsics)),
      extrinsics_(std::move(extrinsics)) {
  if (settings_.board_columns < 2 || settings_.board_rows < 2 ||
      !std::isfinite(settings_.square_size_mm) || settings_.square_size_mm <= 0 ||
      settings_.min_samples < 3) throw ConfigError("invalid handeye board or minimum samples");
  const float STEP = static_cast<float>(settings_.square_size_mm / 1000.0);
  for (int row = 0; row < settings_.board_rows; ++row) {
    for (int col = 0; col < settings_.board_columns; ++col) {
      board_points_.emplace_back(col * STEP, row * STEP, 0);
    }
  }
}

bool HandeyeCalibrator::Add(std::size_t id, const std::vector<cv::Point2f>& corners,
                            const geometry::Quaternion& world_q_imu, std::string& failure) {
  if (corners.size() != board_points_.size()) {
    failure = "incomplete_board";
    return false;
  }
  if (!world_q_imu.coeffs().allFinite() || world_q_imu.norm() < 0.5 || world_q_imu.norm() > 1.5) {
    failure = "invalid_imu_quaternion";
    return false;
  }
  for (const auto& point : corners) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      failure = "invalid_corner"; return false;
    }
  }
  try {
    cv::Mat rvec, tvec, rotation;
    if (!cv::solvePnP(board_points_, corners, intrinsics_.matrix, intrinsics_.distortion,
                      rvec, tvec, false, cv::SOLVEPNP_IPPE) ||
        !cv::checkRange(rvec) || !cv::checkRange(tvec)) {
      failure = "pnp_failed"; return false;
    }
    cv::Rodrigues(rvec, rotation);
    // 整块棋盘必须位于相机前方；不能只检查原点的深度。
    for (const auto& point : board_points_) {
      const double DEPTH = rotation.at<double>(2, 0) * point.x +
          rotation.at<double>(2, 1) * point.y + tvec.at<double>(2);
      if (DEPTH <= 0) { failure = "board_behind_camera"; return false; }
    }
    std::vector<cv::Point2f> projected;
    cv::projectPoints(board_points_, rvec, tvec, intrinsics_.matrix,
                      intrinsics_.distortion, projected);
    double sum = 0;
    for (std::size_t i = 0; i < corners.size(); ++i) {
      const auto DELTA = corners[i] - projected[i];
      sum += DELTA.dot(DELTA);
    }
    if (!std::isfinite(sum)) {
      failure = "non_finite_reprojection";
      return false;
    }
    const auto WORLD_T_GIMBAL =
        geometry::ResolveRotationOnlyWorldTGimbal(world_q_imu.normalized(), extrinsics_);
    observations_.push_back({id, rotation, tvec,
        Matrix(WORLD_T_GIMBAL.rotation.toRotationMatrix()), std::sqrt(sum / corners.size())});
    return true;
  } catch (const cv::Exception& error) {
    failure = error.what();
    return false;
  }
}

HandeyeResult HandeyeCalibrator::Solve() const {
  HandeyeResult result;
  auto& report = result.report;
  report["schema_version"] = 1;
  report["method"] = "PARK";
  report["solved"] = false;
  report["manual_validation_required"] = true;
  report["sample_count"] = observations_.size();
  report["minimum_samples"] = settings_.min_samples;
  report["pnp_samples"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& observation : observations_) {
    YAML::Node sample;
    sample["id"] = observation.id;
    sample["reprojection_rms_px"] = observation.reprojection_rms_px;
    sample["camera_r_board"] = HandeyeMatrixValues(observation.camera_r_board);
    sample["camera_t_board_m"] = HandeyeMatrixValues(observation.camera_t_board);
    sample["world_r_gimbal"] = HandeyeMatrixValues(observation.world_r_gimbal);
    report["pnp_samples"].push_back(sample);
  }
  if (observations_.size() < static_cast<std::size_t>(settings_.min_samples)) {
    report["failure"] = "insufficient_samples";
    return result;
  }
  // 相对旋转需激励至少两条不同轴。仅单轴时 (R-I) 的公共零空间使平移不可观测。
  Eigen::Matrix3d rotation_gram = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d translation_gram = Eigen::Matrix3d::Zero();
  YAML::Node motions(YAML::NodeType::Sequence);
  for (std::size_t i = 0; i < observations_.size(); ++i) {
    for (std::size_t j = i + 1; j < observations_.size(); ++j) {
      const Eigen::Matrix3d RELATIVE = Rotation(observations_[i].world_r_gimbal).transpose() *
                                       Rotation(observations_[j].world_r_gimbal);
      const Eigen::AngleAxisd MOTION(RELATIVE);
      if (MOTION.angle() < 0.5 * std::acos(-1.0) / 180.0) continue;
      const Eigen::Vector3d VECTOR = MOTION.axis() * MOTION.angle();
      const Eigen::Matrix3d DIFFERENCE = RELATIVE - Eigen::Matrix3d::Identity();
      rotation_gram += VECTOR * VECTOR.transpose();
      translation_gram += DIFFERENCE.transpose() * DIFFERENCE;
      YAML::Node motion;
      motion["from"] = observations_[i].id;
      motion["to"] = observations_[j].id;
      motion["angle_deg"] = MOTION.angle() * 180.0 / std::acos(-1.0);
      motion["axis_in_gimbal"] = std::vector<double>{MOTION.axis().x(), MOTION.axis().y(),
                                                     MOTION.axis().z()};
      motions.push_back(motion);
    }
  }
  const Eigen::Vector3d RS = rotation_gram.jacobiSvd().singularValues().cwiseSqrt();
  const Eigen::Vector3d TS = translation_gram.jacobiSvd().singularValues().cwiseSqrt();
  report["relative_motions"] = motions;
  report["rotation_singular_values"] = std::vector<double>{RS[0], RS[1], RS[2]};
  report["translation_singular_values"] = std::vector<double>{TS[0], TS[1], TS[2]};
  report["min_relative_angle_deg"] = 0.5;
  report["min_rotation_axis_ratio"] = 0.05;
  report["min_translation_singular_ratio"] = 0.001;
  if (RS[0] <= 1e-12 || TS[0] <= 1e-12 || RS[1] / RS[0] < 0.05 || TS[2] / TS[0] < 0.001) {
    report["failure"] = "degenerate_motion_use_both_yaw_and_pitch";
    return result;
  }
  report["translation_condition_number"] = TS[0] / TS[2];
  std::vector<cv::Mat> rotations, translations, board_rotations, board_translations;
  for (const auto& observation : observations_) {
    rotations.push_back(observation.world_r_gimbal);
    translations.push_back(cv::Mat::zeros(3, 1, CV_64F));
    board_rotations.push_back(observation.camera_r_board);
    board_translations.push_back(observation.camera_t_board);
  }
  try {
    cv::calibrateHandEye(rotations, translations, board_rotations, board_translations,
                        result.gimbal_r_camera, result.gimbal_t_camera, cv::CALIB_HAND_EYE_PARK);
  } catch (const cv::Exception& error) {
    report["failure"] = error.what();
    return result;
  }
  if (!cv::checkRange(result.gimbal_r_camera) || !cv::checkRange(result.gimbal_t_camera) ||
      !geometry::IsRotationMatrix(Rotation(result.gimbal_r_camera))) {
    report["failure"] = "non_finite_or_invalid_handeye_result";
    return result;
  }
  // 固定棋盘的 world 位姿应一致；残差只报告，不用未经实测的门限宣称精度合格。
  std::vector<Eigen::Matrix3d> world_rotations;
  std::vector<Eigen::Vector3d> world_positions;
  Eigen::Matrix3d rotation_sum = Eigen::Matrix3d::Zero();
  Eigen::Vector3d position_mean = Eigen::Vector3d::Zero();
  for (const auto& observation : observations_) {
    const cv::Mat WB = observation.world_r_gimbal * result.gimbal_r_camera *
                       observation.camera_r_board;
    const cv::Mat WP = observation.world_r_gimbal *
        (result.gimbal_r_camera * observation.camera_t_board + result.gimbal_t_camera);
    world_rotations.push_back(Rotation(WB));
    world_positions.emplace_back(WP.at<double>(0), WP.at<double>(1), WP.at<double>(2));
    rotation_sum += world_rotations.back();
    position_mean += world_positions.back();
  }
  position_mean /= static_cast<double>(observations_.size());
  const Eigen::JacobiSVD<Eigen::Matrix3d> SVD(
      rotation_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  correction(2, 2) = (SVD.matrixU() * SVD.matrixV().transpose()).determinant();
  const Eigen::Matrix3d ROTATION_MEAN = SVD.matrixU() * correction * SVD.matrixV().transpose();
  double position_sum = 0, angle_sum = 0, pixel_sum = 0;
  double worst_position = -1, worst_angle = -1, worst_pixel = -1;
  for (std::size_t i = 0; i < observations_.size(); ++i) {
    const double DISTANCE = (world_positions[i] - position_mean).norm();
    const double ANGLE = Eigen::AngleAxisd(ROTATION_MEAN.transpose() * world_rotations[i]).angle() *
                         180.0 / std::acos(-1.0);
    const double PIXEL = observations_[i].reprojection_rms_px;
    position_sum += DISTANCE * DISTANCE;
    angle_sum += ANGLE * ANGLE;
    pixel_sum += PIXEL * PIXEL;
    auto sample = report["pnp_samples"][i];
    sample["world_t_board_m"] = std::vector<double>{world_positions[i].x(),
        world_positions[i].y(), world_positions[i].z()};
    sample["world_r_board"] = HandeyeMatrixValues(Matrix(world_rotations[i]));
    sample["position_residual_m"] = DISTANCE;
    sample["rotation_residual_deg"] = ANGLE;
    if (DISTANCE > worst_position) {
      worst_position = DISTANCE; report["worst_position_sample_id"] = observations_[i].id;
    }
    if (ANGLE > worst_angle) {
      worst_angle = ANGLE; report["worst_rotation_sample_id"] = observations_[i].id;
    }
    if (PIXEL > worst_pixel) {
      worst_pixel = PIXEL; report["worst_reprojection_sample_id"] = observations_[i].id;
    }
  }
  if (!std::isfinite(position_sum) || !std::isfinite(angle_sum) || !std::isfinite(pixel_sum)) {
    report["failure"] = "non_finite_consistency_residual";
    return result;
  }
  report["position_rms_m"] = std::sqrt(position_sum / observations_.size());
  report["rotation_rms_deg"] = std::sqrt(angle_sum / observations_.size());
  report["reprojection_rms_px"] = std::sqrt(pixel_sum / observations_.size());
  report["max_position_residual_m"] = worst_position;
  report["max_rotation_residual_deg"] = worst_angle;
  report["max_reprojection_rms_px"] = worst_pixel;
  report["world_t_board_mean_m"] = std::vector<double>{position_mean.x(), position_mean.y(),
                                                       position_mean.z()};
  report["world_r_board_mean"] = HandeyeMatrixValues(Matrix(ROTATION_MEAN));
  report["gimbal_r_camera"] = HandeyeMatrixValues(result.gimbal_r_camera);
  report["gimbal_t_camera_m"] = HandeyeMatrixValues(result.gimbal_t_camera);
  report["solved"] = true;
  result.solved = true;
  return result;
}
}  // namespace mv::tool::calibration
