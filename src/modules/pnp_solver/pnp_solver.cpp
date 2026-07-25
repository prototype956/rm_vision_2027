/**
 * @file pnp_solver.cpp
 * @brief PnP 位姿解算器实现（单解 IPPE + 外参变换）
 *
 * 【算法说明（当前版本）】
 *   1. SOLVEPNP_IPPE：装甲板为平面目标，IPPE 给出解析解，比 LM 迭代
 *      快 3~5×，且无初始值依赖，精度相当。
 *   2. 外参变换：可从 calibration.R_camera2gimbal / t_camera2gimbal 加载
 *      标定外参，实现精确相机→云台坐标系转换。不提供外参时退化为 Y 轴
 *      翻转（兼容旧行为）。
 *   3. yaw/pitch 从云台坐标系（非原始相机坐标）计算，误差更小。
 */
#include "pnp_solver.hpp"

#include "core/logger.hpp"
#include "factory/factory.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::modules {

namespace {
const bool PNP_SOLVER_REGISTERED = [] {
  ::mv::Factory<::mv::ISolver>::Register("pnp", [] { return std::make_unique<PnpSolver>(); });
  return true;
}();
}  // namespace

// ── 装甲板坐标模板（静态，仅构建一次）──────────────────────────────────────────

/** 构建装甲板坐标点（BL, BR, TR, TL，Z=0，装甲板平面朝相机）*/
static std::vector<cv::Point3f> MakeWorldPoints(float half_w, float half_h) {
  return {
      {-half_w, -half_h, 0.0F},  // BL
      {half_w, -half_h, 0.0F},   // BR
      {half_w, half_h, 0.0F},    // TR
      {-half_w, half_h, 0.0F},   // TL
  };
}

// ── 构造 / 析构 ────────────────────────────────────────────────────────────

PnpSolver::PnpSolver() = default;
PnpSolver::~PnpSolver() = default;

// ── 云台→世界坐标变换接口实现 ─────────────────────────────────────────────

/**
 * @brief 设定云台→世界坐标系旋转变换（通常由 IMU 四元数提供）
 * @param quaternion 全局绝对方向四元数
 * @thread_safety Not thread-safe
 */
void PnpSolver::SetGimbalToWorldRotation(const Eigen::Quaterniond& quaternion) {
  Eigen::Matrix3d r_imubody2imuabsolute = quaternion.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * r_imubody2imuabsolute * R_gimbal2imubody_;
}

/**
 * @brief 查询云台→世界坐标系旋转矩阵
 * @return 当前云台→世界旋转矩阵
 * @thread_safety Not thread-safe
 */
Eigen::Matrix3d PnpSolver::GetGimbalToWorldRotation() const {
  return R_gimbal2world_;
}

/**
 * @brief 设定云台→世界坐标系平移向量（通常由 IMU 姿态与安装位姿联合计算）
 * @param quaternion 全局绝对方向四元数
 * @thread_safety Not thread-safe
 */
void PnpSolver::SetGimbalToWorldTranslation(const Eigen::Quaterniond& quaternion) {
  Eigen::Matrix3d r_imubody2imuabsolute = quaternion.toRotationMatrix();
  t_gimbal2world_ = r_imubody2imuabsolute * t_gimbal2imubody_;
}

/**
 * @brief 同步更新云台到世界坐标系的旋转与平移缓存
 * @param quaternion 全局绝对方向四元数
 * @thread_safety Not thread-safe
 */
void PnpSolver::SetGimbalOrientation(const Eigen::Quaterniond& quaternion) {
  SetGimbalToWorldRotation(quaternion);
  SetGimbalToWorldTranslation(quaternion);
}

/**
 * @brief 查询云台→世界坐标系平移向量
 * @return 云台在世界坐标系下的 3x1 位置向量（单位：m）
 * @thread_safety Not thread-safe
 */
Eigen::Vector3d PnpSolver::GetGimbalToWorldTranslation() const {
  return t_gimbal2world_;
}

// ── Init ──────────────────────────────────────────────────────────────────

bool PnpSolver::Init(const YAML::Node& config) {
  if (!config || !config["calibration"]) {
    MV_LOG_ERROR("PnpSolver", "Init failed: missing 'calibration' node in config");
    return false;
  }

  const auto& calib = config["calibration"];

  // ── 内参：camera_matrix（必须）──────────────────────────────────────────
  if (!calib["camera_matrix"]) {
    MV_LOG_ERROR("PnpSolver", "Init failed: missing calibration.camera_matrix");
    return false;
  }
  auto cm_vals = calib["camera_matrix"].as<std::vector<double>>();
  if (cm_vals.size() != 9) {
    MV_LOG_ERROR("PnpSolver", "Init failed: camera_matrix must have 9 values, got {}",
                 cm_vals.size());
    return false;
  }
  camera_matrix_ = cv::Mat(3, 3, CV_64F, cm_vals.data()).clone();

  // ── 内参：distort_coeffs（可选）─────────────────────────────────────────
  if (!calib["distort_coeffs"]) {
    MV_LOG_WARN("PnpSolver", "calibration.distort_coeffs not found, using zero distortion");
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
  } else {
    auto dc_vals = calib["distort_coeffs"].as<std::vector<double>>();
    dist_coeffs_ = cv::Mat(1, static_cast<int>(dc_vals.size()), CV_64F, dc_vals.data()).clone();
  }

  // ── 外参：R_camera2gimbal（可选，3×3 行优先）────────────────────────────
  if (calib["R_camera_to_gimbal"]) {
    auto r_vals = calib["R_camera_to_gimbal"].as<std::vector<double>>();
    if (r_vals.size() == 9) {
      R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(r_vals.data());
    } else {
      MV_LOG_WARN("PnpSolver", "R_camera_to_gimbal must have 9 values, got {} — ignored",
                  r_vals.size());
    }
  }

  // ── 外参：t_camera2gimbal（可选，3×1）───────────────────────────────────
  if (calib["t_camera_to_gimbal"]) {
    auto t_vals = calib["t_camera_to_gimbal"].as<std::vector<double>>();
    if (t_vals.size() == 3) {
      t_camera2gimbal_ = Eigen::Vector3d(t_vals[0], t_vals[1], t_vals[2]);
    } else {
      MV_LOG_WARN("PnpSolver", "t_camera_to_gimbal must have 3 values, got {} — ignored",
                  t_vals.size());
    }
  }

  if (calib["R_gimbal_to_imubody"]) {
    auto r_vals = calib["R_gimbal_to_imubody"].as<std::vector<double>>();
    if (r_vals.size() == 9) {
      R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(r_vals.data());
    } else {
      MV_LOG_WARN("PnpSolver", "R_gimbal_to_imubody must have 9 values, got {} — ignored",
                  r_vals.size());
    }
  }

  // ── 装甲板物理尺寸（可选，armor 节点优先）─────────────────────────────
  if (config["armor"]) {
    const auto& armor = config["armor"];
    small_half_w_ = armor["small_half_w"].as<float>(0.0675F);
    big_half_w_ = armor["big_half_w"].as<float>(0.115F);
    half_h_ = armor["half_h"].as<float>(0.0275F);
  }

  initialized_ = true;
  MV_LOG_INFO("PnpSolver",
              "Init OK — fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} "
              "armor small_hw={:.4f} big_hw={:.4f} hh={:.4f}",
              camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
              camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2), small_half_w_,
              big_half_w_, half_h_);
  return true;
}

// ── Solve ─────────────────────────────────────────────────────────────────

/**
 * @brief 基于单解 IPPE 完成单目标位姿解算
 * @param detection 输入角点与装甲类型，输出云台系位置、yaw/pitch 与重投影误差
 * @return true 解算成功并写回 detection 字段
 * @return false 未初始化或 IPPE 解算失败
 * @thread_safety Not thread-safe
 */
bool PnpSolver::Solve(Detection& detection) {
  if (!initialized_) {
    MV_LOG_ERROR("PnpSolver", "Solve() called before Init()");
    return false;
  }

  // 选取世界坐标模板
  const float HALF_W = (detection.type == ArmorType::BIG) ? big_half_w_ : small_half_w_;
  const std::vector<cv::Point3f> WORLD_PTS = MakeWorldPoints(HALF_W, half_h_);

  // 组织图像点
  std::vector<cv::Point2f> img_pts(detection.points.begin(), detection.points.end());

  cv::Vec3d rvec{};
  cv::Vec3d tvec{};

  const bool SOLVED = cv::solvePnP(WORLD_PTS, img_pts, camera_matrix_, dist_coeffs_, rvec, tvec,
                                   false, cv::SOLVEPNP_IPPE);
  if (!SOLVED) {
    MV_LOG_DEBUG("PnpSolver", "solvePnP (IPPE) failed");
    return false;
  }

  // 单解策略下不输出候选解信息，避免沿用历史帧残留值。
  detection.has_alt_solution = false;
  detection.reproj_error_alt = 0.0;
  detection.xyz_in_gimbal_alt = Eigen::Vector3d::Zero();
  detection.reprojected_points_alt.fill(cv::Point2f{});

  // ── 坐标系变换：相机 → 云台（R_c2g * xyz_c + t_c2g）──────────────────────
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);

  const Eigen::Vector3d XYZ_IN_GIMBAL = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

  detection.xyz_in_gimbal = XYZ_IN_GIMBAL;

  // yaw / pitch 从云台坐标系计算
  detection.yaw_angle = std::atan2(XYZ_IN_GIMBAL.x(), XYZ_IN_GIMBAL.z());
  detection.pitch_angle = std::atan2(-XYZ_IN_GIMBAL.y(), XYZ_IN_GIMBAL.z());

  // ── 重投影误差（主解）──────────────────────────────────────────────────────
  {
    std::vector<cv::Point2f> proj_pts;
    cv::projectPoints(WORLD_PTS, rvec, tvec, camera_matrix_, dist_coeffs_, proj_pts);
    if (proj_pts.size() == 4) {
      double err_sq_sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        const auto INDEX = static_cast<size_t>(k);
        detection.reprojected_points.at(INDEX) = proj_pts.at(INDEX);
        const double DELTA_X = img_pts.at(INDEX).x - proj_pts.at(INDEX).x;
        const double DELTA_Y = img_pts.at(INDEX).y - proj_pts.at(INDEX).y;
        err_sq_sum += DELTA_X * DELTA_X + DELTA_Y * DELTA_Y;
      }
      detection.reproj_error = std::sqrt(err_sq_sum / 4.0);
    }
  }

  detection.is_solved = true;
  return true;
}

// ── Yaw 优化（遍历重投影误差）────────────────────────────────────────────

/**
 * @brief 在给定区间遍历 yaw，选择重投影 RMS 最小解
 * @param detection 输入/输出目标结构体，需先由 Solve 填充 xyz_in_gimbal
 * @param yaw_min 搜索下界（rad）
 * @param yaw_max 搜索上界（rad）
 * @param step 搜索步长（rad）
 * @thread_safety Not thread-safe
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void PnpSolver::OptimizeYaw(Detection& detection, double yaw_min, double yaw_max, double step) {
  if (!initialized_) {
    MV_LOG_ERROR("PnpSolver", "OptimizeYaw() called before Init()");
    return;
  }
  // 需要已有的 3D 位置（云台系），通常由 Solve() 填充
  const Eigen::Vector3d XYZ_IN_GIMBAL = detection.xyz_in_gimbal;
  const Eigen::Vector3d XYZ_IN_WORLD = R_gimbal2world_ * XYZ_IN_GIMBAL + t_gimbal2world_;

  // 图像点
  const std::vector<cv::Point2f> IMG_PTS(detection.points.begin(), detection.points.end());

  double best_err = std::numeric_limits<double>::infinity();
  double best_yaw = detection.yaw_angle;
  std::array<cv::Point2f, 4> best_proj{};

  for (double yaw = yaw_min; yaw <= yaw_max; yaw += step) {
    std::array<cv::Point2f, 4> proj_tmp{};
    const double ERROR_VALUE =
        ArmorReprojectionRms(XYZ_IN_WORLD, yaw, detection.type, IMG_PTS, &proj_tmp);
    if (ERROR_VALUE < best_err) {
      best_err = ERROR_VALUE;
      best_yaw = yaw;
      best_proj = proj_tmp;
    }
  }

  detection.yaw_angle = best_yaw;
  detection.reproj_error = best_err;
  detection.reprojected_points = best_proj;
}

/**
 * @brief 根据目标世界坐标与 yaw 重投影装甲板四角点
 *
 * 问题定义：给定目标中心在世界系的位置与朝向，构造
 * \f$R_{armor\to world}\f$，再通过
 * \f$R_{armor\to camera}=R_{c2g}^T R_{g2w}^T R_{armor\to world}\f$
 * 与 \f$t_{armor\to camera}\f$ 投影到像素平面。
 *
 * @param xyz_in_world 目标中心世界坐标（m）
 * @param yaw 目标航向角（rad）
 * @param type 装甲板类型（决定宽度模板）
 * @return 4 个重投影角点（px）；顺序与 MakeWorldPoints 一致
 * @thread_safety Not thread-safe
 */
std::vector<cv::Point2f> PnpSolver::ReprojectArmor(const Eigen::Vector3d& xyz_in_world, double yaw,
                                                   ArmorType type) const {
  const double PITCH_RAD = 15.0 * CV_PI / 180.0;
  const double SIN_YAW = std::sin(yaw);
  const double COS_YAW = std::cos(yaw);
  const double SIN_PITCH = std::sin(PITCH_RAD);
  const double COS_PITCH = std::cos(PITCH_RAD);

  const Eigen::Matrix3d R_ARMOR2WORLD{{COS_YAW * COS_PITCH, -SIN_YAW, COS_YAW * SIN_PITCH},
                                      {SIN_YAW * COS_PITCH, COS_YAW, SIN_YAW * SIN_PITCH},
                                      {-SIN_PITCH, 0.0, COS_PITCH}};
  Eigen::Matrix3d r_armor2camera =
      R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_ARMOR2WORLD;
  Eigen::Vector3d t_armor2camera = R_camera2gimbal_.transpose() *
                                   (R_gimbal2world_.transpose() * xyz_in_world - t_camera2gimbal_);

  cv::Mat r_cv;
  cv::eigen2cv(r_armor2camera, r_cv);
  cv::Vec3d rvec;
  cv::Rodrigues(r_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  std::vector<cv::Point2f> image_points;
  const float HALF_W = (type == ArmorType::BIG) ? big_half_w_ : small_half_w_;
  const auto OBJECT_PTS = MakeWorldPoints(HALF_W, half_h_);
  cv::projectPoints(OBJECT_PTS, rvec, tvec, camera_matrix_, dist_coeffs_, image_points);
  return image_points;
}

/**
 * @brief 计算给定世界坐标与 yaw 的四角点重投影 RMS
 *
 * 核心公式：
 * \f[
 * e_{rms}=\sqrt{\frac{1}{4}\sum_{i=1}^{4}\left\|\mathbf{u}_i-\hat{\mathbf{u}}_i\right\|^2}
 * \f]
 * 其中 \f$\mathbf{u}_i\f$ 为检测角点（px），\f$\hat{\mathbf{u}}_i\f$ 为投影角点（px）。
 *
 * @param xyz_in_world 目标中心世界坐标（m）
 * @param yaw 目标航向角（rad）
 * @param type 装甲板类型
 * @param img_pts 输入图像角点（px）
 * @param out_proj 可选输出，返回本次投影角点
 * @return RMS 重投影误差（px）；若投影异常返回 inf
 * @thread_safety Not thread-safe
 */
double PnpSolver::ArmorReprojectionRms(const Eigen::Vector3d& xyz_in_world, double yaw,
                                       ArmorType type, const std::vector<cv::Point2f>& img_pts,
                                       std::array<cv::Point2f, 4>* out_proj) const {
  auto proj_points = ReprojectArmor(xyz_in_world, yaw, type);
  if (proj_points.size() != 4) {
    return std::numeric_limits<double>::infinity();
  }
  double err_sq = 0.0;
  for (int k = 0; k < 4; ++k) {
    const auto INDEX = static_cast<size_t>(k);
    const double DELTA_X = img_pts.at(INDEX).x - proj_points.at(INDEX).x;
    const double DELTA_Y = img_pts.at(INDEX).y - proj_points.at(INDEX).y;
    err_sq += DELTA_X * DELTA_X + DELTA_Y * DELTA_Y;
    if (out_proj) {
      (*out_proj).at(INDEX) = proj_points.at(INDEX);
    }
  }
  return std::sqrt(err_sq / 4.0);
}

// 计算指定 yaw/pitch 下的重投影误差（兼容 sp 实现）
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
double PnpSolver::ArmorReprojectionError(const Detection& detection, float yaw, float pitch,
                                         float inclined) const {
  if (!initialized_) {
    return std::numeric_limits<double>::infinity();
  }

  // 由 detection 提供云台系位置，转换到世界坐标系
  const Eigen::Vector3d XYZ_IN_GIMBAL = detection.xyz_in_gimbal;
  const Eigen::Vector3d XYZ_IN_WORLD = R_gimbal2world_ * XYZ_IN_GIMBAL + t_gimbal2world_;

  // 与 ReprojectArmor 相同的投影流程，但使用外部传入的 pitch
  const double SIN_YAW = std::sin(yaw);
  const double COS_YAW = std::cos(yaw);
  const double SIN_PITCH = std::sin(pitch);
  const double COS_PITCH = std::cos(pitch);

  const Eigen::Matrix3d R_ARMOR2WORLD{{COS_YAW * COS_PITCH, -SIN_YAW, COS_YAW * SIN_PITCH},
                                      {SIN_YAW * COS_PITCH, COS_YAW, SIN_YAW * SIN_PITCH},
                                      {-SIN_PITCH, 0.0, COS_PITCH}};

  Eigen::Matrix3d r_armor2camera =
      R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_ARMOR2WORLD;
  Eigen::Vector3d t_armor2camera = R_camera2gimbal_.transpose() *
                                   (R_gimbal2world_.transpose() * XYZ_IN_WORLD - t_camera2gimbal_);

  cv::Mat r_cv;
  cv::eigen2cv(r_armor2camera, r_cv);
  cv::Vec3d rvec;
  cv::Rodrigues(r_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  const float HALF_W = (detection.type == ArmorType::BIG) ? big_half_w_ : small_half_w_;
  const auto OBJECT_PTS = MakeWorldPoints(HALF_W, half_h_);
  std::vector<cv::Point2f> proj_pts;
  cv::projectPoints(OBJECT_PTS, rvec, tvec, camera_matrix_, dist_coeffs_, proj_pts);

  if (proj_pts.size() != 4) {
    return std::numeric_limits<double>::infinity();
  }

  // 若提供 inclined (>0)，使用 SJTUCost（像素距离 + 角度差）；否则返回 RMS（与 ArmorReprojectionRms
  // 一致）
  if (inclined > 0.0F) {
    // SJTUCost 接受 (refs, pts, inclined)——refs 为参考（投影），pts 为检测点
    return SJTUCost(proj_pts,
                    std::vector<cv::Point2f>(detection.points.begin(), detection.points.end()),
                    static_cast<double>(inclined));
  }

  double err_sq = 0.0;
  for (int k = 0; k < 4; ++k) {
    const auto INDEX = static_cast<size_t>(k);
    const double DELTA_X = detection.points.at(INDEX).x - proj_pts.at(INDEX).x;
    const double DELTA_Y = detection.points.at(INDEX).y - proj_pts.at(INDEX).y;
    err_sq += DELTA_X * DELTA_X + DELTA_Y * DELTA_Y;
  }
  return std::sqrt(err_sq / 4.0);
}

/**
 * @brief 计算 SJTU 角点匹配代价（像素差 + 边方向差）
 *
 * 对每条边计算归一化像素距离与方向角差，并按倾角进行加权融合：
 * \f$term_1 = d_{pixel}\sin(\theta),\ term_2 = d_{angle}\cos(\theta)\f$，
 * 最终累计 \f$\sqrt{term_1^2 + 2term_2^2}\f$。
 *
 * @param cv_refs 参考角点（通常为投影角点，px）
 * @param cv_pts 待评估角点（通常为检测角点，px）
 * @param inclined 倾角权重参数（rad）
 * @return SJTU 代价，值越小表示匹配越好
 * @note 退化边（长度接近 0）会被跳过
 * @thread_safety Not thread-safe
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
double PnpSolver::SJTUCost(const std::vector<cv::Point2f>& cv_refs,
                           const std::vector<cv::Point2f>& cv_pts, const double& inclined) {
  const std::size_t SIZE = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0U; i < SIZE; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0U; i < SIZE; ++i) {
    const std::size_t NEXT_IDX = (i + 1U) % SIZE;
    Eigen::Vector2d ref_d = refs[NEXT_IDX] - refs[i];
    Eigen::Vector2d pt_d = pts[NEXT_IDX] - pts[i];

    const double REF_LEN = ref_d.norm();
    if (REF_LEN <= 1e-12) {
      continue;  // degenerate edge, skip
    }

    double pixel_dis =
        (0.5 * ((refs[i] - pts[i]).norm() + (refs[NEXT_IDX] - pts[NEXT_IDX]).norm()) +
         std::fabs(REF_LEN - pt_d.norm())) /
        REF_LEN;

    // inline absolute angle between ref_d and pt_d (clamped acos)
    double angular_dis = 0.0;
    const double A_NORM = ref_d.norm();
    const double B_NORM = pt_d.norm();
    if (A_NORM > 1e-12 && B_NORM > 1e-12) {
      double cos_value = (ref_d.dot(pt_d)) / (A_NORM * B_NORM);
      if (cos_value > 1.0) {
        cos_value = 1.0;
      }
      if (cos_value < -1.0) {
        cos_value = -1.0;
      }
      angular_dis = std::acos(cos_value);
    }

    double term1 = pixel_dis * std::sin(inclined);
    double term2 = angular_dis * std::cos(inclined);
    double cost_i = term1 * term1 + 2.0 * term2 * term2;
    cost += std::sqrt(cost_i);
  }
  return cost;
}

}  // namespace mv::modules

/**
 * @brief 将世界坐标点集合投影到图像平面
 *
 * 使用外部给定的世界到相机变换：
 * \f$\mathbf{X}_c = R_{w2c}\mathbf{X}_w + \mathbf{t}_{w2c}\f$，仅保留 \f$Z_c>0\f$ 点。
 *
 * @param world_pts 世界坐标点集合（m）
 * @param R_world2camera 世界系到相机系旋转矩阵
 * @param t_world2camera 世界系到相机系平移向量（m）
 * @return 可见点对应的像素坐标列表（px）；无可见点返回空容器
 * @thread_safety Not thread-safe
 */
std::vector<cv::Point2f> mv::modules::PnpSolver::WorldToPixel(
    const std::vector<cv::Point3f>& world_pts, const Eigen::Matrix3d& r_world2camera,
    const Eigen::Vector3d& t_world2camera) const {
  // 转为 OpenCV rvec/tvec（Rodrigues 需要 3x3 矩阵）
  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(r_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  valid_world_points.reserve(world_pts.size());
  for (const auto& world_point : world_pts) {
    Eigen::Vector3d world_point_e(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = r_world2camera * world_point_e + t_world2camera;
    if (camera_point.z() > 0.0) {
      valid_world_points.push_back(world_point);
    }
  }

  if (valid_world_points.empty()) {
    return {};
  }

  std::vector<cv::Point2f> image_points;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, dist_coeffs_, image_points);
  return image_points;
}
