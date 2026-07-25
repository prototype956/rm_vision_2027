/**
 * @file ekf_track_target.cpp
 * @brief 单目标 EKF 跟踪状态实现（Stage 8-B）
 *
 * 【坐标系约定】
 *   所有传入的 Detection.xyz_in_gimbal 在 EkfPredictor::Predict()（Stage 8-D）
 *   中经 R_gimbal2world 旋转后赋回 xyz_in_gimbal 字段再送入此处，
 *   因此本文件内所有 xyz_in_gimbal 均视为世界坐标系坐标。
 *
 * 【状态向量（11 维）】
 *   x = [cx, dcx, cy, dcy, cz, dcz, α, dα, r, Δl, Δh]
 *        0    1    2    3   4    5   6   7  8   9  10
 *
 * 【观测空间】
 *   与 sp 不同，本实现使用装甲板的直角坐标 [x, y, z] 作为观测量，
 *   避免引入 xyz2ypd 的额外 Jacobian 链，简化数学推导。
 *
 * 【参考】sp_vision_25/tasks/auto_aim/target.cpp
 */
#include "ekf_track_target.hpp"

#include "core/logger.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace mv::modules::detail {

// ── 内部工具 ─────────────────────────────────────────────────────────────────

namespace {
/// 将角度限制到 (-π, π]
inline double LimitRad(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle <= -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

/// 发散检测：半径范围 [0.05 m, 0.5 m] 之内认为合理
constexpr double kRadiusMin = 0.05;
constexpr double kRadiusMax = 0.60;

/// 收敛所需最少 Update 次数
constexpr int kConvergeCountNormal = 3;
constexpr int kConvergeCountOutpost = 10;
}  // namespace

// ── 构造函数 ─────────────────────────────────────────────────────────────────

EkfTrackTarget::EkfTrackTarget(const Detection& detection, std::chrono::steady_clock::time_point t,
                               double radius, int armor_num, const Eigen::VectorXd& P0_diag,
                               const ProcessNoiseParams& process_noise_params)
    : name(detection.number),
      armor_type(detection.type),
      is_init(true),
      armor_num_(armor_num),
      t_(t),
      process_noise_pos_(process_noise_params.process_noise_pos),
      process_noise_ang_(process_noise_params.process_noise_ang),
      process_noise_outpost_pos_(process_noise_params.process_noise_outpost_pos),
      process_noise_outpost_ang_(process_noise_params.process_noise_outpost_ang) {
  // 1. 从检测结果取装甲板 3D 位置（世界系）和 yaw
  const double ax = detection.xyz_in_gimbal[0];
  const double ay = detection.xyz_in_gimbal[1];
  const double az = detection.xyz_in_gimbal[2];
  const double yaw = detection.yaw_angle;  // 装甲板法向 yaw（世界系）

  // 2. 从装甲板位置推算旋转中心（sp 约定：center = armor + r * [cos,sin,0]）
  const double cx = ax + radius * std::cos(yaw);
  const double cy = ay + radius * std::sin(yaw);

  // 3. 初始状态向量（零速，零角速度，无大小板差）
  Eigen::VectorXd x0(11);
  x0 << cx, 0.0, cy, 0.0, az, 0.0, yaw, 0.0, radius, 0.0, 0.0;

  // 4. 初始协方差
  Eigen::MatrixXd P0 = P0_diag.asDiagonal();

  // 5. 角度安全加法（防止 α 越过 ±π 折叠）
  auto x_add = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = LimitRad(c[6]);
    return c;
  };

  ekf_ = tool::ExtendedKalmanFilter(x0, P0, x_add);
}

// ── 时间预测步 ───────────────────────────────────────────────────────────────

void EkfTrackTarget::Predict(std::chrono::steady_clock::time_point t) {
  const double dt = std::chrono::duration<double>(t - t_).count();
  t_ = t;
  Predict(dt);
}

void EkfTrackTarget::Predict(double dt) {
  // ── 状态转移矩阵 F（11×11，CV 模型）────────────────────────────────────
  // clang-format off
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
  F(0, 1) = dt;   // cx += dcx * dt
  F(2, 3) = dt;   // cy += dcy * dt
  F(4, 5) = dt;   // cz += dcz * dt
  F(6, 7) = dt;   // α  += dα  * dt
  // clang-format on

  // ── 分段白噪声模型 Q（PWNM）──────────────────────────────────────────────
  // v1: 线性加速度方差（m²/s⁴），v2: 角加速度方差（rad²/s⁴）
  double v1 = process_noise_pos_;
  double v2 = process_noise_ang_;
  if (name == ArmorNumber::OUTPOST) {
    v1 = process_noise_outpost_pos_;
    v2 = process_noise_outpost_ang_;
  }

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
  // 位置-速度对（cx/dcx, cy/dcy, cz/dcz）
  for (int i : {0, 2, 4}) {
    Q(i, i) = dt4 / 4.0 * v1;
    Q(i, i + 1) = dt3 / 2.0 * v1;
    Q(i + 1, i) = dt3 / 2.0 * v1;
    Q(i + 1, i + 1) = dt2 * v1;
  }
  // 角度-角速度对（α/dα）
  Q(6, 6) = dt4 / 4.0 * v2;
  Q(6, 7) = dt3 / 2.0 * v2;
  Q(7, 6) = dt3 / 2.0 * v2;
  Q(7, 7) = dt2 * v2;
  // r, Δl, Δh 设为 0（认为半径和高度差基本不变）

  // 非线性预测函数：对 CV 而言 f(x) = F*x，但需要 wrap 角度
  auto f = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    Eigen::VectorXd xp = F * x;
    xp[6] = LimitRad(xp[6]);
    return xp;
  };

  // 前哨站角速度饱和（防止 EKF 发散后角速度估计无界增长）
  if (name == ArmorNumber::OUTPOST && std::abs(ekf_.x[7]) > 2.51) {
    ekf_.x[7] = ekf_.x[7] > 0.0 ? 2.51 : -2.51;
  }

  ekf_.Predict(F, Q, f);
}

// ── 观测更新步 ───────────────────────────────────────────────────────────────

void EkfTrackTarget::Update(const Detection& detection) {
  update_count_++;

  // 1. 由检测 3D 点构造 ypd 观测（世界系）
  const Eigen::Vector3d armor_xyz = detection.xyz_in_gimbal;
  const double xy_norm = std::hypot(armor_xyz[0], armor_xyz[1]);
  const double obs_dist = armor_xyz.norm();
  const double obs_yaw = std::atan2(armor_xyz[1], armor_xyz[0]);
  const double obs_pitch = std::atan2(armor_xyz[2], std::max(1e-9, xy_norm));

  // 2. 匹配最近装甲板 ID（按观测装甲板 yaw 选择）
  const int id = SelectNearestArmor(ekf_.x, detection.yaw_angle);

  // 3. 跳变检测
  if (id != last_id) {
    is_switch_ = true;
    switch_count_++;
    jumped = (switch_count_ > 0);
  } else {
    is_switch_ = false;
  }
  last_id = id;

  // 4. 观测函数 h: R^11 -> [yaw, pitch, dist, armor_yaw]
  auto h = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    const Eigen::Vector3d xyz = ArmorXyz(x, id);
    const double xy = std::hypot(xyz[0], xyz[1]);
    Eigen::VectorXd ypd_yaw(4);
    ypd_yaw[0] = std::atan2(xyz[1], xyz[0]);
    ypd_yaw[1] = std::atan2(xyz[2], std::max(1e-9, xy));
    ypd_yaw[2] = xyz.norm();
    ypd_yaw[3] = LimitRad(x[6] + id * 2.0 * M_PI / armor_num_);
    return ypd_yaw;
  };

  // 5. 观测雅可比 H（4x11，分量顺序：[yaw,pitch,dist,armor_yaw]）
  const Eigen::MatrixXd H = HJacobian(ekf_.x, id);

  // 6. 观测量与观测噪声
  Eigen::VectorXd z(4);
  z << obs_yaw, obs_pitch, obs_dist, detection.yaw_angle;

  Eigen::Vector4d r_diag;
  r_diag << 4e-3, 4e-3, 1.0, 9e-2;
  const Eigen::MatrixXd R = r_diag.asDiagonal();

  // 7. 调用 EKF Update（角度分量用 limit_rad 处理）
  ekf_.Update(z, H, R, h,
              [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
                Eigen::VectorXd c = a - b;
                c[0] = LimitRad(c[0]);
                c[1] = LimitRad(c[1]);
                c[3] = LimitRad(c[3]);
                return c;
              });
}

// ── 状态查询 ─────────────────────────────────────────────────────────────────

Eigen::VectorXd EkfTrackTarget::ekf_x() const {
  return ekf_.x;
}

const tool::ExtendedKalmanFilter& EkfTrackTarget::ekf() const {
  return ekf_;
}

std::vector<Eigen::Vector4d> EkfTrackTarget::ArmorXyzaList() const {
  std::vector<Eigen::Vector4d> list;
  list.reserve(static_cast<size_t>(armor_num_));
  for (int i = 0; i < armor_num_; ++i) {
    const Eigen::Vector3d xyz = ArmorXyz(ekf_.x, i);
    const double angle = LimitRad(ekf_.x[6] + i * 2.0 * M_PI / armor_num_);
    list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return list;
}

// ── 发散 / 收敛检测 ──────────────────────────────────────────────────────────

bool EkfTrackTarget::Diverged() const {
  // 方案 1：半径物理约束（最直观）
  const double r = ekf_.x[8];
  const double r2 = armor_num_ == 4 ? r + ekf_.x[9] : r;
  const bool r_ok = r >= kRadiusMin && r <= kRadiusMax;
  const bool r2_ok = r2 >= kRadiusMin && r2 <= kRadiusMax;
  return !(r_ok && r2_ok);
}

bool EkfTrackTarget::Converged() const {
  if (is_converged_)
    return true;  // 缓存，避免重复检查

  const int threshold =
      (name == ArmorNumber::OUTPOST) ? kConvergeCountOutpost : kConvergeCountNormal;
  if (update_count_ >= threshold && !Diverged()) {
    is_converged_ = true;
    return true;
  }
  return false;
}

// ── 内部辅助 ─────────────────────────────────────────────────────────────────

Eigen::Vector3d EkfTrackTarget::ArmorXyz(const Eigen::VectorXd& x, int id) const {
  // armor_angle = α + id * 2π/N（sp 约定：center → armor 方向的负值）
  const double angle = LimitRad(x[6] + id * 2.0 * M_PI / armor_num_);

  // 4 板车：奇数 id（1, 3）使用"侧板"参数（半径 r+Δl，高度 cz+Δh）
  const bool use_side = (armor_num_ == 4) && (id == 1 || id == 3);
  const double r = use_side ? x[8] + x[9] : x[8];

  // sp 约定：armor = center - r * [cos, sin, 0]
  return {x[0] - r * std::cos(angle), x[2] - r * std::sin(angle), use_side ? x[4] + x[10] : x[4]};
}

Eigen::MatrixXd EkfTrackTarget::HJacobian(const Eigen::VectorXd& x, int id) const {
  // h(x) = [yaw, pitch, dist, armor_yaw]
  // 先求 xyz 对状态的导数，再链式乘 xyz->ypd 的导数，最后补 armor_yaw 行。

  // ── Step 1: d(armor_xyz)/dx（3x11）────────────────────────────────────
  const double angle = LimitRad(x[6] + id * 2.0 * M_PI / armor_num_);
  const bool use_side = (armor_num_ == 4) && (id == 1 || id == 3);
  const double r = use_side ? x[8] + x[9] : x[8];

  const double cos_a = std::cos(angle);
  const double sin_a = std::sin(angle);

  Eigen::MatrixXd h_xyz = Eigen::MatrixXd::Zero(3, 11);
  //              cx   dcx  cy   dcy  cz   dcz     α            dα     r           Δl Δh
  h_xyz(0, 0) = 1.0;                      // ∂ax/∂cx
  h_xyz(0, 6) = r * sin_a;                // ∂ax/∂α
  h_xyz(0, 8) = -cos_a;                   // ∂ax/∂r
  h_xyz(0, 9) = use_side ? -cos_a : 0.0;  // ∂ax/∂Δl

  h_xyz(1, 2) = 1.0;                      // ∂ay/∂cy
  h_xyz(1, 6) = -r * cos_a;               // ∂ay/∂α
  h_xyz(1, 8) = -sin_a;                   // ∂ay/∂r
  h_xyz(1, 9) = use_side ? -sin_a : 0.0;  // ∂ay/∂Δl

  h_xyz(2, 4) = 1.0;                    // ∂az/∂cz
  h_xyz(2, 10) = use_side ? 1.0 : 0.0;  // ∂az/∂Δh

  // ── Step 2: d(ypd)/d(xyz)（3x3）──────────────────────────────────────
  const Eigen::Vector3d pred_xyz = ArmorXyz(x, id);
  const double px = pred_xyz[0];
  const double py = pred_xyz[1];
  const double pz = pred_xyz[2];
  const double r2 = px * px + py * py;
  const double r_norm = std::sqrt(std::max(1e-12, r2));
  const double d2 = r2 + pz * pz;
  const double d_norm = std::sqrt(std::max(1e-12, d2));

  Eigen::Matrix<double, 3, 3> j_xyz_to_ypd = Eigen::Matrix<double, 3, 3>::Zero();
  // yaw = atan2(y,x)
  j_xyz_to_ypd(0, 0) = -py / std::max(1e-12, r2);
  j_xyz_to_ypd(0, 1) = px / std::max(1e-12, r2);
  // pitch = atan2(z, sqrt(x^2+y^2))
  j_xyz_to_ypd(1, 0) = -px * pz / std::max(1e-12, r_norm * d2);
  j_xyz_to_ypd(1, 1) = -py * pz / std::max(1e-12, r_norm * d2);
  j_xyz_to_ypd(1, 2) = r_norm / std::max(1e-12, d2);
  // dist = sqrt(x^2+y^2+z^2)
  j_xyz_to_ypd(2, 0) = px / std::max(1e-12, d_norm);
  j_xyz_to_ypd(2, 1) = py / std::max(1e-12, d_norm);
  j_xyz_to_ypd(2, 2) = pz / std::max(1e-12, d_norm);

  // ── Step 3: 链式合成 + armor_yaw 行（4x11）────────────────────────────
  Eigen::MatrixXd h_ypd_yaw = Eigen::MatrixXd::Zero(4, 11);
  h_ypd_yaw.block(0, 0, 3, 11) = j_xyz_to_ypd * h_xyz;
  h_ypd_yaw(3, 6) = 1.0;  // armor_yaw = alpha + id*const

  return h_ypd_yaw;
}

int EkfTrackTarget::SelectNearestArmor(const Eigen::VectorXd& x, double observed_yaw) const {
  int best_id = 0;
  double min_err = 1e9;
  constexpr double kTieEps = 1e-9;

  for (int i = 0; i < armor_num_; ++i) {
    const double pred_angle = LimitRad(x[6] + i * 2.0 * M_PI / armor_num_);
    const double err = std::abs(LimitRad(observed_yaw - pred_angle));
    if (err < min_err - kTieEps) {
      min_err = err;
      best_id = i;
      continue;
    }

    // 角误差相等时优先保持上一帧 ID，减少候选切换抖动。
    if (std::abs(err - min_err) <= kTieEps && i == last_id) {
      best_id = i;
    }
  }
  return best_id;
}

}  // namespace mv::modules::detail
