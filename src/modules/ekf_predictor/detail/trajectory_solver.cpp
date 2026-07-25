/**
 * @file trajectory_solver.cpp
 * @brief 飞行时间迭代求解器实现（Stage 8-B）
 *
 * 【弹道模型（不考虑空气阻力）】
 *   二次方程：a·tan²θ - d·tanθ + (a+h) = 0
 *   其中 a = g·d²/(2·v₀²)，取飞行时间最短的解。
 *
 * 【参考】sp_vision_25/tasks/auto_aim/aimer.cpp
 */
#include "trajectory_solver.hpp"

#include "core/logger.hpp"

#include <algorithm>
#include <cmath>

namespace mv::modules::detail {

// ── 内部弹道模型 ──────────────────────────────────────────────────────────────

namespace {

/// 重力加速度（武汉赛场实测，sp 取值）
constexpr double GRAVITY = 9.7833;

/// 弹道求解结果
struct Trajectory {
  bool unsolvable{true};
  double fly_time{0.0};
  double pitch{0.0};  ///< 抬头为正（rad）
};

/**
 * @brief 单点弹道求解（不考虑空气阻力）
 *
 * @param v0  初速度（m/s）
 * @param d   目标水平距离（m）
 * @param h   目标竖直高度（m，向上为正）
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Trajectory SolveTrajectory(double bullet_speed, double horizontal_dist, double vertical_height) {
  if (bullet_speed < 1.0 || horizontal_dist < 1e-3) {
    return {};
  }

  const double COEFF_A =
      GRAVITY * horizontal_dist * horizontal_dist / (2.0 * bullet_speed * bullet_speed);
  const double COEFF_B = -horizontal_dist;
  const double COEFF_C = COEFF_A + vertical_height;
  const double DELTA = COEFF_B * COEFF_B - 4.0 * COEFF_A * COEFF_C;

  if (DELTA < 0.0) {
    return {};  // 不可解
  }

  const double TAN_1 = (-COEFF_B + std::sqrt(DELTA)) / (2.0 * COEFF_A);
  const double TAN_2 = (-COEFF_B - std::sqrt(DELTA)) / (2.0 * COEFF_A);
  const double PITCH_1 = std::atan(TAN_1);
  const double PITCH_2 = std::atan(TAN_2);
  const double FLY_TIME_1 = horizontal_dist / (bullet_speed * std::cos(PITCH_1));
  const double FLY_TIME_2 = horizontal_dist / (bullet_speed * std::cos(PITCH_2));

  // 取飞行时间较短的解
  const bool USE_1 = FLY_TIME_1 < FLY_TIME_2;
  return {false, USE_1 ? FLY_TIME_1 : FLY_TIME_2, USE_1 ? PITCH_1 : PITCH_2};
}

}  // namespace

// ── 构造函数 ─────────────────────────────────────────────────────────────────

TrajectorySolver::TrajectorySolver(const TrajectorySolverParams& params) : params_(params) {}

// ── Solve（主入口）──────────────────────────────────────────────────────────

GimbalControl TrajectorySolver::Solve(const EkfTrackTarget& target,
                                      std::chrono::steady_clock::time_point timestamp,
                                      double bullet_speed) {
  GimbalControl ctrl;
  ctrl.timestamp = timestamp;

  // 弹速保护（过低时退化为固定值，防止弹道求解发散）
  if (bullet_speed < 14.0) {
    bullet_speed = 23.0;
  }

  // 1. 根据弹速选择发弹延迟补偿时间
  const double DELAY_S = (bullet_speed < params_.decision_speed)
                             ? params_.low_speed_delay_ms / 1000.0
                             : params_.high_speed_delay_ms / 1000.0;

  // 2. 计算"当前时刻到发弹"的总延迟：检测器→预测器耗时(≈5ms) + 发弹延迟
  //    以 timestamp 为基准向未来外推
  const double INIT_DT_S = 0.005 + DELAY_S;
  auto future = timestamp + std::chrono::microseconds(static_cast<int64_t>(INIT_DT_S * 1e6));

  // 3. 预测目标状态（操作副本，不修改调用方内的 target）
  EkfTrackTarget predicted_target = target;
  predicted_target.Predict(future);

  // 4. 初次弹道求解（提供初始飞行时间估计）
  const AimPoint AIM_0 = ChooseAimPoint(predicted_target, 0.0);
  if (!AIM_0.valid) {
    MV_LOG_DEBUG("TrajectorySolver", "No valid aim point (initial)");
    return ctrl;
  }

  const Eigen::Vector3d XYZ_0 = AIM_0.xyza.head<3>();
  const double DIST_0 = std::sqrt(XYZ_0[0] * XYZ_0[0] + XYZ_0[1] * XYZ_0[1]);
  Trajectory traj = SolveTrajectory(bullet_speed, DIST_0, XYZ_0[2]);

  if (traj.unsolvable) {
    MV_LOG_DEBUG("TrajectorySolver", "Unsolvable initial trajectory d={:.2f} z={:.2f}", DIST_0,
                 XYZ_0[2]);
    return ctrl;
  }

  // 5. 迭代收敛飞行时间（最多 max_iter 次）
  const double CONVERGE_S = params_.iter_converge_ms / 1000.0;
  AimPoint final_aim = AIM_0;

  for (int iter = 0; iter < params_.max_iter; ++iter) {
    // 以"当前未来时刻 + 上一次飞行时间"预测目标位置
    const auto PREDICT_T =
        future + std::chrono::microseconds(static_cast<int64_t>(traj.fly_time * 1e6));

    EkfTrackTarget iter_target = target;
    iter_target.Predict(PREDICT_T);

    const AimPoint AIM_I = ChooseAimPoint(iter_target, 0.0);
    if (!AIM_I.valid) {
      MV_LOG_DEBUG("TrajectorySolver", "No valid aim point at iter {}", iter);
      return ctrl;
    }

    const Eigen::Vector3d XYZ_I = AIM_I.xyza.head<3>();
    const double DIST_I = std::sqrt(XYZ_I[0] * XYZ_I[0] + XYZ_I[1] * XYZ_I[1]);
    const Trajectory TRAJ_NEW = SolveTrajectory(bullet_speed, DIST_I, XYZ_I[2]);

    if (TRAJ_NEW.unsolvable) {
      MV_LOG_DEBUG("TrajectorySolver", "Unsolvable at iter {} d={:.2f}", iter, DIST_I);
      return ctrl;
    }

    final_aim = AIM_I;

    if (std::abs(TRAJ_NEW.fly_time - traj.fly_time) < CONVERGE_S) {
      traj = TRAJ_NEW;
      break;  // 收敛
    }

    traj = TRAJ_NEW;
  }

  // 6. 计算最终 yaw / pitch 并加偏置
  const Eigen::Vector3d FINAL_XYZ = final_aim.xyza.head<3>();
  const double YAW = std::atan2(FINAL_XYZ[1], FINAL_XYZ[0]) + params_.yaw_offset_rad;
  const double PITCH = -(traj.pitch + params_.pitch_offset_rad);  // 世界系向上为负

  ctrl.tracking = true;
  ctrl.yaw = YAW;
  ctrl.pitch = PITCH;
  ctrl.distance = FINAL_XYZ.norm();
  ctrl.fire = false;  // fire 由 Voter 决定

  debug_aim_point_ = final_aim;
  return ctrl;
}

// ── ChooseAimPoint ───────────────────────────────────────────────────────────

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
AimPoint TrajectorySolver::ChooseAimPoint(const EkfTrackTarget& target, double current_yaw) const {
  (void)current_yaw;  // 保留参数供未来接入云台实际 yaw
  const std::vector<Eigen::Vector4d> XYZA_LIST = target.ArmorXyzaList();
  if (XYZA_LIST.empty()) {
    return {};
  }

  // 如果目标尚未发生跳变，直接选 id=0 的装甲板
  if (!target.jumped) {
    return {true, XYZA_LIST[0]};
  }

  const Eigen::VectorXd& ekf_x = target.ekf_x();
  // 整车旋转中心在世界系 xy 平面的投影角（cloud→center 方向）
  const double CENTER_YAW = std::atan2(ekf_x[2], ekf_x[0]);  // x[2]=cy, x[0]=cx

  const int ARMOR_COUNT = static_cast<int>(XYZA_LIST.size());

  // 计算每块装甲板相对旋转中心的 delta_angle
  std::vector<double> delta_angles(ARMOR_COUNT);
  for (int i = 0; i < ARMOR_COUNT; ++i) {
    // limit_rad
    double delta_angle = XYZA_LIST[i][3] - CENTER_YAW;
    while (delta_angle > M_PI) {
      delta_angle -= 2.0 * M_PI;
    }
    while (delta_angle <= -M_PI) {
      delta_angle += 2.0 * M_PI;
    }
    delta_angles[i] = delta_angle;
  }

  // 非小陀螺（角速度 |dα| <= 2 且非前哨站）：选在可射击范围内、delta_angle 绝对值最小的装甲板
  if (std::abs(ekf_x[7]) <= 2.0 && target.name != ArmorNumber::OUTPOST) {
    const double MAX_ANGLE = params_.max_approaching_angle;

    // 先收集可射击候选 id
    std::vector<int> id_list;
    for (int i = 0; i < ARMOR_COUNT; ++i) {
      if (std::abs(delta_angles[i]) > MAX_ANGLE) {
        continue;
      }
      id_list.push_back(i);
    }

    if (id_list.empty()) {
      return {false, XYZA_LIST[0]};
    }

    // 双候选锁定模式：防止在两个可击打装甲板之间来回翻转。
    if (id_list.size() > 1) {
      int first_candidate_id = id_list[0];
      int second_candidate_id = id_list[1];

      // 未锁定或锁定 id 已失效时，按角误差更小者进入锁定。
      if (lock_id_ != first_candidate_id && lock_id_ != second_candidate_id) {
        lock_id_ = (std::abs(delta_angles[first_candidate_id]) <
                    std::abs(delta_angles[second_candidate_id]))
                       ? first_candidate_id
                       : second_candidate_id;
      }
      return {true, XYZA_LIST[lock_id_]};
    }

    // 只有单候选时退出锁定模式。
    lock_id_ = -1;
    return {true, XYZA_LIST[id_list[0]]};
  }

  // 小陀螺模式：选择"正在靠近"的装甲板
  const double COMING_ANGLE =
      (target.name == ArmorNumber::OUTPOST) ? 70.0 / 57.3 : params_.max_approaching_angle;
  const double LEAVING_ANGLE =
      (target.name == ArmorNumber::OUTPOST) ? 30.0 / 57.3 : params_.max_leaving_angle;

  for (int i = 0; i < ARMOR_COUNT; ++i) {
    if (std::abs(delta_angles[i]) > COMING_ANGLE) {
      continue;
    }
    // dα > 0 = 逆时针，选 delta_angle > -leaving_angle 的装甲板
    if (ekf_x[7] > 0.0 && delta_angles[i] < LEAVING_ANGLE) {
      return {true, XYZA_LIST[i]};
    }
    if (ekf_x[7] < 0.0 && delta_angles[i] > -LEAVING_ANGLE) {
      return {true, XYZA_LIST[i]};
    }
  }

  return {false, XYZA_LIST[0]};
}

}  // namespace mv::modules::detail
