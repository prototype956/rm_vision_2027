/**
 * @file ekf_track_target.hpp
 * @brief 单目标 EKF 跟踪状态封装（11 维状态向量）
 *
 * 【职责】
 *   EkfTrackTarget 封装单个装甲车目标的扩展卡尔曼滤波器：
 *   - 持有 ExtendedKalmanFilter 实例；
 *   - 提供面向业务的 Predict(dt) / Update(detection) 接口；
 *   - 提供装甲板空间位置查询（armor_xyza_list()），供 TrajectorySolver 选最优瞄准点。
 *
 * 【状态向量（11 维，世界坐标系）】
 *
 *   x = [ cx,  dcx,   cy,  dcy,   cz,  dcz,   α,  dα,   r,  Δl,  Δh ]
 *         ─────────────────────────────────────────────────────────────
 *   序号:   0    1     2    3      4    5      6   7    8    9   10
 *
 *   - (cx, cy, cz)   : 旋转中心在世界系中的 3D 坐标（m）
 *   - (dcx,dcy,dcz)  : 旋转中心速度（m/s）
 *   - α              : 当前朝向装甲板的旋转角（rad），范围 [-π, π]
 *   - dα             : 旋转角速度（rad/s，正值=逆时针）
 *   - r              : 主旋转半径（m，正面装甲板到旋转中心的距离）
 *   - Δl             : 大小板半径差（r_side - r，仅对 4 板车有效）
 *   - Δh             : 大小板高度差（z_side - z，仅对 4 板车有效）
 *
 * 【为什么在世界系建模？】
 *   云台运动时，旋转中心在云台系中会随云台旋转移动，导致 EKF 建模的
 *   匀速假设失效（dcx/dcy 会突变）。世界系中旋转中心仅受目标运动影响，
 *   云台运动被 IMU 旋转补偿消除，EKF 精度更高。
 *   坐标系变换链：Detection.xyz_in_gimbal → R_gimbal2world → EKF 世界系输入。
 *
 * 【多板装甲车建模（不同 armor_num 的差异）】
 *
 *   armor_num = 4（步兵/英雄/哨兵）：
 *     board 0（主）: angle = α
 *     board 1（侧）: angle = α + π/2
 *     board 2（后）: angle = α + π
 *     board 3（侧）: angle = α + 3π/2
 *     大小装甲板交替，半径分别为 r 和 r + Δl，高度分别为 z 和 z + Δh
 *
 *   armor_num = 3（前哨站）：
 *     board 0/1/2：间隔 2π/3，半径均为 r，高度均为 z（无 Δl/Δh）
 *
 *   armor_num = 2（平衡步兵）：
 *     board 0/1：间隔 π，半径均为 r
 *
 * 【发散检测】
 *   - Diverged()：半径物理边界超限（r 或 r+Δl 不在允许区间）视为发散；
 *   - Converged()：更新次数达到阈值且未发散，视为收敛。
 *
 * 【线程安全】
 *   非线程安全，由 EkfTracker 在单一线程顺序调用。
 */
#pragma once

#include "interfaces/types.hpp"
#include "tool/ekf/extended_kalman_filter.hpp"

#include <chrono>
#include <vector>

#include <Eigen/Dense>

namespace mv::modules::detail {

struct ProcessNoiseParams {
  double process_noise_pos{100.0};
  double process_noise_ang{400.0};
  double process_noise_outpost_pos{10.0};
  double process_noise_outpost_ang{0.1};
};

/**
 * @brief 单目标 EKF 跟踪状态（11 维状态向量，世界坐标系）
 *
 * 对 ExtendedKalmanFilter 的业务封装层：
 * - Update(detection) 构建 4 维观测 z=[yaw,pitch,dist,armor_yaw] 送入 EKF；
 * - Predict(dt) 推进状态估计，构建状态转移矩阵 F 和过程噪声 Q；
 * - armor_xyza_list() 根据当前状态向量反算所有装甲板的 [x,y,z,yaw]。
 */
class EkfTrackTarget {
 public:
  // ── 公开标签（供 EkfTracker 识别目标）────────────────────────────────
  ArmorNumber name{ArmorNumber::UNKNOWN};
  ArmorType armor_type{ArmorType::SMALL};
  bool jumped{false};  ///< 本帧是否发生了装甲板跳变（调试可视化用）
  int last_id{0};      ///< 上一帧匹配的装甲板 ID（调试用）

  // ── 构造 ─────────────────────────────────────────────────────────────

  EkfTrackTarget() = default;

  /**
   * @brief 从第一次检测结果初始化 EKF
   *
   * @param detection   首帧检测结果（Detection.xyz_in_gimbal，上游已旋转到世界系）
   * @param t           当前时间戳
   * @param radius      初始旋转半径估计（m，从配置读取，按 armor_num 区分）
   * @param armor_num   装甲板数（2/3/4，决定多板建模策略）
   * @param P0_diag     初始协方差对角元素（11 维，从配置读取）
   * @param process_noise_params 过程噪声参数集合（普通/前哨站）
   */
  EkfTrackTarget(const Detection& detection, std::chrono::steady_clock::time_point t, double radius,
                 int armor_num, const Eigen::VectorXd& P0_diag,
                 const ProcessNoiseParams& process_noise_params = {});

  // ── 核心接口 ──────────────────────────────────────────────────────────

  /**
   * @brief 时间更新步（推进状态预测）
   *
   * 根据 dt 构建 CV 状态转移矩阵 F 和分段白噪声 Q，调用 ekf_.Predict()。
   * 注意角速度分量的 Q 与位置分量不同（前哨站转速慢，普通车快）。
   *
   * @param t   当前帧时间戳（内部计算 dt = t - t_）
   */
  void Predict(std::chrono::steady_clock::time_point t);

  /**
   * @brief 时间更新步（直接传 dt，供轨迹外推使用）
   * @param dt  时间间隔（s）
   */
  void Predict(double dt);

  /**
   * @brief 观测更新步（融合新检测结果）
   *
   * 从 Detection.xyz_in_gimbal（上游已旋转到世界系）构建 4 维观测
   * z=[obs_yaw, obs_pitch, obs_dist, detection.yaw_angle]，
   * 匹配最近装甲板 ID，构建观测 h(x) 和雅可比 H，调用 ekf_.Update()。
   *
   * @param detection  当前帧匹配到的装甲板检测结果（Detection.xyz_in_gimbal，上游已转世界系）
   */
  void Update(const Detection& detection);

  // ── 状态查询 ──────────────────────────────────────────────────────────

  /** @return EKF 当前状态向量（11 维）*/
  [[nodiscard]] Eigen::VectorXd ekf_x() const;

  /** @return EKF 内部对象（只读，供 EkfTracker 的 NIS 检查）*/
  [[nodiscard]] const tool::ExtendedKalmanFilter& ekf() const;

  /**
   * @brief 计算所有装甲板的空间位置列表
   *
   * 返回每块装甲板的 [x, y, z, yaw]（4 维向量，世界系，单位 m/rad）。
   * 装甲板数由 armor_num_ 决定（2/3/4 块）。
   * TrajectorySolver 遍历此列表，选择与云台朝向最近的装甲板作为瞄准点。
   *
   * @return 装甲板位置列表，长度 = armor_num_
   */
  [[nodiscard]] std::vector<Eigen::Vector4d> ArmorXyzaList() const;

  // ── 收敛 / 发散检测 ──────────────────────────────────────────────────

  /**
   * @brief 检查 EKF 是否发散（数值不稳定）
   *
   * 判据：半径物理边界超限（r 或 r+Δl 超出允许范围）。
   * EkfTracker 在每帧 Update 后调用此方法，发散则重置为 LOST。
   */
  [[nodiscard]] bool Diverged() const;

  /**
   * @brief 检查 EKF 是否已收敛（观测与模型吻合）
   *
   * 判据：更新次数达到阈值（普通目标 >= 3，前哨站 >= 10）且未发散。
   * 收敛后 EkfTracker 才将状态转换到 TRACKING（允许开火）。
   */
  [[nodiscard]] bool Converged() const;

  /** @return 是否已完成初始化（构造后置 true）*/
  bool is_init{false};

 private:
  int armor_num_{4};     ///< 装甲板数（2/3/4）
  int switch_count_{0};  ///< 装甲板跳变计数（累积，调试用）
  int update_count_{0};  ///< 累积 Update() 调用次数

  bool is_switch_{false};             ///< 本次 Update 是否发生跳变
  mutable bool is_converged_{false};  ///< EKF 是否已收敛（缓存，避免每帧重查窗口）

  tool::ExtendedKalmanFilter ekf_;           ///< EKF 实例（核心滤波器）
  std::chrono::steady_clock::time_point t_;  ///< 上次更新时间戳

  // 过程噪声参数（由 tracker/init 注入）
  double process_noise_pos_{100.0};
  double process_noise_ang_{400.0};
  double process_noise_outpost_pos_{10.0};
  double process_noise_outpost_ang_{0.1};

  /**
   * @brief 计算单块装甲板在世界系中的 3D 坐标
   *
   * 从状态向量 x 推算第 id 块装甲板的 xyz（观测函数 h 的正向计算）。
   * 用于：①构造 Update 的 h(x̂⁻)；②在 ArmorXyzaList() 中输出所有装甲板位置。
   *
   * @param x  11 维状态向量
   * @param id 装甲板序号（0~armor_num_-1）
   */
  [[nodiscard]] Eigen::Vector3d ArmorXyz(const Eigen::VectorXd& x, int id) const;

  /**
   * @brief 计算观测函数 h(x) 关于 x 的雅可比矩阵
   *
   * h: R^11 -> R^4（h(x) = [yaw, pitch, dist, armor_yaw]）
   * 返回 4x11 雅可比矩阵。
   * 解析求导，非数值差分（精度更高、速度更快）。
   *
   * @param x  当前状态向量（在此处线性化）
   * @param id 装甲板序号
   */
  [[nodiscard]] Eigen::MatrixXd HJacobian(const Eigen::VectorXd& x, int id) const;

  /** @brief 选择与观测角 ψ 最近的装甲板 ID；同误差时优先保持 last_id 防抖 */
  [[nodiscard]] int SelectNearestArmor(const Eigen::VectorXd& x, double observed_yaw) const;
};

}  // namespace mv::modules::detail
