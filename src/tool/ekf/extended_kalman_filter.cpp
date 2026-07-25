/**
 * @file extended_kalman_filter.cpp
 * @brief 扩展卡尔曼滤波器实现
 *
 * 【实现说明（Stage 8-A）】
 *   参考：sp_vision_25/tools/extended_kalman_filter.cpp，进行以下改进：
 *
 *   1. 命名规范：snake_case → PascalCase（Google style）；
 *      命名空间：tools:: → mv::tool::。
 *
 *   2. NIS 使用先验创新（Prior Innovation）而非后验残差：
 *      - sp 原码在 x 更新后才计算 residual，实际是后验残差，不符合标准 NIS 定义；
 *      - 本实现在 x 更新前保存 innovation = z_subtract(z, h(x̂⁻)) 和
 *        先验 S = H·P⁻·Hᵀ + R，再传给 UpdateNis()，数学上正确。
 *
 *   3. 后验协方差使用 Joseph 稳定形式：
 *        P = (I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ
 *      相比对称形式 P = (I−KH)·P，当 K 计算存在数值误差时仍保持 P 正定。
 *
 *   4. 卡方临界值表：按观测维度动态查표（1~6 维），避免硬编码某一维度。
 *      失败判据：NIS > chi2_95（上界）—— 超出 95% 置信区间上界说明噪声过大或模型失配。
 *
 *   5. EKF 本身不做任何重置决策；EkfTracker 读取 recent_nis_failures
 *      滑动窗口失败率（> 0.4 判为发散）后自行决定是否重置。
 */

#include "extended_kalman_filter.hpp"

#include <numeric>

namespace mv::tool {

// ── 构造函数 ─────────────────────────────────────────────────────────────────

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add)
    : x(x0), P(P0), x_add_(std::move(x_add)) {
  I_ = Eigen::MatrixXd::Identity(x0.rows(), x0.rows());

  // 初始化调试字段，方便 Foxglove 可视化时读取
  data["nis"] = 0.0;
  data["nis_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

// ── 预测步 ───────────────────────────────────────────────────────────────────

Eigen::VectorXd ExtendedKalmanFilter::Predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q) {
  // 线性预测是非线性预测的特殊情况：f(x) = F·x。
  // 委托给非线性版本，避免重复协方差更新代码。
  return Predict(F, Q, [&F](const Eigen::VectorXd& xk) { return F * xk; });
}

Eigen::VectorXd ExtendedKalmanFilter::Predict(
    const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f) {
  // 为什么先更新 P 再更新 x？
  //   f(x) 可能是非线性的，雅可比 F 是在当前 x̂ 处线性化得到的。
  //   若先用 f 更新 x，则 F 就不再是"当前 x̂"处的雅可比了，会引入额外误差。
  //   先用 F（当前线性化）更新 P，再用 f 推进 x，顺序一致。
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

// ── 更新步 ───────────────────────────────────────────────────────────────────

Eigen::VectorXd ExtendedKalmanFilter::Update(
    const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract) {
  // 线性更新是非线性更新的特殊情况：h(x) = H·x。
  return Update(
      z, H, R, [&H](const Eigen::VectorXd& xk) { return H * xk; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::Update(
    const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract) {
  // ── Step 1: 先验创新向量（在 x 更新前计算，用于 NIS）─────────────────────
  // ν = z − h(x̂⁻)
  // 必须用 z_subtract 而非直接相减，因为观测量中的角度分量需要 limit_rad 归一化，
  // 否则当真实角度与预测角度跨越 ±π 边界时，创新会出现 ±2π 的大跳变。
  const Eigen::VectorXd innovation = z_subtract(z, h(x));

  // ── Step 2: 创新协方差（先验 S，与 K 共用，避免重复计算）───────────────
  // S = H·P⁻·Hᵀ + R
  const Eigen::MatrixXd S = H * P * H.transpose() + R;

  // ── Step 3: 卡尔曼增益 ───────────────────────────────────────────────────
  // K = P⁻·Hᵀ·S⁻¹
  // 注：对小维度 S（一般 3×3 或 4×4）直接 .inverse() 即可，
  //     若未来观测维度变大可改为 S.llt().solve(H * P.transpose()).transpose()。
  const Eigen::MatrixXd K = P * H.transpose() * S.inverse();

  // ── Step 4: 后验状态（通过 x_add 处理角度分量周期性）───────────────────
  // x̂ = x_add(x̂⁻, K·ν)
  x = x_add_(x, K * innovation);

  // ── Step 5: 后验协方差（Joseph 稳定形式）────────────────────────────────
  // P = (I−KH)·P⁻·(I−KH)ᵀ + K·R·Kᵀ
  //
  // 为什么用 Joseph 形式而非简单的 P = (I−KH)·P⁻？
  //   简单形式在 K 存在数值误差时会破坏 P 的对称正定性，累积后
  //   P 可能出现负特征值，导致 EKF 发散。Joseph 形式无论 K 准不准
  //   都能保持 P 正定，代价是多一次矩阵乘法——在 11×11 矩阵上可接受。
  const Eigen::MatrixXd IKH = I_ - K * H;
  P = IKH * P * IKH.transpose() + K * R * K.transpose();

  // ── Step 6: NIS 卡方检验（先验创新 + 先验 S）────────────────────────────
  UpdateNis(innovation, S);

  return x;
}

// ── NIS 滑动窗口更新 ─────────────────────────────────────────────────────────

void ExtendedKalmanFilter::UpdateNis(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& S) {
  // NIS = νᵀ·S⁻¹·ν，服从 χ²(dim) 分布（若 EKF 模型正确）
  last_nis = (innovation.transpose() * S.inverse() * innovation).value();

  // 卡方分布 95% 置信区间上界，按观测维度动态查表（dim = 1 ~ 6）。
  // 超过上界意味着"噪声实际上比模型预设的要大"——即 Q/R 低估或模型失配。
  // 参考：https://en.wikipedia.org/wiki/Chi-squared_distribution#Table_of_%CF%872_values_vs_p-values
  static constexpr double kChi2_95[] = {
      0.0,     // [0] 占位
      3.841,   // [1] dof=1
      5.991,   // [2] dof=2
      7.815,   // [3] dof=3
      9.488,   // [4] dof=4
      11.070,  // [5] dof=5
      12.592,  // [6] dof=6
  };
  const int dim = static_cast<int>(innovation.size());
  const double threshold = (dim >= 1 && dim <= 6) ? kChi2_95[dim] : 7.815;

  // 滑动窗口：push 新帧结果，超出 window_size 时 pop 最旧帧
  recent_nis_failures.push_back(last_nis > threshold ? 1 : 0);
  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  // 统计近期失败率并写入 data 供调试/Foxglove 可视化读取
  const int failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  data["nis"] = last_nis;
  data["nis_fail"] = static_cast<double>(recent_nis_failures.back());
  data["recent_nis_failures"] =
      static_cast<double>(failures) / static_cast<double>(recent_nis_failures.size());
}

}  // namespace mv::tool
