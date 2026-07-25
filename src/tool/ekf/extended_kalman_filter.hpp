/**
 * @file extended_kalman_filter.hpp
 * @brief 扩展卡尔曼滤波器（EKF）工具类
 *
 * 【为什么需要独立的 EKF 工具类？】
 *   EkfPredictor 的核心算法（11 维状态估计、NIS 收敛检验）需要一个通用的
 *   非线性卡尔曼滤波器原语。将 EKF 抽取为独立工具有以下好处：
 *   1. EkfTrackTarget（目标 EKF）和未来可能的 BuffPredictor（打符 EKF）
 *      共享同一套数值稳定的实现，不重复造轮子；
 *   2. EKF 核心运算（predict/update）可单独单测，不依赖任何业务数据结构；
 *   3. Pimpl 隔离 Eigen 矩阵的大型头文件，外层模块只 include 此头文件即可，
 *      不会因 Eigen 版本差异导致编译时间膨胀。
 *
 * 【算法说明】
 *   实现标准 EKF（一阶线性化近似）：
 *     Predict : x̂⁻ = F·x̂   (or f(x̂) with nonlinear f)
 *               P⁻  = F·P·Fᵀ + Q
 *     Update  : K   = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
 *               x̂   = x̂⁻ + K·(z − h(x̂⁻))
 *               P   = (I − K·H)·P⁻
 *
 *   角度状态分量需要特殊的"加法"定义（防止 2π 折叠），
 *   通过 x_add 函数对象注入，而不是硬编码在 EKF 内部——
 *   这样 EKF 本身对状态向量的物理含义保持无知，保持通用性。
 *
 * 【NIS 收敛检验（Normalized Innovation Squared）】
 *   NIS = νᵀ·S⁻¹·ν，其中 ν = z − h(x̂⁻)，S = H·P⁻·Hᵀ + R
 *   - NIS 服从卡方分布，自由度 = 观测维度；
 *   - 维护滑动窗口（window_size = 100 帧），统计 NIS 超出 95% 置信区间的比例；
 *   - 若失败率 > 40%，说明 EKF 已发散，EkfTracker 据此重置目标。
 *   - 不在 EKF 内部直接重置，由外部（EkfTracker）决策——这符合单一职责原则：
 *     EKF 只"感知"自己是否收敛，不"决定"如何处理。
 *
 * 【线程安全】
 *   非线程安全。EkfPredictor 在单一检测线程调用，无并发访问。
 */
#pragma once

#include <deque>
#include <functional>
#include <map>

#include <Eigen/Dense>

namespace mv::tool {

/**
 * @brief 通用扩展卡尔曼滤波器
 *
 * 模板化的状态维度由构造时传入的向量/矩阵大小动态确定（动态大小 Eigen）。
 *
 * 典型初始化（11 维装甲车 EKF）：
 * @code
 *   Eigen::VectorXd x0(11);   // 初始状态
 *   Eigen::MatrixXd P0(11,11); // 初始协方差
 *
 *   // 角度分量（index 6）需要 limit_rad 防止 2π 折叠
 *   auto x_add = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
 *     auto c = a + b;
 *     c[6] = limit_rad(c[6]);
 *     return c;
 *   };
 *
 *   mv::tool::ExtendedKalmanFilter ekf(x0, P0, x_add);
 * @endcode
 */
class ExtendedKalmanFilter {
 public:
  // ── 公开状态（供 EkfTrackTarget 读取诊断信息）──────────────────────────

  Eigen::VectorXd x;  ///< 当前后验状态估计向量
  Eigen::MatrixXd P;  ///< 当前后验协方差矩阵

  // ── NIS 统计（供 EkfTracker 决策是否重置）──────────────────────────────

  /** NIS 统计附属数据（卡方统计值、自由度等，供调试/Foxglove 可视化）*/
  std::map<std::string, double> data;

  /**
   * 最近 window_size 帧的 NIS 失败记录（0=通过，1=失败）
   * std::deque 方便 push_back / pop_front 滑动窗口操作。
   */
  std::deque<int> recent_nis_failures{0};

  /** 滑动窗口大小，默认 100 帧 */
  size_t window_size{100};

  /** 最后一帧的 NIS 值（调试输出用）*/
  double last_nis{0.0};

  // ── 构造 / 析构 ─────────────────────────────────────────────────────────

  ExtendedKalmanFilter() = default;

  /**
   * @brief 构造并初始化 EKF
   *
   * @param x0     初始状态向量（维度决定后续调用的矩阵大小）
   * @param P0     初始协方差矩阵（须与 x0 维度匹配）
   * @param x_add  状态"加法"函数（默认为普通加法）。
   *               含角度分量时须传入自定义函数对 angle 做 limit_rad 归一化，
   *               防止角度状态在迭代中溢出 [-π, π] 导致 EKF 发散。
   */
  explicit ExtendedKalmanFilter(
      const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
      std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add =
          [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a + b; });

  // ── 预测步（时间更新）───────────────────────────────────────────────────

  /**
   * @brief 线性预测步（状态转移矩阵 F 已知）
   *
   * x̂⁻ = F·x̂，P⁻ = F·P·Fᵀ + Q
   *
   * 用于近似线性的状态转移（如 CV 模型中的匀速直线假设）。
   *
   * @param F   线性状态转移矩阵（n×n）
   * @param Q   过程噪声协方差矩阵（n×n，分段白噪声模型）
   * @return 预测后的状态向量（与 x 同步更新）
   */
  Eigen::VectorXd Predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

  /**
   * @brief 非线性预测步（显式传入非线性状态转移函数 f）
   *
   * x̂⁻ = f(x̂)，P⁻ = F·P·Fᵀ + Q
   * F 仍需调用方提供（f 在当前点的雅可比矩阵）。
   *
   * @param F   f 在当前 x 处的雅可比（数值或解析）
   * @param Q   过程噪声协方差
   * @param f   非线性状态转移函数 x_{k+1} = f(x_k)
   */
  Eigen::VectorXd Predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q,
                          std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f);

  // ── 更新步（观测更新）───────────────────────────────────────────────────

  /**
   * @brief 线性观测更新步
   *
   * K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
   * x̂ = x̂⁻ + K·(z − H·x̂⁻)
   * P = (I − K·H)·P⁻
   *
   * @param z           观测向量（m 维）
   * @param H           线性观测矩阵（m×n）
   * @param R           观测噪声协方差矩阵（m×m）
   * @param z_subtract  观测"减法"函数（默认普通减法）。
   *                    若观测量中含角度（如 yaw），需传入带 limit_rad 的版本，
   *                    防止创新量 ν = z − h(x) 因 2π 折叠产生大跳变。
   */
  Eigen::VectorXd Update(
      const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
      std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract =
          [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a - b; });

  /**
   * @brief 非线性观测更新步（显式传入观测函数 h）
   *
   * K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
   * x̂ = x̂⁻ + K·(z − h(x̂⁻))
   * H 须调用方提供（h 在预测状态处的雅可比）。
   *
   * 装甲板 EKF 中观测函数 h(x) = [x_armor, y_armor, z_armor] 是非线性的
   * （由旋转参数推算装甲板 xyz），因此使用此版本。
   *
   * @param z          观测向量
   * @param H          h 在 x̂⁻ 处的雅可比（解析推导，见 EkfTrackTarget）
   * @param R          观测噪声协方差
   * @param h          非线性观测函数
   * @param z_subtract 观测减法（同线性版说明）
   */
  Eigen::VectorXd Update(
      const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
      std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
      std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract =
          [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a - b; });

 private:
  Eigen::MatrixXd I_;  ///< 单位矩阵缓存（避免每帧重建）

  /** 状态加法函数（注入点在构造函数，处理角度分量的周期性）*/
  std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add_;

  /** NIS 卡方检验：更新 recent_nis_failures 滑动窗口 */
  void UpdateNis(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& S);
};

}  // namespace mv::tool
