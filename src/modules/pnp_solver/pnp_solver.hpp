/**
 * @file pnp_solver.hpp
 * @brief 装甲板平面目标 PnP 解算器（单解 IPPE + 相机到云台外参变换）。
 * 输入：角点像素坐标（px）与装甲类型；输出：云台系位置（m）与 yaw/pitch（rad）。
 * 坐标、单位、YAML 字段与选解细则见 docs/modules/pnp/pnp_solver.md。
 * 工厂键："pnp"。
 */
#pragma once

#include "interfaces/i_solver.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace mv::modules {

/**
 * @brief 基于 IPPE 解析解的装甲板位姿解算器
 *
 * 实现 ISolver 接口，采用 solvePnP(IPPE) 单解策略。
 * 单实例无历史解状态；实例间状态互不共享。
 * @thread_safety Not thread-safe
 */
class PnpSolver final : public ISolver {
 public:
  PnpSolver();
  ~PnpSolver() override;

  PnpSolver(const PnpSolver&) = delete;
  PnpSolver& operator=(const PnpSolver&) = delete;
  PnpSolver(PnpSolver&&) = delete;
  PnpSolver& operator=(PnpSolver&&) = delete;

  /**
   * @brief 加载相机内参与可选外参，进入可解算状态。
   * @param config 配置根节点，需包含 calibration.camera_matrix。
   * @return true   内参解析成功，对象进入就绪状态
   * @return false  缺少 calibration 节点或 camera_matrix 维度错误，
   *               错误原因已通过日志输出，对象保持未初始化状态
   *
   * @note 外参（R_camera_to_gimbal / t_camera_to_gimbal）可选：
   *       不提供时自动回退为 Y 轴翻转简化外参，与旧版行为兼容。
   * @thread_safety Not thread-safe
   */
  bool Init(const YAML::Node& config) override;

  /**
   * @brief 对单块装甲板执行 PnP 解算，就地填写位姿字段
   *
   * 问题定义：给定平面装甲板四角点像素坐标 \f$\mathbf{u}_i\f$（px）与
   * 世界模板点 \f$\mathbf{X}_i\f$（m），求解相机位姿 \f$(\mathbf{R},\mathbf{t})\f$。
   * 核心投影模型：
   * \f[
   * s\,\mathbf{u}_i = \mathbf{K}\left(\mathbf{R}\mathbf{X}_i + \mathbf{t}\right)
   * \f]
   * 读取 detection.points（4 个图像角点）和 detection.type（决定世界坐标模板尺寸），
   * 调用 solvePnP(IPPE) 获取单解后写入：
   *   - detection.xyz_in_gimbal    云台坐标系三维位置（m）
   *   - detection.yaw_angle        水平角（rad，顺时针为正）
   *   - detection.pitch_angle      俯仰角（rad，向下为正）
   *   - detection.reprojected_points  主解重投影角点（4 点）
   *   - detection.reproj_error     主解重投影均方根误差（px）
   *   - detection.has_alt_solution  单解策略下恒为 false
   *   - detection.xyz_in_gimbal_alt / reprojected_points_alt / reproj_error_alt 在每帧复位
   *
   * @param detection  输入/输出，需预先填好 points / type
   * @return true      解算成功，detection.is_solved 被置为 true
   * @return false     解算失败（Init() 未调用，或 solvePnP 返回 false）
   * @thread_safety Not thread-safe
   */
  bool Solve(Detection& detection) override;

  /**
   * @brief 设置云台到世界坐标系的旋转矩阵。
   * @param quaternion IMU 提供的姿态四元数。
   * @thread_safety Not thread-safe
   */
  void SetGimbalToWorldRotation(const Eigen::Quaterniond& quaternion);

  /**
   * @brief 获取云台到世界坐标系旋转矩阵。
   * @return 3x3 旋转矩阵。
   * @thread_safety Not thread-safe
   */
  [[nodiscard]] Eigen::Matrix3d GetGimbalToWorldRotation() const;

  /**
   * @brief 设置云台到世界坐标系的平移向量。
   * @param quaternion IMU 提供的姿态四元数。
   * @thread_safety Not thread-safe
   */
  void SetGimbalToWorldTranslation(const Eigen::Quaterniond& quaternion);

  /**
   * @brief 注入云台姿态（IMU 四元数），用于世界系相关计算
   * @param quaternion 云台坐标系到世界坐标系旋转四元数
   */
  void SetGimbalOrientation(const Eigen::Quaterniond& quaternion) override;

  /**
   * @brief 获取云台到世界坐标系平移向量。
   * @return 3x1 平移向量（m）。
   * @thread_safety Not thread-safe
   */
  [[nodiscard]] Eigen::Vector3d GetGimbalToWorldTranslation() const;

  /**
   * @brief 是否已完成 Init() 初始化
   *
   * 在 Solve() 调用前可用此方法检查对象是否就绪，避免未初始化时调用产生错误日志。
   * @thread_safety Not thread-safe
   */
  [[nodiscard]] bool IsInitialized() const noexcept override { return initialized_; }

  /**
   * @brief 在给定区间遍历 yaw，寻找重投影误差最小的 yaw 并更新 detection
   * @param detection 输入/输出，需先由 Solve 填充 xyz_in_gimbal
   * @param yaw_min 最小 yaw（弧度）
   * @param yaw_max 最大 yaw（弧度）
   * @param step 步长（弧度）
   * @thread_safety Not thread-safe
   */
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    void OptimizeYaw(Detection& detection, double yaw_min, double yaw_max, double step);
    // NOLINTEND(bugprone-easily-swappable-parameters)

  /**
   * @brief 计算指定 yaw/pitch 下的重投影误差
   *
   * 问题定义：在给定 \f$(yaw, pitch)\f$ 下，将装甲板模板点投影回图像并与观测角点比较。
   * 当 inclined \f$>0\f$ 时使用 SJTUCost（像素差 + 边方向差），否则返回 RMS 误差：
   * \f[
   * e_{rms}=\sqrt{\frac{1}{4}\sum_{i=1}^{4}\|\mathbf{u}_i-\hat{\mathbf{u}}_i\|^2}
   * \f]
   * 其中 \f$\mathbf{u}_i\f$ 为检测角点（px），\f$\hat{\mathbf{u}}_i\f$ 为投影角点（px）。
   *
   * @param detection 目标检测结构体（需包含 points 和 type，以及已由 Solve 填写的 xyz_in_gimbal）
   * @param yaw 指定 yaw 角（rad）
   * @param pitch 指定 pitch 角（rad）
   * @param inclined 倾斜角（用于 SJTUCost，加权角度/像素误差），若 <= 0 则返回 RMS
   * @return 重投影误差（px）；若未初始化或投影失败返回 inf
   * @thread_safety Not thread-safe
   */
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    [[nodiscard]] double ArmorReprojectionError(const Detection& detection, float yaw, float pitch,
                                                                                            float inclined) const;
    // NOLINTEND(bugprone-easily-swappable-parameters)

  /**
   * @brief 使用外部给定的世界到相机变换，将三维点投影到像素平面。
   * @param world_pts 世界坐标点集合（m）。
   * @param R_world2camera 世界系到相机系旋转矩阵。
   * @param t_world2camera 世界系到相机系平移向量（m）。
   * @return 可见点像素坐标（px），当全部点位于相机后方时返回空。
   * @thread_safety Not thread-safe
   */
  [[nodiscard]] std::vector<cv::Point2f> WorldToPixel(const std::vector<cv::Point3f>& world_pts,
                                                      const Eigen::Matrix3d& r_world2camera,
                                                      const Eigen::Vector3d& t_world2camera) const;

 private:
  /**
   * @brief 给定目标中心世界坐标与 yaw，重投影装甲板四角点。
   * @param xyz_in_world 目标中心世界坐标（m）。
   * @param yaw 目标航向角（rad）。
   * @param type 装甲板类型（决定宽度模板）。
   * @return 四角点像素坐标（px），顺序为 BL/BR/TR/TL。
   * @thread_safety Not thread-safe
   */
  [[nodiscard]] std::vector<cv::Point2f> ReprojectArmor(const Eigen::Vector3d& xyz_in_world,
                                                        double yaw, ArmorType type) const;

  /**
   * @brief 计算给定 yaw 下的四角点重投影 RMS。
   * @param xyz_in_world 目标中心世界坐标（m）。
   * @param yaw 目标航向角（rad）。
   * @param type 装甲板类型。
   * @param img_pts 检测角点（px）。
   * @param out_proj 可选输出，写入本次投影角点（px）。
   * @return RMS 重投影误差（px）；投影异常时返回 inf。
   * @thread_safety Not thread-safe
   */
  [[nodiscard]] double ArmorReprojectionRms(const Eigen::Vector3d& xyz_in_world, double yaw,
                                            ArmorType type, const std::vector<cv::Point2f>& img_pts,
                                            std::array<cv::Point2f, 4>* out_proj) const;

  /**
   * @brief SJTU 角点代价函数（像素距离 + 边方向差）。
   *
   * 对每条边构造像素差 \f$d_p\f$ 与角度差 \f$d_a\f$，按倾角 \f$\theta\f$ 融合：
   * \f[
   * cost_i = \sqrt{(d_p\sin\theta)^2 + 2(d_a\cos\theta)^2},\quad
   * cost=\sum_i cost_i
   * \f]
   *
   * @param cv_refs 参考角点（通常为投影角点，px）。
   * @param cv_pts 待评估角点（通常为检测角点，px）。
   * @param inclined 倾角权重 \f$\theta\f$（rad）。
   * @return 匹配代价，值越小表示匹配越好。
   * @thread_safety Not thread-safe
   */
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    [[nodiscard]] static double SJTUCost(const std::vector<cv::Point2f>& cv_refs,
                                                                             const std::vector<cv::Point2f>& cv_pts,
                                                                             const double& inclined);
    // NOLINTEND(bugprone-easily-swappable-parameters)

  // ── 内参 ──────────────────────────────────────────────────────────────────
  cv::Mat camera_matrix_;  ///< 3×3，CV_64F
  cv::Mat dist_coeffs_;    ///< 1×5，CV_64F

  // ── 外参（相机 → 云台坐标系）────────────────────────────────────────────
  // 默认：Y 轴翻转（相机 down+Y → 云台 up+Y），等效于旧版简化实现
  // 提供 calibration.R_camera_to_gimbal / t_camera_to_gimbal 后可使用精确外参
  Eigen::Matrix3d R_camera2gimbal_{
      (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0).finished()};
  Eigen::Vector3d t_camera2gimbal_{Eigen::Vector3d::Zero()};

  //云台→imu坐标系的旋转矩阵
  Eigen::Matrix3d R_gimbal2imubody_{Eigen::Matrix3d::Identity()};
  //云台→imu坐标系的平移向量（单位：米）
  Eigen::Vector3d t_gimbal2imubody_{Eigen::Vector3d::Zero()};

  //云台→世界坐标系的旋转矩阵
  Eigen::Matrix3d R_gimbal2world_{Eigen::Matrix3d::Identity()};
  //云台→世界坐标系的平移向量（单位：米）
  Eigen::Vector3d t_gimbal2world_{Eigen::Vector3d::Zero()};

  // ── 世界坐标模板 ──────────────────────────────────────────────────────────
  // 默认值与 vision.yaml armor 节点一致；Init() 读取 yaml 后覆盖。
  float small_half_w_{0.0675F};  ///< 小装甲板半宽（m）
  float big_half_w_{0.115F};     ///< 大装甲板半宽（m）
  float half_h_{0.0275F};        ///< 装甲板半高，大小装甲相同（m）

  bool initialized_{false};
};

}  // namespace mv::modules
