#pragma once

#include "geometry/gimbal_geometry.hpp"
#include "tool/calibration/calibration.hpp"

namespace mv::tool::calibration {

/** @brief 采集时冻结的针孔内参；不允许离线解算使用当前配置替换。 */
struct HandeyeIntrinsics {
  cv::Size size;
  int roi_x{0};
  int roi_y{0};
  cv::Mat matrix;
  cv::Mat distortion;
};
HandeyeIntrinsics ParseHandeyeIntrinsics(const YAML::Node& root);

/** @brief 审核通过的一组板到相机位姿与同步云台姿态；平移统一为米。 */
struct HandeyeObservation {
  std::size_t id{0};
  cv::Mat camera_r_board;  ///< 实际方向为 board -> camera（PnP 输出）。
  cv::Mat camera_t_board;  ///< 棋盘原点在相机系的位置，米。
  cv::Mat world_r_gimbal;  ///< gimbal -> world，世界原点位于固定转轴交点。
  double reprojection_rms_px{0};
};

struct HandeyeResult {
  bool solved{false};  ///< 仅表示非退化数值求解成功，不代表实机精度验收通过。
  cv::Mat gimbal_r_camera;
  cv::Mat gimbal_t_camera;  ///< 相机光学中心在云台系的位置，米。
  YAML::Node report;
};

/** @brief 单 yaw、固定中心的手眼求解器；不持有硬件，不执行内参标定。 */
class HandeyeCalibrator final {
 public:
  HandeyeCalibrator(CalibrationSettings settings, HandeyeIntrinsics intrinsics,
                    geometry::GimbalExtrinsics extrinsics);
  /** @brief 对人工确认方向的角点执行 IPPE；失败返回原因，不添加样本。 */
  bool Add(std::size_t id, const std::vector<cv::Point2f>& corners,
           const geometry::Quaternion& world_q_imu, std::string& failure);
  [[nodiscard]] HandeyeResult Solve() const;
  [[nodiscard]] const std::vector<HandeyeObservation>& Observations() const {
    return observations_;
  }

 private:
  CalibrationSettings settings_;
  HandeyeIntrinsics intrinsics_;
  geometry::GimbalExtrinsics extrinsics_;
  std::vector<cv::Point3f> board_points_;  ///< 棋盘原点为审核图中的 0 号角点。
  std::vector<HandeyeObservation> observations_;
};

/** @brief 行主序展开 OpenCV double 矩阵，供报告及部署 YAML 共用。 */
std::vector<double> HandeyeMatrixValues(const cv::Mat& matrix);

}  // namespace mv::tool::calibration
