/**
 * @file calibration_io.hpp
 * @brief 标定工具公共 I/O 与图案检测辅助接口
 *
 * 【职责边界】
 *   - 负责图案类型解析、角点检测、世界点模板生成；
 *   - 负责 `vision.yaml` 中 `calibration` 节点的读取与增量写回；
 *   - 不负责相机采集、PnP 求解和手眼标定算法本体。
 *
 * 【设计约定】
 *   - 只写 EX 的 `calibration` 字段，不覆盖其他配置节点；
 *   - 平移单位统一为米（m），角度相关值由调用方保证单位一致；
 *   - 写回接口采用结构体参数，避免多指针参数顺序误传。
 */
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace mv::tool::calibration {

/**
 * @brief 标定板类型
 */
enum class PatternType {
  CHESSBOARD,
  CIRCLES_GRID,
};

/**
 * @brief 标定板几何配置
 */
struct PatternConfig {
  PatternType type{PatternType::CHESSBOARD};
  int cols{10};
  int rows{7};
  double spacing_mm{40.0};
};

/**
 * @brief 解析标定板类型字符串
 */
PatternType ParsePatternType(const std::string& text);

/**
 * @brief 标定板类型转字符串（用于日志输出）
 */
std::string PatternTypeToString(PatternType type);

/**
 * @brief 按配置生成标定板 3D 模板点（单位：mm）
 */
std::vector<cv::Point3f> BuildPatternObjectPoints(const PatternConfig& cfg);

/**
 * @brief 识别棋盘格/圆点板角点
 *
 * @param image        输入图像
 * @param cfg          标定板配置
 * @param corners      输出角点
 * @param debug_image  可选输出，用于绘制角点可视化
 */
bool DetectPatternCorners(const cv::Mat& image, const PatternConfig& cfg,
                          std::vector<cv::Point2f>* corners, cv::Mat* debug_image = nullptr);

/**
 * @brief 将 Mat 展平成行优先 vector<double>
 */
std::vector<double> MatToRowMajorVector(const cv::Mat& mat);

/**
 * @brief 加载 vision.yaml
 */
bool LoadVisionYaml(const std::string& vision_yaml_path, YAML::Node* root);

/**
 * @brief 写回 vision.yaml 中 calibration 节点的增量更新项
 */
struct VisionCalibrationUpdate {
  const cv::Mat* camera_matrix{nullptr};
  const cv::Mat* distort_coeffs{nullptr};
  const cv::Mat* r_camera_to_gimbal{nullptr};
  const cv::Mat* t_camera_to_gimbal{nullptr};
  const cv::Mat* r_gimbal_to_imu{nullptr};
};

bool UpdateVisionCalibration(const std::string& vision_yaml_path,
                             const VisionCalibrationUpdate& update);

/**
 * @brief calibration 节点读取结果
 */
struct CalibrationData {
  cv::Mat camera_matrix;
  cv::Mat distort_coeffs;
  Eigen::Matrix3d r_gimbal_to_imu{Eigen::Matrix3d::Identity()};
};

bool ReadCalibrationNode(const YAML::Node& root, CalibrationData* data);

}  // namespace mv::tool::calibration
