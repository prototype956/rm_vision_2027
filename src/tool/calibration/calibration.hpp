#pragma once

#include "hal/camera/i_camera.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <filesystem>
#include <opencv2/core.hpp>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace mv::tool::calibration {

/**
 * @brief 棋盘格采集、求解和质量验收参数。
 */
struct CalibrationSettings {
  int board_columns{9};              ///< 棋盘格横向内角点数量。
  int board_rows{6};                 ///< 棋盘格纵向内角点数量。
  double square_size_mm{25.0};       ///< 相邻内角点的实际距离，单位为毫米。
  int min_samples{20};               ///< 通过质量验收所需的最少有效样本数。
  double min_sharpness{100.0};       ///< 棋盘区域拉普拉斯方差的接纳下限。
  double max_rms_px{0.5};            ///< 全局重投影 RMS 上限，单位为像素。
  double max_view_rms_px{1.0};       ///< 单视图重投影 RMS 上限，单位为像素。
  double min_area_ratio{2.0};        ///< 最大与最小棋盘投影面积的最小倍率。
  int min_tilted_views{4};           ///< 每个方向所需的最少倾斜视图数。
  double min_tilt_ratio{1.15};       ///< 对边长度达到该比值时计为倾斜视图。
  std::filesystem::path output_dir;  ///< 会话输出目录的规范化绝对路径。
};

/**
 * @brief 从单帧图像提取的棋盘检测结果和采集质量指标。
 */
struct FrameObservation {
  bool found{false};                  ///< 是否检测到完整的棋盘内角点。
  bool sharp_enough{false};           ///< 清晰度是否达到采集阈值。
  double sharpness{0.0};              ///< 棋盘区域的拉普拉斯方差。
  double projected_area{0.0};         ///< 四个最外侧角点围成的像素面积。
  double horizontal_tilt_ratio{1.0};  ///< 左右边长度比，用于衡量水平透视倾斜。
  double vertical_tilt_ratio{1.0};    ///< 上下边长度比，用于衡量垂直透视倾斜。
  std::vector<cv::Point2f> corners;   ///< OpenCV 棋盘顺序排列的亚像素角点。
};

/**
 * @brief 已接纳并写入会话目录的标定样本。
 */
struct CalibrationSample {
  std::size_t id{0};                  ///< 会话内单调递增且不会复用的样本编号。
  std::filesystem::path image_path;   ///< 相对于会话目录的原图路径。
  bool active{true};                  ///< 是否参与后续求解和质量验收。
  double sharpness{0.0};              ///< 接纳时记录的棋盘区域清晰度。
  double projected_area{0.0};         ///< 接纳时记录的棋盘投影面积。
  double horizontal_tilt_ratio{1.0};  ///< 接纳时记录的水平倾斜比。
  double vertical_tilt_ratio{1.0};    ///< 接纳时记录的垂直倾斜比。
  std::vector<cv::Point2f> corners;   ///< 参与求解的棋盘图像坐标。
};

/**
 * @brief 有效样本对画面位置、距离和倾斜姿态的覆盖统计。
 */
struct CoverageMetrics {
  std::array<bool, 9> grid_cells{};  ///< 画面 3x3 网格中是否至少落入一个角点。
  int occupied_grid_cells{0};        ///< grid_cells 中已覆盖的网格数量。
  double area_ratio{0.0};            ///< 最大与最小棋盘投影面积之比。
  int horizontal_tilted_views{0};    ///< 达到水平倾斜阈值的有效样本数。
  int vertical_tilted_views{0};      ///< 达到垂直倾斜阈值的有效样本数。
};

/**
 * @brief 内参求解结果及全部质量验收信息。
 */
struct CalibrationResult {
  bool solved{false};               ///< OpenCV 是否成功完成内参求解。
  bool accepted{false};             ///< 求解和所有质量条件是否均通过。
  cv::Mat camera_matrix;            ///< 3x3 针孔相机内参矩阵，类型为 CV_64F。
  cv::Mat distortion_coefficients;  ///< [k1, k2, p1, p2, k3] 畸变系数。
  double rms_px{0.0};               ///< OpenCV 返回的全局重投影 RMS。
  double max_view_rms_px{0.0};      ///< 所有有效样本中的最大单视图 RMS。
  std::size_t worst_sample_id{0};   ///< 最大单视图 RMS 对应的样本编号。
  std::vector<std::size_t> active_sample_ids;  ///< 与 per_view_rms_px 同序的样本编号。
  std::vector<double> per_view_rms_px;         ///< 各有效样本的重投影 RMS。
  CoverageMetrics coverage;                    ///< 求解时有效样本的姿态覆盖统计。
  std::vector<std::string> failures;           ///< 未通过的质量条件或求解错误。
};

/**
 * @brief 解析并严格校验相机标定配置。
 *
 * @param root 已由 ConfigLoader 加载的 YAML 根节点。
 * @param project_root 项目根目录，用于解析相对输出目录。
 * @return 完成值域检查和路径解析的标定参数。
 * @throws ConfigError 配置缺失、包含未知键或字段值超出允许范围。
 */
CalibrationSettings ParseCalibrationSettings(const YAML::Node& root,
                                             const std::filesystem::path& project_root);

/**
 * @brief 管理棋盘观测、样本状态、内参求解和质量验收。
 *
 * 类不负责相机采集和原图写盘，调用方需确保传入图像路径与实际文件一致。
 */
class CameraCalibrator final {
 public:
  /**
   * @brief 使用固定参数和图像尺寸创建会话内标定器。
   * @throws std::invalid_argument 图像宽度或高度非正数。
   */
  CameraCalibrator(CalibrationSettings settings, cv::Size image_size);

  /**
   * @brief 检测棋盘角点并计算清晰度、面积及透视倾斜指标。
   * @param bgr_image 与构造尺寸一致的 CV_8UC3 图像。
   * @throws std::invalid_argument 图像为空、尺寸不匹配或不是 BGR8。
   */
  [[nodiscard]] FrameObservation Observe(const cv::Mat& bgr_image) const;
  /** @brief 判断观测的位置、尺度和姿态是否与现有有效样本过于接近。 */
  [[nodiscard]] bool IsLikelyDuplicate(const FrameObservation& observation) const;
  /** @brief 接纳完整且清晰的观测；成功时分配新样本编号并返回 true。 */
  bool AddSample(const FrameObservation& observation, const std::filesystem::path& image_path);
  /** @brief 将最近一个有效样本标为排除，返回其编号；无有效样本时返回空。 */
  [[nodiscard]] std::optional<std::size_t> UndoLastSample();
  /** @brief 使用所有有效样本求解内参，并执行姿态覆盖及重投影误差验收。 */
  [[nodiscard]] CalibrationResult Solve() const;

  /** @brief 获取当前会话使用的只读标定参数。 */
  [[nodiscard]] const CalibrationSettings& Settings() const noexcept;
  /** @brief 获取包含已排除条目的完整样本历史。 */
  [[nodiscard]] const std::vector<CalibrationSample>& Samples() const noexcept;
  /** @brief 获取当前参与求解的有效样本数。 */
  [[nodiscard]] std::size_t ActiveSampleCount() const noexcept;

 private:
  /** @brief 汇总指定样本的 3x3 位置、尺度和双向倾斜覆盖情况。 */
  [[nodiscard]] CoverageMetrics AnalyzeCoverage(
      const std::vector<const CalibrationSample*>& samples) const;

  CalibrationSettings settings_;            ///< 会话期间保持不变的标定与验收参数。
  cv::Size image_size_;                     ///< 所有观测必须匹配的固定图像尺寸。
  std::vector<CalibrationSample> samples_;  ///< 包含有效和已排除样本的追加式历史。
  std::size_t next_sample_id_{1};  ///< 下一个样本编号，排除样本后也不回退。
};

/**
 * @brief 原子写入可恢复采集状态、样本明细和可选求解结果。
 * @param path session.yaml 的目标路径。
 * @param status collecting、passed、failed 或 aborted 会话状态。
 * @throws std::runtime_error 目录创建或文件写入失败。
 */
void WriteSession(const std::filesystem::path& path, const CalibrationSettings& settings,
                  const hal::CameraInfo& camera_info, const std::vector<CalibrationSample>& samples,
                  const std::optional<CalibrationResult>& result, const std::string& status);

/**
 * @brief 原子写入通过验收的五参数针孔相机内参。
 * @throws std::invalid_argument 结果未通过验收或矩阵格式不符合要求。
 * @throws std::runtime_error 目录创建或文件写入失败。
 */
void WriteIntrinsics(const std::filesystem::path& path, const CalibrationSettings& settings,
                     const hal::CameraInfo& camera_info, const CalibrationResult& result);

}  // namespace mv::tool::calibration
