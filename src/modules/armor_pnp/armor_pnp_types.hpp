#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <optional>

namespace mv::modules {

/** @brief 标识一次 PnP 解算所使用的二维角点来源。 */
enum class PnpInputSource : std::uint8_t {
  GROUND_TRUTH = 0,  ///< 同帧仿真真值投影角点。
  DETECTION = 1,     ///< 检测角点经精修或原子回退后的正式输入。
};

/** @brief 将输入来源转换为稳定的日志与 Foxglove 字段名称。 */
[[nodiscard]] const char* PnpInputSourceName(PnpInputSource source) noexcept;

/** @brief 一组输入四角的最终求解状态。 */
enum class PnpStatus : std::uint8_t {
  SUCCESS = 0,     ///< 至少一个候选通过约束并选出最小重投影误差解。
  INVALID_INPUT,   ///< 相机内参无效或输入角点包含非有限值。
  NO_SOLUTION,     ///< IPPE 未返回候选或 OpenCV 解算失败。
  NEGATIVE_DEPTH,  ///< 候选至少有一个物点位于相机后方。
  BACK_FACING,     ///< 候选装甲正面未朝向相机。
  OUT_OF_RANGE,    ///< 候选平移距离超出配置范围。
};

/** @brief 将求解状态转换为稳定的日志与统计字段名称。 */
[[nodiscard]] const char* PnpStatusName(PnpStatus status) noexcept;

/** @brief 一个通过几何约束的装甲位姿估计及其可选仿真真值误差。 */
struct ArmorPoseEstimate {
  PnpInputSource source{PnpInputSource::DETECTION};  ///< 本次二维输入来源。
  std::size_t input_index{0};             ///< 在当前帧对应来源数组中的索引。
  std::optional<std::uint64_t> truth_id;  ///< 匹配的仿真装甲 ID；无可靠匹配时为空。
  std::uint8_t label{0};                  ///< 检测标签或仿真真值标签的数值表示。
  hal::CameraFrame::ArmorType type{hal::CameraFrame::ArmorType::SMALL};  ///< PnP 物点尺寸类型。
  double width_m{0.0};                               ///< 本次解算使用的装甲物理宽度。
  double height_m{0.0};                              ///< 本次解算使用的装甲物理高度。
  geometry::RigidTransform camera_t_armor;           ///< armor 到 camera_optical 的变换。
  std::array<cv::Point2f, 4> image_corners{};        ///< TL、TR、BR、BL 顺序的输入角点。
  std::array<cv::Point2f, 4> reprojected_corners{};  ///< 所选位姿的模型重投影角点。
  std::size_t candidate_index{0};                    ///< solvePnPGeneric 返回的候选索引。
  std::optional<double> candidate_rmse_gap_px;       ///< 次优与最优候选的 RMSE 差。
  double reprojection_rmse_px{0.0};               ///< 模型重投影相对输入四角的 RMSE。
  double image_width_px{0.0};                     ///< 输入四边形上下边平均宽度。
  double image_height_px{0.0};                    ///< 输入四边形左右边平均高度。
  double distance_m{0.0};                         ///< 装甲原点到相机原点的距离。
  double viewing_angle_deg{0.0};                  ///< 装甲法向与指向相机方向的夹角。
  std::optional<double> truth_distance_m;         ///< 匹配真值的相机距离。
  std::optional<double> truth_viewing_angle_deg;  ///< 匹配真值的观察角。
  std::optional<hal::CameraFrame::ArmorType> truth_type;  ///< 匹配真值的装甲尺寸。
  std::optional<double> mean_corner_error_px;  ///< 输入角点到真值投影的平均距离。
  std::array<double, 4> corner_errors_px{};    ///< 四个输入角点到真值投影的距离。
  std::array<double, 4> corner_delta_u_px{};   ///< 输入角点减真值角点的水平偏差。
  std::array<double, 4> corner_delta_v_px{};   ///< 输入角点减真值角点的垂直偏差。
  std::optional<double> position_error_m;      ///< 估计与真值平移误差的欧氏距离。
  std::optional<std::array<double, 3>> position_error_camera_m;  ///< 相机系 XYZ 有符号误差。
  std::optional<double> depth_error_m;                           ///< 相机 Z 轴误差绝对值。
  std::optional<double> signed_depth_error_m;                    ///< 估计深度减真值深度。
  std::optional<double> rotation_error_deg;  ///< 估计与真值姿态的角距离。
  std::optional<double> position_jitter_m;  ///< 同一真值目标相邻连续帧的位置误差变化量。
};

/** @brief 一组输入角点的 PnP 尝试及其可选角点精修诊断。 */
struct ArmorPnpAttempt {
  PnpInputSource source{PnpInputSource::DETECTION};  ///< 输入角点来源。
  std::size_t input_index{0};                        ///< 来源数组中的索引。
  PnpStatus status{PnpStatus::INVALID_INPUT};        ///< 求解或候选拒绝状态。
  std::optional<ArmorPoseEstimate> estimate;         ///< 成功时的最优位姿估计。
  std::optional<CornerRefinementResult> refinement;  ///< 检测链对应的角点精修结果。
};

/** @brief 一组指标样本的数量、P50 与 P95 最近秩分位数。 */
struct PnpPercentiles {
  std::size_t samples{0};  ///< 有效样本数量。
  double p50{0.0};         ///< 第 50 百分位数。
  double p95{0.0};         ///< 第 95 百分位数。
};

/** @brief 单个 PnP 来源的累计精度与稳定性快照。 */
struct PnpSourceSummary {
  PnpPercentiles reprojection_rmse_px;  ///< 模型重投影 RMSE。
  PnpPercentiles mean_corner_error_px;  ///< 输入角点真值误差。
  PnpPercentiles position_error_m;      ///< 三维位置误差。
  PnpPercentiles depth_error_m;         ///< 深度绝对误差。
  PnpPercentiles rotation_error_deg;    ///< 姿态角距离误差。
  PnpPercentiles position_jitter_m;     ///< 连续帧位置误差变化量。
};

/** @brief 角点精修累计计数、耗时和精修前后二维误差快照。 */
struct CornerRefinementSummary {
  std::size_t attempted{0};                            ///< 累计接收的检测精修结果数。
  std::size_t succeeded{0};                            ///< 累计成功提交四角的精修数。
  std::size_t fallback{0};                             ///< 累计整体回退原始角点数。
  std::map<std::string, std::size_t> failure_reasons;  ///< 按稳定状态名聚合的回退次数。
  PnpPercentiles elapsed_ms;                           ///< 单次精修耗时。
  PnpPercentiles raw_mean_corner_error_px;             ///< 原始网络角点真值误差。
  PnpPercentiles final_mean_corner_error_px;           ///< 正式 PnP 输入角点真值误差。
};

/** @brief 正式检测链的累计 PnP 求解健康状态。 */
struct PnpSolveSummary {
  std::size_t attempted{0};           ///< 累计正式解算次数。
  std::size_t succeeded{0};           ///< 累计成功次数。
  std::size_t candidate_switches{0};  ///< 同一真值目标连续帧切换 IPPE 候选的次数。
  std::map<std::string, std::size_t> rejection_reasons;  ///< 按 PnpStatus 名称聚合的失败次数。
};

/** @brief 当前帧逐目标结果及同一序号下的全局累计统计快照。 */
struct ArmorPnpFrameResult {
  std::uint64_t summary_sequence{0};  ///< 全局与分组统计最近一次原子更新的帧序号。
  std::vector<ArmorPnpAttempt> attempts;  ///< 当前帧真值与正式检测链的逐目标尝试。
  PnpSourceSummary ground_truth_summary;  ///< 真值投影角点基准链累计指标。
  PnpSourceSummary detection_summary;     ///< 正式检测链累计指标。
  std::map<std::string, PnpSourceSummary> distance_groups;  ///< 按真值距离分组的检测指标。
  std::map<std::string, PnpSourceSummary> angle_groups;  ///< 按真值观察角分组的检测指标。
  std::map<std::string, PnpSourceSummary> size_groups;  ///< 按真值装甲尺寸分组的检测指标。
  PnpSolveSummary solve_summary;               ///< 正式检测链累计求解状态快照。
  CornerRefinementSummary refinement_summary;  ///< 角点精修累计状态快照。
};

/** @brief 将检测标签映射到物点尺寸；ONE 与 BASE_BIG 使用大装甲。 */
[[nodiscard]] hal::CameraFrame::ArmorType ArmorTypeForLabel(ArmorLabel label) noexcept;

}  // namespace mv::modules
