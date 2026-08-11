#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <span>
#include <yaml-cpp/yaml.h>

namespace mv::modules {

/** @brief JLU 灰度矩与亮度梯度角点精修参数。 */
struct ArmorCornerRefinerConfig {
  bool enabled{true};                   ///< 是否启用角点精修。
  int pass_optimize_lightbar_width{3};  ///< 不执行精修的最大估计灯条宽度，单位为像素。
  double normalize_max_brightness{25.0};      ///< 灰度矩计算前归一化的亮度上界。
  double lightbar_min_mean_brightness{30.0};  ///< 接受灯条 ROI 所需的最小原始平均灰度。
  double padding_scale{0.07};  ///< 灯条旋转矩形外接 ROI 的相对扩张比例。
  double search_start_ratio{0.40};  ///< 端点搜索起点到灰度质心的距离占灯条长度比例。
  double search_end_ratio{0.60};  ///< 端点搜索终点到灰度质心的距离占灯条长度比例。
};

/**
 * @brief 解析并严格校验角点精修配置。
 * @throws ConfigError 配置缺失、包含未知键或参数值无效。
 */
[[nodiscard]] ArmorCornerRefinerConfig ParseArmorCornerRefinerConfig(const YAML::Node& root);

/** @brief 单块装甲角点精修的最终状态。 */
enum class CornerRefinementStatus : std::uint8_t {
  SUCCESS = 0,       ///< 两侧灯条的四个端点均已精修。
  DISABLED,          ///< 配置关闭精修，直接使用原始角点。
  INVALID_IMAGE,     ///< 输入不是非空 CV_8UC1 灰度图。
  INVALID_GEOMETRY,  ///< 输入角点非有限值或灯条长度退化。
  LIGHT_TOO_NARROW,  ///< 估计灯条宽度不足以执行多线扫描。
  ROI_INVALID,       ///< 灯条 ROI 与图像相交后为空。
  LIGHT_TOO_DARK,    ///< 灯条 ROI 原始平均灰度未达到阈值。
  MOMENTS_ZERO,      ///< 归一化 ROI 的零阶矩为零。
  PCA_DEGENERATE,    ///< 二阶中心矩无法给出有效主轴。
  TOP_NOT_FOUND,     ///< 至少一个灯条未找到顶部亮度下降边缘。
  BOTTOM_NOT_FOUND,  ///< 至少一个灯条未找到底部亮度下降边缘。
};

/** @brief 将精修状态转换为稳定的日志与统计字段名称。 */
[[nodiscard]] const char* CornerRefinementStatusName(CornerRefinementStatus status) noexcept;

/** @brief 单个灯条端点的梯度搜索诊断。 */
struct EndpointRefinementDiagnostic {
  cv::Point2f original{};                    ///< 网络输出的原始角点。
  cv::Point2f candidate{};                   ///< 各有效扫描线候选的算术平均点。
  cv::Point2f final{};                       ///< 原子回退处理后的最终角点。
  cv::Point2f search_start{};                ///< 灰度质心主轴上的搜索起点。
  cv::Point2f search_end{};                  ///< 灰度质心主轴上的搜索终点。
  std::vector<cv::Point2f> scan_candidates;  ///< 每条有效扫描线找到的最大亮度下降点。
  bool found{false};                         ///< 是否至少存在一条有效扫描线候选。
  bool applied{false};  ///< 候选是否随整块装甲精修成功而正式采用。
};

/** @brief 单个灯条的灰度矩主轴与端点精修诊断。 */
struct LightbarRefinementDiagnostic {
  cv::Point2f center{};         ///< 初始几何中心或成功计算后的灰度质心。
  cv::Point2f axis{};           ///< 统一指向图像上方的单位主轴。
  cv::Point2f top{};            ///< 原始或成功精修后的顶部端点。
  cv::Point2f bottom{};         ///< 原始或成功精修后的底部端点。
  double length_px{0.0};        ///< 根据原始两端点估计的灯条长度。
  double width_px{0.0};         ///< 按固定长宽比估计的灯条宽度。
  double mean_brightness{0.0};  ///< 灯条 ROI 归一化前的平均灰度。
  bool axis_valid{false};       ///< 灰度矩是否给出了有效质心与主轴。
  bool success{false};          ///< 该灯条上下端点是否均已找到。
};

/** @brief 单块装甲四角的原子精修结果及完整诊断。 */
struct CornerRefinementResult {
  std::array<cv::Point2f, 4> original_corners{};  ///< TL、TR、BR、BL 顺序的网络角点。
  std::array<cv::Point2f, 4> refined_corners{};  ///< 成功时的精修角点，失败时等于原始角点。
  std::array<cv::Point2f, 4> corner_displacements{};  ///< 最终角点相对原始角点的位移。
  std::array<LightbarRefinementDiagnostic, 2> lightbars{};  ///< 左、右灯条诊断。
  std::array<EndpointRefinementDiagnostic, 4> endpoints{};  ///< 与四角顺序对应的端点诊断。
  CornerRefinementStatus status{CornerRefinementStatus::INVALID_GEOMETRY};  ///< 最终状态。
  int failure_light_index{-1};  ///< 失败灯条索引；左为 0、右为 1，无特定灯条时为 -1。
  bool success{false};          ///< 是否成功提交全部四个精修角点。
  bool fallback{true};          ///< 是否整体回退到原始网络角点。
  double elapsed_ms{0.0};  ///< 本次 Refine() 调用耗时，单位为毫秒。
};

/**
 * @brief 使用灯条灰度矩主轴和亮度下降边缘精修装甲四角。
 *
 * 左右灯条独立搜索，但结果以整块装甲为单位原子提交；任一端点失败都会回退全部原始角点。
 */
class ArmorCornerRefiner final {
 public:
  /** @brief 保存已校验的不可变精修参数。 */
  explicit ArmorCornerRefiner(ArmorCornerRefinerConfig config);

  /**
   * @brief 精修 TL、TR、BR、BL 顺序的四个装甲角点。
   * @param gray_image 与检测结果同帧的 CV_8UC1 灰度图。
   * @param corners TL、TR、BR、BL 顺序的原始网络角点。
   * @return 精修角点、回退状态、耗时和逐灯条/端点诊断。
   */
  [[nodiscard]] CornerRefinementResult Refine(const cv::Mat& gray_image,
                                              std::span<const cv::Point2f, 4> corners) const;

 private:
  ArmorCornerRefinerConfig config_;  ///< 构造时保存的精修参数副本。
};

}  // namespace mv::modules
