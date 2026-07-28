#pragma once

#include "modules/armor_detector/armor_detector.hpp"

#include <cstddef>
#include <vector>

namespace mv::modules::detail {

// 0526 模型的固定输入尺寸和输出 Tensor 契约。
constexpr int K_MODEL_WIDTH = 640;
constexpr int K_MODEL_HEIGHT = 640;
constexpr std::size_t K_OUTPUT_ROWS = 25200;
constexpr std::size_t K_OUTPUT_COLUMNS = 22;

/**
 * @brief 左上对齐 Letterbox 在模型空间与原图空间之间的映射参数。
 */
struct LetterboxTransform {
  float scale{1.0F};                   ///< 原图到模型有效区域的统一缩放比例。
  int content_width{K_MODEL_WIDTH};    ///< 模型画布内有效图像宽度。
  int content_height{K_MODEL_HEIGHT};  ///< 模型画布内有效图像高度。
  int source_width{K_MODEL_WIDTH};     ///< 调用方输入图像宽度。
  int source_height{K_MODEL_HEIGHT};   ///< 调用方输入图像高度。
};

/**
 * @brief 0526 输出 Tensor 的完整解码结果。
 */
struct DecodeResult {
  std::vector<ArmorDetection> detections;  ///< 完成颜色、几何和 NMS 筛选的结果。
  std::size_t threshold_candidates{0};     ///< 通过 objectness 阈值的原始行数。
};

/**
 * @brief 计算保持长宽比、左上对齐的 640x640 Letterbox 映射。
 *
 * @param source_width 输入图像宽度。
 * @param source_height 输入图像高度。
 * @return 原图和模型有效区域之间的缩放参数。
 * @throws std::invalid_argument 任一输入尺寸非正。
 */
[[nodiscard]] LetterboxTransform MakeLetterboxTransform(int source_width, int source_height);

/**
 * @brief 以数值稳定的分支形式计算 sigmoid。
 *
 * @param value 模型输出的原始 logit。
 * @return 映射到 [0, 1] 的概率值。
 */
[[nodiscard]] float StableSigmoid(float value) noexcept;

/**
 * @brief 计算两个轴对齐矩形的交并比。
 *
 * @param lhs 第一个轴对齐矩形。
 * @param rhs 第二个轴对齐矩形。
 * @return 无重叠或并集面积非正时返回 0，否则返回 [0, 1] 范围内的 IoU。
 */
[[nodiscard]] float IntersectionOverUnion(const cv::Rect2f& lhs, const cv::Rect2f& rhs) noexcept;

/**
 * @brief 按 objectness 从高到低执行类别无关 NMS。
 *
 * @param candidates 已通过基础筛选的候选结果。
 * @param iou_threshold 抑制重叠候选的 IoU 阈值，范围为 [0, 1]。
 * @return 保留候选在输入数组中的索引，顺序与 objectness 降序一致。
 * @throws std::invalid_argument iou_threshold 超出允许范围。
 */
[[nodiscard]] std::vector<std::size_t> ClassAgnosticNms(
    const std::vector<ArmorDetection>& candidates, float iou_threshold);

/**
 * @brief 解码 RobotDetectionModel 0526 输出并映射回输入图像空间。
 *
 * 每一行依次包含 4 个模型角点、1 个 objectness logit、4 个颜色通道和 9 个
 * 装甲类别通道。函数按置信度、敌方颜色、Letterbox 有效区域和几何有效性筛选，
 * 最后执行类别无关 NMS。
 *
 * @param output 连续存储的 FP32 模型输出首地址。
 * @param rows 输出行数，必须为 K_OUTPUT_ROWS。
 * @param columns 每行元素数，必须为 K_OUTPUT_COLUMNS。
 * @param transform 当前输入图像对应的 Letterbox 映射。
 * @param enemy_color 需要保留的敌方颜色。
 * @param confidence_threshold sigmoid objectness 阈值，范围为 (0, 1)。
 * @param nms_iou_threshold NMS IoU 阈值，范围为 [0, 1]。
 * @return 完成筛选和 NMS 的检测结果及阈值候选数量。
 * @throws std::invalid_argument 输出指针、形状或阈值非法。
 */
[[nodiscard]] DecodeResult DecodeYolo0526(const float* output, std::size_t rows,
                                          std::size_t columns, const LetterboxTransform& transform,
                                          ArmorColor enemy_color, float confidence_threshold,
                                          float nms_iou_threshold);

}  // namespace mv::modules::detail
