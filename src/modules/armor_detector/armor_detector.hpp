#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <filesystem>
#include <opencv2/core.hpp>

namespace mv::modules {

/**
 * @brief 装甲板灯条颜色。
 *
 * 检测器只返回配置指定的敌方颜色，枚举值同时用于上层显示和目标筛选。
 */
enum class ArmorColor {
  RED,
  BLUE,
};

/**
 * @brief RobotDetectionModel 0526 支持的装甲类别。
 *
 * 枚举顺序与模型输出中的 9 个类别通道保持一致，不应随意调整。
 */
enum class ArmorLabel {
  SENTRY,
  ONE,
  TWO,
  THREE,
  FOUR,
  FIVE,
  OUTPOST,
  BASE_SMALL,
  BASE_BIG,
};

/**
 * @brief 将装甲颜色转换为稳定的英文字符串。
 *
 * @param color 待转换的颜色。
 * @return 静态字符串，生命周期覆盖整个进程。
 */
[[nodiscard]] const char* ArmorColorName(ArmorColor color) noexcept;

/**
 * @brief 将装甲类别转换为稳定的英文字符串。
 *
 * @param label 待转换的类别。
 * @return 静态字符串，生命周期覆盖整个进程。
 */
[[nodiscard]] const char* ArmorLabelName(ArmorLabel label) noexcept;

/**
 * @brief 单个装甲板的检测结果。
 *
 * 坐标均已从 640x640 模型空间映射并裁剪回输入图像空间。
 */
struct ArmorDetection {
  ArmorColor color{ArmorColor::RED};  ///< 模型判定且通过敌方颜色筛选的灯条颜色。
  ArmorLabel label{ArmorLabel::SENTRY};  ///< 模型判定的装甲类别。
  float objectness{0.0F};                ///< sigmoid 后的目标存在置信度。
  cv::Rect2f bounding_box;               ///< 四个角点的轴对齐外接矩形。
  std::array<cv::Point2f, 4> corners{};  ///< 左上、右上、右下、左下顺序的四角点。
};

/**
 * @brief 最近一次 Detect() 的分阶段性能指标。
 *
 * 所有耗时均不包含相机抓帧、调用方图像拷贝和结果绘制。下一次 Detect() 会覆盖
 * 当前统计值，因此调用方应在同一同步调用链中及时读取。
 */
struct DetectorStats {
  double preprocess_ms{0.0};            ///< Letterbox 与缩放耗时。
  double inference_ms{0.0};             ///< OpenVINO 同步推理耗时。
  double postprocess_ms{0.0};           ///< 输出解码、筛选与 NMS 耗时。
  double total_ms{0.0};                 ///< 上述三个阶段的完整链路耗时。
  std::size_t threshold_candidates{0};  ///< 通过 objectness 阈值的原始候选数。
  std::size_t kept_detections{0};       ///< 颜色、几何筛选和 NMS 后的结果数。
};

/**
 * @brief 装甲检测器初始化参数。
 */
struct ArmorDetectorConfig {
  std::filesystem::path model_path;         ///< 0526 ONNX 模型的绝对路径。
  std::string device{"GPU"};                ///< OpenVINO GPU 或 GPU.<index> 设备名。
  ArmorColor enemy_color{ArmorColor::RED};  ///< 需要保留的敌方装甲颜色。
  float confidence_threshold{0.65F};        ///< objectness 筛选阈值，范围为 (0, 1)。
  float nms_iou_threshold{0.45F};           ///< NMS IoU 阈值，范围为 [0, 1]。
};

/**
 * @brief 模型、设备或 OpenVINO 编译阶段的初始化异常。
 */
class ArmorDetectorInitError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/**
 * @brief 已初始化检测器在推理或后处理阶段的运行异常。
 */
class ArmorDetectorRuntimeError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/**
 * @brief 基于 OpenVINO 的 RobotDetectionModel 0526 同步装甲检测器。
 *
 * 实例不可拷贝、不可移动且非线程安全。Init() 只能成功调用一次，后续可重复同步
 * 调用 Detect()；Detect() 不修改调用方传入的图像。
 */
class YoloArmorDetector final {
 public:
  YoloArmorDetector();
  ~YoloArmorDetector();

  YoloArmorDetector(const YoloArmorDetector&) = delete;
  YoloArmorDetector& operator=(const YoloArmorDetector&) = delete;
  YoloArmorDetector(YoloArmorDetector&&) = delete;
  YoloArmorDetector& operator=(YoloArmorDetector&&) = delete;

  /**
   * @brief 加载模型、校验输入输出契约并在指定 GPU 上编译。
   *
   * 初始化阶段创建并绑定可复用的输入 Tensor 和 InferRequest，同时执行固定次数
   * 的空白帧预热。CPU、AUTO 和 MULTI 设备均不接受。
   *
   * @param config 已完成字段和值域校验的检测器配置。
   * @throws std::logic_error 当前实例已经初始化。
   * @throws ArmorDetectorInitError 模型缺失、契约不匹配、GPU 不可用或编译失败。
   */
  void Init(const ArmorDetectorConfig& config);

  /**
   * @brief 对一张 BGR 图像同步执行预处理、推理和后处理。
   *
   * 任意正尺寸图像都会按比例缩放到 640x640 左上对齐画布，结果坐标再映射回原图。
   *
   * @param bgr_image 非空的 CV_8UC3 BGR 图像，函数不会修改其内容。
   * @return 经置信度、敌方颜色、几何有效性和 NMS 筛选后的装甲结果。
   * @throws std::logic_error 检测器尚未初始化。
   * @throws std::invalid_argument 输入图像为空或类型不正确。
   * @throws ArmorDetectorRuntimeError OpenVINO 推理或后处理失败。
   */
  [[nodiscard]] std::vector<ArmorDetection> Detect(const cv::Mat& bgr_image);

  /**
   * @brief 检查当前实例是否已成功初始化。
   *
   * @return 可以调用 Detect() 时返回 true。
   */
  [[nodiscard]] bool IsInitialized() const noexcept;

  /**
   * @brief 获取最近一次成功检测的性能统计。
   *
   * @return 由检测器持有的只读引用，在下一次 Detect() 时更新。
   */
  [[nodiscard]] const DetectorStats& LastStats() const noexcept;

 private:
  struct Impl;                  ///< 隐藏 OpenVINO 类型和运行时资源。
  std::unique_ptr<Impl> impl_;  ///< 检测器的唯一实现对象。
};

}  // namespace mv::modules
