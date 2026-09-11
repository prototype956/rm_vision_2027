#pragma once

#include "modules/armor_detector/armor_detector_output.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

#include <filesystem>
#include <opencv2/core.hpp>

namespace mv::modules {

/** @brief 推理引擎选择；AUTO 只探测已编译的 GPU 后端，不接受 CPU 回退。 */
enum class ArmorInferenceBackend : std::uint8_t { AUTO, OPENVINO, TENSORRT };
[[nodiscard]] std::string_view ArmorInferenceBackendName(ArmorInferenceBackend backend) noexcept;

/**
 * @brief 最近一次 Detect() 的分阶段性能指标。
 *
 * 所有耗时均不包含相机抓帧、调用方图像拷贝和结果绘制。下一次 Detect() 会覆盖
 * 当前统计值，因此调用方应在同一同步调用链中及时读取。
 */
struct ArmorDetectorDiagnostics {
  double preprocess_ms{0.0};            ///< Letterbox 与缩放耗时。
  double inference_ms{0.0};             ///< 后端转换、传输与同步推理耗时。
  double postprocess_ms{0.0};           ///< 输出解码、筛选与 NMS 耗时。
  double total_ms{0.0};                 ///< 上述三个阶段的完整链路耗时。
  std::size_t threshold_candidates{0};  ///< 通过 objectness 阈值的原始候选数。
  std::size_t kept_detections{0};       ///< 颜色、几何筛选和 NMS 后的结果数。
};

/** @brief 单次检测调用的正式输出和诊断输出。 */
struct ArmorDetectorResult {
  ArmorDetectorOutput output;
  ArmorDetectorDiagnostics diagnostics;
};

/**
 * @brief 装甲检测器初始化参数。
 */
struct ArmorDetectorConfig {
  ArmorInferenceBackend backend{ArmorInferenceBackend::AUTO};  ///< 引擎选择，初始化后不可修改。
  std::filesystem::path model_path;  ///< 0526 ONNX；TensorRT 也支持本机生成的 .engine。
  std::string device{"GPU"};         ///< GPU 或 GPU.<index>；TensorRT 对应 CUDA 设备序号。
  ArmorColor enemy_color{ArmorColor::RED};  ///< 需要保留的敌方装甲颜色。
  float confidence_threshold{0.65F};        ///< objectness 筛选阈值，范围为 (0, 1)。
  float nms_iou_threshold{0.45F};           ///< NMS IoU 阈值，范围为 [0, 1]。
};

/** @brief 不重载模型即可在帧边界更新的检测后处理参数。 */
struct ArmorDetectorRuntimeConfig {
  ArmorColor enemy_color{ArmorColor::RED};  ///< 需要保留的敌方装甲颜色。
  float confidence_threshold{0.65F};        ///< objectness 筛选阈值，范围为 (0, 1)。
  float nms_iou_threshold{0.45F};           ///< NMS IoU 阈值，范围为 [0, 1]。
};

/**
 * @brief 模型、设备或后端编译阶段的初始化异常。
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
 * @brief 支持 OpenVINO/TensorRT 的 RobotDetectionModel 0526 同步装甲检测器。
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
   * 初始化阶段创建并绑定可复用的推理资源，同时执行固定次数
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
   * @throws ArmorDetectorRuntimeError 后端推理或后处理失败。
   */
  [[nodiscard]] ArmorDetectorResult Detect(const cv::Mat& bgr_image);

  /**
   * @brief 在调用线程的下一次 Detect() 前替换轻量后处理参数。
   *
   * 调用方必须保证本函数不与 Detect() 并发，并传入已经过配置解析器校验的参数。
   */
  void UpdateRuntimeConfig(const ArmorDetectorRuntimeConfig& config) noexcept;

  /**
   * @brief 检查当前实例是否已成功初始化。
   *
   * @return 可以调用 Detect() 时返回 true。
   */
  [[nodiscard]] bool IsInitialized() const noexcept;

 private:
  struct Impl;                  ///< 隐藏推理后端类型和运行时资源。
  std::unique_ptr<Impl> impl_;  ///< 检测器的唯一实现对象。
};

}  // namespace mv::modules
