#include "modules/armor_detector/armor_detector.hpp"

#include "core/logger.hpp"
#include "modules/armor_detector/armor_detector_postprocess.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/openvino.hpp>
#include <openvino/runtime/properties.hpp>

namespace mv::modules {
namespace {

constexpr char K_EXPECTED_INPUT_NAME[] = "images";
constexpr char K_EXPECTED_OUTPUT_NAME[] = "output";
constexpr int K_WARMUP_RUNS = 10;

using Clock = std::chrono::steady_clock;

// 将单调时钟的两个时间点统一换算为毫秒。
double Milliseconds(Clock::time_point start, Clock::time_point end) noexcept {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

// 连接 OpenVINO 设备名列表，供初始化日志和异常信息使用。
std::string Join(const std::vector<std::string>& values) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  return stream.str();
}

// OpenVINO 可能返回 GPU 或带编号的 GPU.<index> 执行设备。
bool IsGpuExecutionDevice(const std::string& device) {
  return device == "GPU" || device.starts_with("GPU.");
}

// 在添加 OpenVINO 预处理前验证原始 0526 ONNX 的固定输入输出契约。
void ValidateModelContract(const std::shared_ptr<ov::Model>& model) {
  if (model->inputs().size() != 1 || model->outputs().size() != 1) {
    throw ArmorDetectorInitError("0526 model must have exactly one input and one output");
  }

  const auto INPUT = model->input();
  const auto OUTPUT = model->output();
  if (INPUT.get_any_name() != K_EXPECTED_INPUT_NAME) {
    throw ArmorDetectorInitError("0526 model input must be named 'images'");
  }
  if (OUTPUT.get_any_name() != K_EXPECTED_OUTPUT_NAME) {
    throw ArmorDetectorInitError("0526 model output must be named 'output'");
  }
  if (INPUT.get_element_type() != ov::element::f16 ||
      INPUT.get_shape() != ov::Shape{1, 3, 640, 640}) {
    throw ArmorDetectorInitError("0526 model input must be FP16 [1,3,640,640]");
  }
  if (OUTPUT.get_element_type() != ov::element::f32 ||
      OUTPUT.get_shape() != ov::Shape{1, detail::K_OUTPUT_ROWS, detail::K_OUTPUT_COLUMNS}) {
    throw ArmorDetectorInitError("0526 model output must be FP32 [1,25200,22]");
  }
}

}  // namespace

// PImpl 隔离 OpenVINO 头文件，并集中持有可跨帧复用的推理资源与输入缓冲区。
struct YoloArmorDetector::Impl {
  ov::Core core;
  ov::CompiledModel compiled_model;
  ov::InferRequest infer_request;
  ov::Tensor input_tensor;
  cv::Mat input_image{detail::K_MODEL_HEIGHT, detail::K_MODEL_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0)};
  ArmorDetectorConfig config;
  DetectorStats stats;
  bool initialized{false};
};

YoloArmorDetector::YoloArmorDetector() : impl_(std::make_unique<Impl>()) {}

YoloArmorDetector::~YoloArmorDetector() = default;

void YoloArmorDetector::Init(const ArmorDetectorConfig& config) {
  if (impl_->initialized) {
    throw std::logic_error("armor detector is already initialized");
  }
  if (!std::filesystem::is_regular_file(config.model_path)) {
    throw ArmorDetectorInitError(
        "0526 model is missing: " + config.model_path.string() +
        "; see src/modules/armor_detector/models/README.md for manual copy instructions");
  }
  if (!IsGpuExecutionDevice(config.device)) {
    throw ArmorDetectorInitError(
        "armor detector only accepts GPU or GPU.<index> as an explicit device");
  }

  try {
    // 编译前显式确认目标 GPU 存在，避免 OpenVINO 隐式选择其他执行设备。
    const auto AVAILABLE_DEVICES = impl_->core.get_available_devices();
    if (std::find(AVAILABLE_DEVICES.begin(), AVAILABLE_DEVICES.end(), config.device) ==
        AVAILABLE_DEVICES.end()) {
      throw ArmorDetectorInitError("configured GPU device '" + config.device +
                                   "' is unavailable; OpenVINO devices: [" +
                                   Join(AVAILABLE_DEVICES) + "]");
    }

    auto model = impl_->core.read_model(config.model_path);
    ValidateModelContract(model);

    // 调用方提供 NHWC BGR U8；OpenVINO 负责转为模型需要的 NCHW RGB FP16 并归一化。
    ov::preprocess::PrePostProcessor preprocessor(model);
    preprocessor.input()
        .tensor()
        .set_element_type(ov::element::u8)
        .set_shape({1, detail::K_MODEL_HEIGHT, detail::K_MODEL_WIDTH, 3})
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);
    preprocessor.input().model().set_layout("NCHW");
    preprocessor.input()
        .preprocess()
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .convert_element_type(ov::element::f16)
        .scale(255.0F);
    preprocessor.output().tensor().set_element_type(ov::element::f32);
    model = preprocessor.build();

    // 同步单帧调用优先降低延迟，编译后再次核对实际执行设备没有发生 CPU 回退。
    impl_->compiled_model = impl_->core.compile_model(
        model, config.device, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    const auto EXECUTION_DEVICES = impl_->compiled_model.get_property(ov::execution_devices);
    if (EXECUTION_DEVICES.empty() ||
        !std::all_of(EXECUTION_DEVICES.begin(), EXECUTION_DEVICES.end(),
                     [](const std::string& device) { return IsGpuExecutionDevice(device); })) {
      throw ArmorDetectorInitError(
          "compiled model selected a non-GPU execution device; execution devices: [" +
          Join(EXECUTION_DEVICES) + "]");
    }

    // Tensor 直接引用固定 cv::Mat 缓冲区，后续每帧只更新像素而不重复分配。
    impl_->input_tensor =
        ov::Tensor(ov::element::u8, {1, detail::K_MODEL_HEIGHT, detail::K_MODEL_WIDTH, 3},
                   impl_->input_image.data);
    impl_->infer_request = impl_->compiled_model.create_infer_request();
    impl_->infer_request.set_input_tensor(impl_->input_tensor);
    // 使用全黑输入预热 GPU 和内部执行图，预热耗时不进入任何 DetectorStats。
    for (int run = 0; run < K_WARMUP_RUNS; ++run) {
      impl_->infer_request.infer();
    }

    impl_->config = config;
    impl_->initialized = true;
    const auto FULL_DEVICE_NAME = impl_->core.get_property(config.device, ov::device::full_name);
    MV_LOG_INFO("ArmorDetector", "initialized {} on {} (execution devices: {}, enemy color: {})",
                config.model_path.string(), FULL_DEVICE_NAME, Join(EXECUTION_DEVICES),
                ArmorColorName(config.enemy_color));
  } catch (const ArmorDetectorInitError&) {
    throw;
  } catch (const std::exception& error) {
    throw ArmorDetectorInitError("failed to initialize 0526 detector: " +
                                 std::string(error.what()));
  }
}

std::vector<ArmorDetection> YoloArmorDetector::Detect(const cv::Mat& bgr_image) {
  if (!impl_->initialized) {
    throw std::logic_error("armor detector is not initialized");
  }
  if (bgr_image.empty()) {
    throw std::invalid_argument("armor detector input image must not be empty");
  }
  if (bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("armor detector input image must be CV_8UC3 BGR");
  }

  const auto TOTAL_START = Clock::now();
  // 0526 部署约定为左上对齐 Letterbox，未占用区域保持黑色。
  const auto TRANSFORM = detail::MakeLetterboxTransform(bgr_image.cols, bgr_image.rows);
  impl_->input_image.setTo(cv::Scalar(0, 0, 0));
  const cv::Rect DESTINATION(0, 0, TRANSFORM.content_width, TRANSFORM.content_height);
  cv::resize(bgr_image, impl_->input_image(DESTINATION), DESTINATION.size(), 0.0, 0.0,
             cv::INTER_LINEAR);
  const auto PREPROCESS_END = Clock::now();

  try {
    impl_->infer_request.infer();
  } catch (const std::exception& error) {
    throw ArmorDetectorRuntimeError("OpenVINO inference failed: " + std::string(error.what()));
  }
  const auto INFERENCE_END = Clock::now();

  try {
    // 运行期间仍校验输出类型和形状，避免异常模型或插件结果越界进入解码器。
    const auto OUTPUT = impl_->infer_request.get_output_tensor();
    if (OUTPUT.get_element_type() != ov::element::f32 ||
        OUTPUT.get_shape() != ov::Shape{1, detail::K_OUTPUT_ROWS, detail::K_OUTPUT_COLUMNS}) {
      throw ArmorDetectorRuntimeError("OpenVINO returned an unexpected 0526 output tensor");
    }
    const detail::DecodeThresholds THRESHOLDS{
        .confidence = impl_->config.confidence_threshold,
        .nms_iou = impl_->config.nms_iou_threshold};
    auto decoded = detail::DecodeYolo0526(
        OUTPUT.data<const float>(), detail::K_OUTPUT_ROWS, detail::K_OUTPUT_COLUMNS, TRANSFORM,
        impl_->config.enemy_color, THRESHOLDS);
    const auto POSTPROCESS_END = Clock::now();

    // 统计值只在整次检测成功后更新，异常不会留下部分阶段的新旧混合数据。
    impl_->stats.preprocess_ms = Milliseconds(TOTAL_START, PREPROCESS_END);
    impl_->stats.inference_ms = Milliseconds(PREPROCESS_END, INFERENCE_END);
    impl_->stats.postprocess_ms = Milliseconds(INFERENCE_END, POSTPROCESS_END);
    impl_->stats.total_ms = Milliseconds(TOTAL_START, POSTPROCESS_END);
    impl_->stats.threshold_candidates = decoded.threshold_candidates;
    impl_->stats.kept_detections = decoded.detections.size();
    return std::move(decoded.detections);
  } catch (const ArmorDetectorRuntimeError&) {
    throw;
  } catch (const std::exception& error) {
    throw ArmorDetectorRuntimeError("0526 postprocessing failed: " + std::string(error.what()));
  }
}

bool YoloArmorDetector::IsInitialized() const noexcept {
  return impl_->initialized;
}

const DetectorStats& YoloArmorDetector::LastStats() const noexcept {
  return impl_->stats;
}

}  // namespace mv::modules
