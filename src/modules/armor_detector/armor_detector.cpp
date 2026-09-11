#include "modules/armor_detector/armor_detector.hpp"

#include "core/logger.hpp"
#include "modules/armor_detector/armor_detector_postprocess.hpp"
#include "modules/armor_detector/armor_inference_backend.hpp"

#include <chrono>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <span>

namespace mv::modules {
namespace {
using Clock = std::chrono::steady_clock;
double Milliseconds(Clock::time_point start, Clock::time_point end) noexcept {
  return std::chrono::duration<double, std::milli>(end - start).count();
}
constexpr int K_WARMUP_RUNS = 10;
}  // namespace

struct YoloArmorDetector::Impl {
  // 后端绑定此固定画布；成员顺序保证后端先于画布销毁。
  cv::Mat input_image{detail::K_MODEL_HEIGHT, detail::K_MODEL_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0)};
  std::unique_ptr<detail::ArmorInferenceBackend> backend;
  std::span<const float> output;
  ArmorDetectorConfig config;
  bool initialized{false};
};

YoloArmorDetector::YoloArmorDetector() : impl_(std::make_unique<Impl>()) {}
YoloArmorDetector::~YoloArmorDetector() = default;

void YoloArmorDetector::Init(const ArmorDetectorConfig& config) {
  if (impl_->initialized)
    throw std::logic_error("armor detector is already initialized");
  try {
    auto backend = detail::MakeInferenceBackend(config);
    if (!std::filesystem::is_regular_file(config.model_path)) {
      throw ArmorDetectorInitError("0526 model is missing: " + config.model_path.string() +
                                   "; see src/modules/armor_detector/models/README.md");
    }
    impl_->input_image.setTo(cv::Scalar(0, 0, 0));
    backend->Initialize(config, impl_->input_image);
    for (int run = 0; run < K_WARMUP_RUNS; ++run) {
      (void)backend->Infer();
    }
    MV_LOG_INFO("ArmorDetector", "initialized {} with {} on {} (device: {}, enemy color: {})",
                config.model_path.string(), backend->Name(), backend->DeviceName(), config.device,
                ArmorColorName(config.enemy_color));
    impl_->config = config;
    impl_->backend = std::move(backend);
    impl_->initialized = true;
  } catch (const ArmorDetectorInitError&) {
    throw;
  } catch (const std::exception& error) {
    throw ArmorDetectorInitError("failed to initialize 0526 detector: " +
                                 std::string(error.what()));
  }
}

ArmorDetectorResult YoloArmorDetector::Detect(const cv::Mat& bgr_image) {
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
    const auto OUTPUT = impl_->backend->Infer();
    if (OUTPUT.size() != detail::K_OUTPUT_ROWS * detail::K_OUTPUT_COLUMNS) {
      throw ArmorDetectorRuntimeError("unexpected 0526 output tensor size");
    }
    impl_->output = OUTPUT;
  } catch (const std::exception& error) {
    throw ArmorDetectorRuntimeError(impl_->backend->Name() +
                                    " inference failed: " + std::string(error.what()));
  }
  const auto INFERENCE_END = Clock::now();

  try {
    const detail::DecodeThresholds THRESHOLDS{.confidence = impl_->config.confidence_threshold,
                                              .nms_iou = impl_->config.nms_iou_threshold};
    auto decoded = detail::DecodeYolo0526(impl_->output.data(), detail::K_OUTPUT_ROWS,
                                          detail::K_OUTPUT_COLUMNS, TRANSFORM,
                                          impl_->config.enemy_color, THRESHOLDS);
    const auto POSTPROCESS_END = Clock::now();

    // 统计值只在整次检测成功后更新，异常不会留下部分阶段的新旧混合数据。
    ArmorDetectorResult result;
    result.output.detections = std::move(decoded.detections);
    result.diagnostics.preprocess_ms = Milliseconds(TOTAL_START, PREPROCESS_END);
    result.diagnostics.inference_ms = Milliseconds(PREPROCESS_END, INFERENCE_END);
    result.diagnostics.postprocess_ms = Milliseconds(INFERENCE_END, POSTPROCESS_END);
    result.diagnostics.total_ms = Milliseconds(TOTAL_START, POSTPROCESS_END);
    result.diagnostics.threshold_candidates = decoded.threshold_candidates;
    result.diagnostics.kept_detections = result.output.detections.size();
    return result;
  } catch (const ArmorDetectorRuntimeError&) {
    throw;
  } catch (const std::exception& error) {
    throw ArmorDetectorRuntimeError("0526 postprocessing failed: " + std::string(error.what()));
  }
}

void YoloArmorDetector::UpdateRuntimeConfig(const ArmorDetectorRuntimeConfig& config) noexcept {
  impl_->config.enemy_color = config.enemy_color;
  impl_->config.confidence_threshold = config.confidence_threshold;
  impl_->config.nms_iou_threshold = config.nms_iou_threshold;
}

bool YoloArmorDetector::IsInitialized() const noexcept {
  return impl_->initialized;
}

}  // namespace mv::modules
