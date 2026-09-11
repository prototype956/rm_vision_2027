#include "modules/armor_detector/armor_detector_postprocess.hpp"
#include "modules/armor_detector/armor_inference_backend.hpp"

#include <algorithm>
#include <sstream>
#include <vector>

#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/openvino.hpp>
#include <openvino/runtime/properties.hpp>

namespace mv::modules::detail {
namespace {
constexpr char K_EXPECTED_INPUT_NAME[] = "images";
constexpr char K_EXPECTED_OUTPUT_NAME[] = "output";

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

class OpenVinoBackend final : public ArmorInferenceBackend {
 public:
  void Initialize(const ArmorDetectorConfig& config, const cv::Mat& input) override {
    // 编译前显式确认目标 GPU 存在，避免 OpenVINO 隐式选择其他执行设备。
    const auto AVAILABLE_DEVICES = core_.get_available_devices();
    if (std::find(AVAILABLE_DEVICES.begin(), AVAILABLE_DEVICES.end(), config.device) ==
        AVAILABLE_DEVICES.end()) {
      throw ArmorDetectorInitError("configured GPU device '" + config.device +
                                   "' is unavailable; OpenVINO devices: [" +
                                   Join(AVAILABLE_DEVICES) + "]");
    }

    auto model = core_.read_model(config.model_path);
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
    compiled_model_ = core_.compile_model(
        model, config.device, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    const auto EXECUTION_DEVICES = compiled_model_.get_property(ov::execution_devices);
    if (EXECUTION_DEVICES.empty() ||
        !std::all_of(EXECUTION_DEVICES.begin(), EXECUTION_DEVICES.end(),
                     [](const std::string& device) { return IsGpuExecutionDevice(device); })) {
      throw ArmorDetectorInitError(
          "compiled model selected a non-GPU execution device; execution devices: [" +
          Join(EXECUTION_DEVICES) + "]");
    }

    // Tensor 直接引用固定 cv::Mat 缓冲区，后续每帧只更新像素而不重复分配。
    input_tensor_ = ov::Tensor(ov::element::u8,
                               {1, detail::K_MODEL_HEIGHT, detail::K_MODEL_WIDTH, 3}, input.data);
    infer_request_ = compiled_model_.create_infer_request();
    infer_request_.set_input_tensor(input_tensor_);

    device_name_ = core_.get_property(config.device, ov::device::full_name);
  }
  std::span<const float> Infer() override {
    infer_request_.infer();
    output_tensor_ = infer_request_.get_output_tensor();
    if (output_tensor_.get_element_type() != ov::element::f32 ||
        output_tensor_.get_shape() != ov::Shape{1, K_OUTPUT_ROWS, K_OUTPUT_COLUMNS}) {
      throw ArmorDetectorRuntimeError("OpenVINO returned an unexpected 0526 output tensor");
    }
    return {output_tensor_.data<const float>(), K_OUTPUT_ROWS * K_OUTPUT_COLUMNS};
  }
  std::string Name() const override { return "openvino"; }
  std::string DeviceName() const override { return device_name_; }

 private:
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;
  ov::Tensor input_tensor_;
  ov::Tensor output_tensor_;
  std::string device_name_;
};
}  // namespace

bool OpenVinoDeviceAvailable(const std::string& device) {
  ov::Core core;
  const auto DEVICES = core.get_available_devices();
  return std::find(DEVICES.begin(), DEVICES.end(), device) != DEVICES.end();
}

std::unique_ptr<ArmorInferenceBackend> MakeOpenVinoBackend() {
  return std::make_unique<OpenVinoBackend>();
}
}  // namespace mv::modules::detail
