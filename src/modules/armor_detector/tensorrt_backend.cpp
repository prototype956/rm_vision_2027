#include "core/logger.hpp"
#include "modules/armor_detector/armor_detector_postprocess.hpp"
#include "modules/armor_detector/armor_inference_backend.hpp"

#include <array>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <vector>

#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <NvOnnxParser.h>

namespace mv::modules::detail {
namespace {
constexpr char K_INPUT_NAME[] = "images";
constexpr char K_OUTPUT_NAME[] = "output";
constexpr std::size_t K_INPUT_PIXELS = static_cast<std::size_t>(K_MODEL_HEIGHT) * K_MODEL_WIDTH;
constexpr std::size_t K_OUTPUT_ELEMENTS = K_OUTPUT_ROWS * K_OUTPUT_COLUMNS;

void CheckCuda(cudaError_t status, const char* operation) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
  }
}

class TensorRtLogger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* message) noexcept override {
    // TensorRT invokes this noexcept callback from internal threads as well.
    try {
      if (severity <= Severity::kWARNING)
        MV_LOG_WARN("TensorRT", "{}", message);
    } catch (...) {
      // Fallback does not allocate or unwind through the TensorRT ABI.
      std::fputs("TensorRT logger failed: ", stderr);
      std::fputs(message, stderr);
      std::fputc('\n', stderr);
    }
  }
};

std::shared_ptr<TensorRtLogger> GetTensorRtLogger() {
  // TensorRT registers one process-wide logger. Sharing ownership keeps it alive even
  // when the first detector is destroyed before another detector or a static runtime.
  static const auto LOGGER = std::make_shared<TensorRtLogger>();
  return LOGGER;
}

bool HasShape(const nvinfer1::Dims& shape, const std::array<int, 4>& expected, int rank) {
  if (shape.nbDims != rank)
    return false;
  for (int axis = 0; axis < rank; ++axis) {
    if (shape.d[axis] != expected[axis])
      return false;
  }
  return true;
}

/** @brief 同步 CUDA 推理；上下文仅由检测线程使用，设备编号在每次调用时绑定。 */
class TensorRtBackend final : public ArmorInferenceBackend {
 public:
  ~TensorRtBackend() override {
    // Init can throw after any allocation; release resources on their owning device.
    if (device_index_ < 0)
      return;
    (void)cudaSetDevice(device_index_);
    if (stream_ != nullptr)
      (void)cudaStreamSynchronize(stream_);
    context_.reset();
    if (device_input_ != nullptr)
      (void)cudaFree(device_input_);
    if (device_output_ != nullptr)
      (void)cudaFree(device_output_);
    if (stream_ != nullptr)
      (void)cudaStreamDestroy(stream_);
  }

  void Initialize(const ArmorDetectorConfig& config, const cv::Mat& input) override {
    const int DEVICE_INDEX = ParseGpuDeviceIndex(config.device);
    CheckCuda(cudaSetDevice(DEVICE_INDEX), "select CUDA device");
    device_index_ = DEVICE_INDEX;
    cudaDeviceProp properties{};
    CheckCuda(cudaGetDeviceProperties(&properties, device_index_), "query CUDA device");
    device_name_ = properties.name;
    input_image_ = input;
    host_input_.resize(3 * K_INPUT_PIXELS);
    host_output_.resize(K_OUTPUT_ELEMENTS);
    runtime_.reset(nvinfer1::createInferRuntime(*logger_));
    if (!runtime_)
      throw ArmorDetectorInitError("could not create TensorRT runtime");

    if (config.model_path.extension() == ".engine") {
      std::ifstream file(config.model_path, std::ios::binary);
      if (!file)
        throw ArmorDetectorInitError("could not open TensorRT engine");
      const std::vector<char> SERIALIZED{std::istreambuf_iterator<char>(file), {}};
      if (SERIALIZED.empty() || file.bad())
        throw ArmorDetectorInitError("could not read TensorRT engine");
      engine_.reset(runtime_->deserializeCudaEngine(SERIALIZED.data(), SERIALIZED.size()));
    } else if (config.model_path.extension() == ".onnx") {
      std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(*logger_));
      if (!builder)
        throw ArmorDetectorInitError("could not create TensorRT builder");
        // TensorRT 11 makes strongly typed networks the default and removes this flag.
#if NV_TENSORRT_MAJOR < 11
      constexpr auto FLAGS =
          1U << static_cast<unsigned>(nvinfer1::NetworkDefinitionCreationFlag::kSTRONGLY_TYPED);
#else
      constexpr unsigned FLAGS = 0;
#endif
      std::unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(FLAGS));
      if (!network)
        throw ArmorDetectorInitError("could not create TensorRT network");
      std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network, *logger_));
      if (!parser ||
          !parser->parseFromFile(config.model_path.c_str(),
                                 static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        throw ArmorDetectorInitError("TensorRT could not parse ONNX; see TensorRT log");
      }
      // Reject dynamic/wrong contracts before spending time building CUDA tactics.
      if (network->getNbInputs() != 1 || network->getNbOutputs() != 1 ||
          std::string_view(network->getInput(0)->getName()) != K_INPUT_NAME ||
          std::string_view(network->getOutput(0)->getName()) != K_OUTPUT_NAME ||
          network->getInput(0)->getType() != nvinfer1::DataType::kHALF ||
          network->getOutput(0)->getType() != nvinfer1::DataType::kFLOAT ||
          !HasShape(network->getInput(0)->getDimensions(), {1, 3, 640, 640}, 4) ||
          !HasShape(network->getOutput(0)->getDimensions(), {1, K_OUTPUT_ROWS, K_OUTPUT_COLUMNS, 0},
                    3)) {
        throw ArmorDetectorInitError(
            "0526 ONNX requires images: FP16 [1,3,640,640], output: FP32 [1,25200,22]");
      }
      std::unique_ptr<nvinfer1::IBuilderConfig> settings(builder->createBuilderConfig());
      if (!settings)
        throw ArmorDetectorInitError("could not create TensorRT builder config");
      settings->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30);
      MV_LOG_INFO("TensorRT", "building {} for {}; first initialization may take several minutes",
                  config.model_path.string(), device_name_);
      std::unique_ptr<nvinfer1::IHostMemory> serialized(
          builder->buildSerializedNetwork(*network, *settings));
      if (!serialized)
        throw ArmorDetectorInitError("TensorRT engine build failed; see TensorRT log");
      engine_.reset(runtime_->deserializeCudaEngine(serialized->data(), serialized->size()));
    } else {
      throw ArmorDetectorInitError("TensorRT model_path must end in .onnx or .engine");
    }
    if (!engine_)
      throw ArmorDetectorInitError(
          "could not deserialize TensorRT engine; rebuild it on this GPU with this TensorRT "
          "version");
    ValidateEngine();
    context_.reset(engine_->createExecutionContext());
    if (!context_)
      throw ArmorDetectorInitError("could not create TensorRT execution context");
    CheckCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "create CUDA stream");
    CheckCuda(cudaMalloc(&device_input_, host_input_.size() * sizeof(cv::float16_t)),
              "allocate CUDA input");
    CheckCuda(cudaMalloc(&device_output_, host_output_.size() * sizeof(float)),
              "allocate CUDA output");
    if (!context_->setTensorAddress(K_INPUT_NAME, device_input_) ||
        !context_->setTensorAddress(K_OUTPUT_NAME, device_output_)) {
      throw ArmorDetectorInitError("could not bind TensorRT input/output buffers");
    }
  }

  std::span<const float> Infer() override {
    CheckCuda(cudaSetDevice(device_index_), "select CUDA device for inference");
    // Same contract as OpenVINO: NHWC BGR U8 -> NCHW RGB FP16, normalized by 255.
    for (int row = 0; row < K_MODEL_HEIGHT; ++row) {
      const auto* pixels = input_image_.ptr<cv::Vec3b>(row);
      for (int col = 0; col < K_MODEL_WIDTH; ++col) {
        const std::size_t INDEX = static_cast<std::size_t>(row) * K_MODEL_WIDTH + col;
        for (int channel = 0; channel < 3; ++channel) {
          host_input_[channel * K_INPUT_PIXELS + INDEX] =
              cv::float16_t(static_cast<float>(pixels[col][2 - channel]) / 255.0F);
        }
      }
    }
    try {
      CheckCuda(cudaMemcpyAsync(device_input_, host_input_.data(),
                                host_input_.size() * sizeof(cv::float16_t), cudaMemcpyHostToDevice,
                                stream_),
                "copy TensorRT input");
      if (!context_->enqueueV3(stream_))
        throw ArmorDetectorRuntimeError("TensorRT enqueueV3 failed");
      CheckCuda(
          cudaMemcpyAsync(host_output_.data(), device_output_, host_output_.size() * sizeof(float),
                          cudaMemcpyDeviceToHost, stream_),
          "copy TensorRT output");
      CheckCuda(cudaStreamSynchronize(stream_), "synchronize TensorRT inference");
    } catch (...) {
      // A failed enqueue/copy can leave earlier operations pending. Do not let the next
      // Detect overwrite host buffers until those operations have finished.
      (void)cudaStreamSynchronize(stream_);
      throw;
    }
    return host_output_;
  }

  std::string Name() const override { return "tensorrt"; }
  std::string DeviceName() const override { return device_name_; }

 private:
  void ValidateEngine() const {
    if (engine_->getNbIOTensors() != 2 ||
        engine_->getTensorIOMode(K_INPUT_NAME) != nvinfer1::TensorIOMode::kINPUT ||
        engine_->getTensorIOMode(K_OUTPUT_NAME) != nvinfer1::TensorIOMode::kOUTPUT ||
        engine_->getTensorDataType(K_INPUT_NAME) != nvinfer1::DataType::kHALF ||
        engine_->getTensorDataType(K_OUTPUT_NAME) != nvinfer1::DataType::kFLOAT ||
        !HasShape(engine_->getTensorShape(K_INPUT_NAME), {1, 3, 640, 640}, 4) ||
        !HasShape(engine_->getTensorShape(K_OUTPUT_NAME), {1, K_OUTPUT_ROWS, K_OUTPUT_COLUMNS, 0},
                  3)) {
      throw ArmorDetectorInitError(
          "0526 TensorRT engine requires images: FP16 [1,3,640,640], output: FP32 [1,25200,22]");
    }
    for (const auto* name : {K_INPUT_NAME, K_OUTPUT_NAME}) {
      if (engine_->getTensorLocation(name) != nvinfer1::TensorLocation::kDEVICE ||
          engine_->getTensorFormat(name) != nvinfer1::TensorFormat::kLINEAR) {
        throw ArmorDetectorInitError("0526 TensorRT input/output must use linear device tensors");
      }
    }
  }

  // Logger outlives all TensorRT objects; context is destroyed before engine/runtime.
  std::shared_ptr<TensorRtLogger> logger_{GetTensorRtLogger()};
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  int device_index_{-1};
  std::string device_name_;
  cv::Mat input_image_;
  std::vector<cv::float16_t> host_input_;
  std::vector<float> host_output_;
  cudaStream_t stream_{nullptr};
  void* device_input_{nullptr};
  void* device_output_{nullptr};
};
}  // namespace

bool TensorRtDeviceAvailable(int device) {
  int count = 0;
  if (cudaGetDeviceCount(&count) != cudaSuccess || device < 0 || device >= count)
    return false;
  cudaDeviceProp properties{};
  return cudaGetDeviceProperties(&properties, device) == cudaSuccess && properties.major > 0;
}

std::unique_ptr<ArmorInferenceBackend> MakeTensorRtBackend() {
  return std::make_unique<TensorRtBackend>();
}
}  // namespace mv::modules::detail
