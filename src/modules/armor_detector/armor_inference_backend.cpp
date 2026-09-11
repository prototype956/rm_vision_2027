#include "modules/armor_detector/armor_inference_backend.hpp"

#include <string_view>

#include <charconv>

namespace mv::modules {

std::string_view ArmorInferenceBackendName(ArmorInferenceBackend backend) noexcept {
  switch (backend) {
    case ArmorInferenceBackend::AUTO:
      return "auto";
    case ArmorInferenceBackend::OPENVINO:
      return "openvino";
    case ArmorInferenceBackend::TENSORRT:
      return "tensorrt";
  }
  return "unknown";
}

namespace detail {
int ParseGpuDeviceIndex(const std::string& device) {
  if (device == "GPU")
    return 0;
  constexpr std::string_view PREFIX = "GPU.";
  if (device.starts_with(PREFIX) && device.size() > PREFIX.size()) {
    const auto* begin = device.data() + PREFIX.size();
    const auto* end = device.data() + device.size();
    int index = -1;
    const auto RESULT = std::from_chars(begin, end, index);
    if (*begin >= '0' && *begin <= '9' && RESULT.ec == std::errc{} && RESULT.ptr == end &&
        index >= 0)
      return index;
  }
  throw ArmorDetectorInitError("device must be GPU or GPU.<nonnegative integer>");
}

std::unique_ptr<ArmorInferenceBackend> MakeInferenceBackend(const ArmorDetectorConfig& config) {
  const int DEVICE_INDEX = ParseGpuDeviceIndex(config.device);
  if (config.backend == mv::modules::ArmorInferenceBackend::TENSORRT) {
#if MV_USE_TENSORRT
    return MakeTensorRtBackend();
#else
    throw ArmorDetectorInitError("TensorRT backend was not built; configure USE_TENSORRT=ON");
#endif
  }
  if (config.backend == mv::modules::ArmorInferenceBackend::OPENVINO) {
#if MV_USE_OPENVINO
    return MakeOpenVinoBackend();
#else
    throw ArmorDetectorInitError("OpenVINO backend was not built; configure USE_OPENVINO=ON");
#endif
  }
  if (config.backend != mv::modules::ArmorInferenceBackend::AUTO) {
    throw ArmorDetectorInitError("invalid inference backend");
  }
#if MV_USE_TENSORRT
  if (TensorRtDeviceAvailable(DEVICE_INDEX))
    return MakeTensorRtBackend();
#else
  (void)DEVICE_INDEX;
#endif
#if MV_USE_OPENVINO
  if (OpenVinoDeviceAvailable(config.device))
    return MakeOpenVinoBackend();
#endif
  throw ArmorDetectorInitError(
      "auto backend found no compatible GPU in the compiled backends for " + config.device +
      "; Intel GPU requires USE_OPENVINO=ON, NVIDIA GPU requires USE_TENSORRT=ON");
}
}  // namespace detail
}  // namespace mv::modules
