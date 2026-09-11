#pragma once

#include "modules/armor_detector/armor_detector.hpp"

#include <memory>
#include <string>

#include <span>

namespace mv::modules::detail {

/** @brief 后端持有固定输入画布的共享引用，输出 span 有效至下一次 Infer() 或销毁。 */
class ArmorInferenceBackend {
 public:
  virtual ~ArmorInferenceBackend() = default;
  /** @brief 绑定 640×640 BGR U8 画布；调用方只更新像素，不更换缓冲区。 */
  virtual void Initialize(const ArmorDetectorConfig& config, const cv::Mat& input) = 0;
  /** @brief 同步返回 FP32 [1,25200,22]；包含布局/精度转换与必要的主机设备传输。 */
  [[nodiscard]] virtual std::span<const float> Infer() = 0;
  [[nodiscard]] virtual std::string Name() const = 0;
  [[nodiscard]] virtual std::string DeviceName() const = 0;
};

/** @brief 仅在已编译后端中选取设备；auto 优先 NVIDIA，之后 Intel GPU，不回退 CPU。 */
[[nodiscard]] std::unique_ptr<ArmorInferenceBackend> MakeInferenceBackend(
    const ArmorDetectorConfig& config);
/** @brief GPU 表示设备 0；GPU.<非负整数> 表示明确的设备序号，非法字符串抛异常。 */
[[nodiscard]] int ParseGpuDeviceIndex(const std::string& device);

#if MV_USE_OPENVINO
[[nodiscard]] std::unique_ptr<ArmorInferenceBackend> MakeOpenVinoBackend();
[[nodiscard]] bool OpenVinoDeviceAvailable(const std::string& device);
#endif
#if MV_USE_TENSORRT
[[nodiscard]] std::unique_ptr<ArmorInferenceBackend> MakeTensorRtBackend();
[[nodiscard]] bool TensorRtDeviceAvailable(int device);
#endif

}  // namespace mv::modules::detail
