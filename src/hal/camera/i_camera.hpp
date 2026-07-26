#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace mv::hal {

/**
 * @brief HAL 向上层提供的图像像素格式。
 *
 * 当前两个相机后端都只接受 BGR8 配置；MONO8 为后续单色相机支持预留。
 */
enum class PixelFormat : uint8_t { UNKNOWN = 0, BGR8, MONO8 };

/**
 * @brief 单次取帧结果。
 *
 * TIMEOUT 和 INVALID_FRAME 通常允许调用者继续取下一帧；DISCONNECTED 和 FATAL
 * 表示当前相机实例无法继续正常抓帧，应退出循环并执行 Close()。
 */
enum class GrabStatus : uint8_t { OK = 0, TIMEOUT, DISCONNECTED, INVALID_FRAME, FATAL };

/**
 * @brief 将取帧状态转换为稳定的日志字符串。
 *
 * @param status 待转换的取帧状态。
 * @return 静态字符串，生命周期覆盖整个进程。
 */
[[nodiscard]] inline const char* GrabStatusName(GrabStatus status) noexcept {
  switch (status) {
    case GrabStatus::OK:
      return "ok";
    case GrabStatus::TIMEOUT:
      return "timeout";
    case GrabStatus::DISCONNECTED:
      return "disconnected";
    case GrabStatus::INVALID_FRAME:
      return "invalid_frame";
    case GrabStatus::FATAL:
      return "fatal";
  }
  return "unknown";
}

/**
 * @brief HAL 成功抓取的一帧图像及其元数据。
 *
 * 仅当 ICamera::Grab() 返回 GrabStatus::OK 时，调用者才能读取本结构中的内容。
 * image 独立持有有效像素数据，不依赖相机驱动的 DMA 缓冲区生命周期。
 */
struct CameraFrame {
  cv::Mat image;                                      ///< OpenCV 图像矩阵。
  std::chrono::steady_clock::time_point timestamp{};  ///< HAL 完成该帧处理的单调时钟时间。
  uint64_t sequence{0};                               ///< 本次 Open() 后从 0 递增的帧序号。
};

/**
 * @brief 相机成功打开后实际生效的设备与成像参数。
 *
 * 字段由具体后端在 Open() 成功前填充。调用者应使用这里的实际值校验输出，不能
 * 假定驱动一定接受了配置文件中的请求值。
 */
struct CameraInfo {
  std::string device_name;                         ///< 后端提供的设备名称或输入源标识。
  int sensor_width{0};                             ///< 传感器或后端报告的原始宽度。
  int sensor_height{0};                            ///< 传感器或后端报告的原始高度。
  int output_width{0};                             ///< 每帧输出宽度。
  int output_height{0};                            ///< 每帧输出高度。
  int roi_offset_x{0};                             ///< 硬件 ROI 左上角横坐标。
  int roi_offset_y{0};                             ///< 硬件 ROI 左上角纵坐标。
  int exposure_us{0};                              ///< 曝光时间，单位为微秒。
  int grab_timeout_ms{0};                          ///< 单次阻塞取帧超时，单位为毫秒。
  PixelFormat pixel_format{PixelFormat::UNKNOWN};  ///< 每帧实际像素格式。
};

/**
 * @brief 相机后端统一接口。
 *
 * 标准调用顺序为 Open() -> Info() -> 重复 Grab() -> Close()。对象不可拷贝，具体
 * 后端负责管理设备句柄和图像缓冲区；析构时也会释放已打开的设备。
 */
class ICamera {
 public:
  ICamera() = default;
  virtual ~ICamera() = default;

  ICamera(const ICamera&) = delete;
  ICamera& operator=(const ICamera&) = delete;

 protected:
  ICamera(ICamera&&) = default;
  ICamera& operator=(ICamera&&) = default;

 public:
  /**
   * @brief 根据配置打开并启动相机。
   *
   * 重复调用已打开的相机是幂等操作。返回 false 后相机仍视为关闭状态。
   *
   * @param config 具体后端对应的 YAML 配置根节点。
   * @return 成功打开并开始采集时返回 true，否则返回 false。
   */
  virtual bool Open(const YAML::Node& config) = 0;

  /**
   * @brief 停止采集并释放相机资源。
   *
   * 允许对已关闭的相机重复调用。
   */
  virtual void Close() = 0;

  /**
   * @brief 阻塞等待并返回一帧图像。
   *
   * @param[out] frame 成功时写入完整图像和元数据；非 OK 状态下内容未定义。
   * @return 本次取帧状态，调用者应根据状态决定继续、重试或退出。
   */
  virtual GrabStatus Grab(CameraFrame& frame) = 0;

  /**
   * @brief 获取当前相机参数快照。
   *
   * @return Open() 成功后实际生效的设备和输出信息。
   */
  [[nodiscard]] virtual CameraInfo Info() const = 0;

  /**
   * @brief 检查相机是否处于已打开状态。
   *
   * @return 可以继续调用 Grab() 时返回 true。
   */
  [[nodiscard]] virtual bool IsOpen() const = 0;
};

}  // namespace mv::hal
