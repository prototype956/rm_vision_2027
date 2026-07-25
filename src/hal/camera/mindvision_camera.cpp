#include "mindvision_camera.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core/core_c.h>
#include <opencv2/imgproc.hpp>

#ifdef MV_HAS_MVSDK
#include <CameraApi.h>
#endif

#include "../../core/logger.hpp"
#include "../../core/config.hpp"

namespace mv::hal {

// ============================================================================
// Impl 定义（所有 SDK 类型都在这里，不污染头文件）
// ============================================================================

struct MindVisionCamera::Impl {
  bool is_open{false};

  // 图像参数
  int width{1280};
  int height{720};
  int exposure_us{5000};
  int grab_timeout_ms{100};
  int channel{3};
  uint64_t sequence{0};
  CameraInfo info{};

#ifdef MV_HAS_MVSDK
  int h_camera{0};
  std::vector<unsigned char> rgb_buffer{};

  tSdkCameraDevInfo dev_info{};
  tSdkCameraCapbility capability{};
  tSdkFrameHead frame_head{};
  tSdkImageResolution resolution{};
  BYTE* raw_buffer{nullptr};
  IplImage* ipl_image{nullptr};
#endif
};

// ============================================================================
// 构造 / 析构 / 移动
// ============================================================================

MindVisionCamera::MindVisionCamera() : impl_(std::make_unique<Impl>()) {}

// 析构必须在 .cpp 定义，因为此时 Impl 是完整类型
MindVisionCamera::~MindVisionCamera() {
  Close();
}

MindVisionCamera::MindVisionCamera(MindVisionCamera&&) noexcept = default;
MindVisionCamera& MindVisionCamera::operator=(MindVisionCamera&&) noexcept = default;

// ============================================================================
// ICamera 接口实现
// ============================================================================

bool MindVisionCamera::Open(const YAML::Node& config) {
#ifndef MV_HAS_MVSDK
  // 无 SDK 时记录日志并返回 false，而不是编译错误
  // 这让 CI 和开发机在没有 MindVision 硬件时仍能跑通流水线
  (void)config;  // suppress unused parameter warning in stub mode
  MV_LOG_WARN("HAL.Camera.MV", "MV_HAS_MVSDK not defined, running in stub mode");
  return false;
#else
  if (impl_->is_open) {
    return true;  // 幂等：已打开则直接返回
  }

  int device_index = 0;
  bool centered_roi = true;
  bool auto_exposure = false;
  try {
    ConfigLoader::RejectUnknownKeys(
        config, {"schema_version", "device", "output", "roi", "exposure", "capture"},
        "MindVision camera config");
    if (ConfigLoader::Require<int>(config, "schema_version", "MindVision camera config") != 1) {
      throw ConfigError("MindVision camera config schema_version must be 1");
    }

    const auto device = config["device"];
    ConfigLoader::RequireMap(device, "MindVision camera config.device");
    ConfigLoader::RejectUnknownKeys(device, {"index"}, "MindVision camera config.device");
    device_index = ConfigLoader::Require<int>(device, "index", "MindVision camera config.device");

    const auto output = config["output"];
    ConfigLoader::RequireMap(output, "MindVision camera config.output");
    ConfigLoader::RejectUnknownKeys(output, {"width", "height", "pixel_format"},
                                    "MindVision camera config.output");
    impl_->width =
        ConfigLoader::Require<int>(output, "width", "MindVision camera config.output");
    impl_->height =
        ConfigLoader::Require<int>(output, "height", "MindVision camera config.output");
    const auto pixel_format = ConfigLoader::Require<std::string>(
        output, "pixel_format", "MindVision camera config.output");
    if (pixel_format != "bgr8") {
      throw ConfigError("MindVision camera currently requires output.pixel_format=bgr8");
    }

    const auto roi = config["roi"];
    ConfigLoader::RequireMap(roi, "MindVision camera config.roi");
    ConfigLoader::RejectUnknownKeys(roi, {"centered"}, "MindVision camera config.roi");
    centered_roi =
        ConfigLoader::Require<bool>(roi, "centered", "MindVision camera config.roi");
    if (!centered_roi) {
      throw ConfigError("MindVision camera currently requires roi.centered=true");
    }

    const auto exposure = config["exposure"];
    ConfigLoader::RequireMap(exposure, "MindVision camera config.exposure");
    ConfigLoader::RejectUnknownKeys(exposure, {"auto", "time_us"},
                                    "MindVision camera config.exposure");
    auto_exposure =
        ConfigLoader::Require<bool>(exposure, "auto", "MindVision camera config.exposure");
    impl_->exposure_us =
        ConfigLoader::Require<int>(exposure, "time_us", "MindVision camera config.exposure");

    const auto capture = config["capture"];
    ConfigLoader::RequireMap(capture, "MindVision camera config.capture");
    ConfigLoader::RejectUnknownKeys(capture, {"timeout_ms"},
                                    "MindVision camera config.capture");
    impl_->grab_timeout_ms =
        ConfigLoader::Require<int>(capture, "timeout_ms", "MindVision camera config.capture");

    if (device_index < 0 || impl_->width <= 0 || impl_->height <= 0 ||
        impl_->exposure_us <= 0 || impl_->grab_timeout_ms <= 0) {
      throw ConfigError("MindVision device index and numeric camera parameters must be positive");
    }
  } catch (const std::exception& error) {
    MV_LOG_ERROR("HAL.Camera.MV", "invalid config: {}", error.what());
    return false;
  }

  CameraSdkInit(1);

  std::vector<tSdkCameraDevInfo> devices(8);
  int camera_count = static_cast<int>(devices.size());
  int status = CameraEnumerateDevice(devices.data(), &camera_count);
  if (status != CAMERA_STATUS_SUCCESS || camera_count <= 0) {
    MV_LOG_ERROR("HAL.Camera.MV", "no device found (CameraEnumerateDevice returned {})", status);
    return false;
  }
  if (device_index >= camera_count) {
    MV_LOG_ERROR("HAL.Camera.MV", "device.index={} out of range; {} camera(s) found", device_index,
                 camera_count);
    return false;
  }
  impl_->dev_info = devices[static_cast<std::size_t>(device_index)];

  status = CameraInit(&impl_->dev_info, -1, -1, &impl_->h_camera);
  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraInit failed, status={}", status);
    return false;
  }

  status = CameraGetCapability(impl_->h_camera, &impl_->capability);
  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraGetCapability failed, status={}", status);
    CameraUnInit(impl_->h_camera);
    return false;
  }

  // 显式固定 ISP 输出格式，避免 SDK 默认格式与上层 BGR 假设不一致。
  if (impl_->capability.sIspCapacity.bMonoSensor) {
    MV_LOG_ERROR("HAL.Camera.MV", "selected camera is monochrome but BGR8 was requested");
    CameraUnInit(impl_->h_camera);
    return false;
  }
  impl_->channel = 3;
  status = CameraSetIspOutFormat(impl_->h_camera, CAMERA_MEDIA_TYPE_BGR8);
  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraSetIspOutFormat(BGR8) failed, status={}", status);
    CameraUnInit(impl_->h_camera);
    return false;
  }

  const int sensor_width = impl_->capability.sResolutionRange.iWidthMax;
  const int sensor_height = impl_->capability.sResolutionRange.iHeightMax;
  const int sensor_min_width = impl_->capability.sResolutionRange.iWidthMin;
  const int sensor_min_height = impl_->capability.sResolutionRange.iHeightMin;
  if (impl_->width < sensor_min_width || impl_->width > sensor_width ||
      impl_->height < sensor_min_height || impl_->height > sensor_height) {
    MV_LOG_ERROR("HAL.Camera.MV",
                 "requested {}x{} outside camera ROI capability width=[{},{}] height=[{},{}]",
                 impl_->width, impl_->height, sensor_min_width, sensor_width, sensor_min_height,
                 sensor_height);
    CameraUnInit(impl_->h_camera);
    return false;
  }

  // 分配 RGB 缓冲区（大小取相机支持的最大分辨率，避免反复 realloc）
  std::size_t buf_size = static_cast<std::size_t>(impl_->capability.sResolutionRange.iHeightMax) *
                         static_cast<std::size_t>(impl_->capability.sResolutionRange.iWidthMax) * 3;
  try {
    impl_->rgb_buffer.resize(buf_size);
  } catch (const std::bad_alloc&) {
    MV_LOG_ERROR("HAL.Camera.MV", "failed to allocate RGB buffer ({} bytes)", buf_size);
    CameraUnInit(impl_->h_camera);
    return false;
  }

  // 设置分辨率
  status = CameraGetImageResolution(impl_->h_camera, &impl_->resolution);
  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraGetImageResolution failed, status={}", status);
    CameraUnInit(impl_->h_camera);
    return false;
  }
  impl_->resolution.iIndex = 0xFF;  // 0xFF = 自定义分辨率
  impl_->resolution.uBinSumMode = 0;
  impl_->resolution.uBinAverageMode = 0;
  impl_->resolution.uSkipMode = 0;
  impl_->resolution.uResampleMask = 0;
  impl_->resolution.iHOffsetFOV = std::max(0, (sensor_width - impl_->width) / 2);
  impl_->resolution.iVOffsetFOV = std::max(0, (sensor_height - impl_->height) / 2);
  // Most MindVision sensors require even ROI offsets.
  impl_->resolution.iHOffsetFOV -= impl_->resolution.iHOffsetFOV % 2;
  impl_->resolution.iVOffsetFOV -= impl_->resolution.iVOffsetFOV % 2;
  impl_->resolution.iWidthFOV = impl_->width;
  impl_->resolution.iHeightFOV = impl_->height;
  impl_->resolution.iWidth = impl_->width;
  impl_->resolution.iHeight = impl_->height;
  impl_->resolution.iWidthZoomHd = 0;
  impl_->resolution.iHeightZoomHd = 0;
  impl_->resolution.iWidthZoomSw = 0;
  impl_->resolution.iHeightZoomSw = 0;
  status = CameraSetImageResolution(impl_->h_camera, &impl_->resolution);
  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR(
        "HAL.Camera.MV",
        "CameraSetImageResolution {}x{} ROI=({}, {}) failed, status={}; capability "
        "width=[{},{}] height=[{},{}]",
        impl_->width, impl_->height, impl_->resolution.iHOffsetFOV,
        impl_->resolution.iVOffsetFOV, status, sensor_min_width, sensor_width, sensor_min_height,
        sensor_height);
    CameraUnInit(impl_->h_camera);
    return false;
  }

  tSdkImageResolution actual_resolution{};
  status = CameraGetImageResolution(impl_->h_camera, &actual_resolution);
  if (status != CAMERA_STATUS_SUCCESS || actual_resolution.iWidth != impl_->width ||
      actual_resolution.iHeight != impl_->height) {
    MV_LOG_ERROR("HAL.Camera.MV", "resolution verification failed: requested {}x{}, actual {}x{}, "
                                  "status={}",
                 impl_->width, impl_->height, actual_resolution.iWidth, actual_resolution.iHeight,
                 status);
    CameraUnInit(impl_->h_camera);
    return false;
  }
  impl_->resolution = actual_resolution;

  // 设置曝光（关闭 AE，手动控制）
  status = CameraSetAeState(impl_->h_camera, auto_exposure ? TRUE : FALSE);
  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraSetAeState failed, status={}", status);
    CameraUnInit(impl_->h_camera);
    return false;
  }
  if (!auto_exposure) {
    status = CameraSetExposureTime(impl_->h_camera, static_cast<double>(impl_->exposure_us));
    if (status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraSetExposureTime failed, status={}", status);
      CameraUnInit(impl_->h_camera);
      return false;
    }
    double actual_exposure_us = 0.0;
    status = CameraGetExposureTime(impl_->h_camera, &actual_exposure_us);
    if (status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraGetExposureTime failed, status={}", status);
      CameraUnInit(impl_->h_camera);
      return false;
    }
    impl_->exposure_us = static_cast<int>(std::lround(actual_exposure_us));
  }

  status = CameraPlay(impl_->h_camera);
  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraPlay failed, status={}", status);
    CameraUnInit(impl_->h_camera);
    return false;
  }

  impl_->info.device_name =
      std::string(impl_->dev_info.acProductName) + " sn=" + impl_->dev_info.acSn;
  impl_->info.sensor_width = sensor_width;
  impl_->info.sensor_height = sensor_height;
  impl_->info.output_width = impl_->width;
  impl_->info.output_height = impl_->height;
  impl_->info.roi_offset_x = impl_->resolution.iHOffsetFOV;
  impl_->info.roi_offset_y = impl_->resolution.iVOffsetFOV;
  impl_->info.exposure_us = impl_->exposure_us;
  impl_->info.grab_timeout_ms = impl_->grab_timeout_ms;
  impl_->info.pixel_format = PixelFormat::BGR8;
  impl_->sequence = 0;
  impl_->is_open = true;
  MV_LOG_INFO("HAL.Camera.MV",
              "opened '{}' sensor={}x{} requested={}x{} actual={}x{} ROI=({}, {}) BGR8 "
              "exposure={}us timeout={}ms",
              impl_->info.device_name, sensor_width, sensor_height, impl_->width, impl_->height,
              impl_->resolution.iWidth, impl_->resolution.iHeight,
              impl_->resolution.iHOffsetFOV, impl_->resolution.iVOffsetFOV, impl_->exposure_us,
              impl_->grab_timeout_ms);
  return true;
#endif
}

void MindVisionCamera::Close() {
#ifdef MV_HAS_MVSDK
  if (!impl_->is_open) {
    return;  // 幂等
  }

  const int status = CameraUnInit(impl_->h_camera);
  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraUnInit failed, status={}", status);
  }

  impl_->rgb_buffer.clear();
  impl_->rgb_buffer.shrink_to_fit();
  if (impl_->ipl_image != nullptr) {
    cvReleaseImageHeader(&impl_->ipl_image);
    impl_->ipl_image = nullptr;
  }

  impl_->is_open = false;
  MV_LOG_INFO("HAL.Camera.MV", "closed");
#endif
}

GrabStatus MindVisionCamera::Grab(CameraFrame& frame) {
#ifndef MV_HAS_MVSDK
  (void)frame;
  return GrabStatus::FATAL;
#else
  if (!impl_->is_open) {
    MV_LOG_WARN("HAL.Camera.MV", "Grab called on closed camera");
    return GrabStatus::DISCONNECTED;
  }

  int status = CameraGetImageBuffer(impl_->h_camera, &impl_->frame_head, &impl_->raw_buffer,
                                    impl_->grab_timeout_ms);

  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_WARN("HAL.Camera.MV", "CameraGetImageBuffer timeout/error, status={}", status);
    if (status == CAMERA_STATUS_TIME_OUT) {
      return GrabStatus::TIMEOUT;
    }
    if (status == CAMERA_STATUS_DEVICE_LOST || status == CAMERA_STATUS_NOT_INITIALIZED ||
        status == CAMERA_STATUS_USB_CONTROL_ERROR || status == CAMERA_STATUS_USB_BULK_ERROR) {
      return GrabStatus::DISCONNECTED;
    }
    return GrabStatus::FATAL;
  }

  // SDK 做去马赛克 / 颜色空间转换，结果写入 rgb_buffer
  status = CameraImageProcess(impl_->h_camera, impl_->raw_buffer, impl_->rgb_buffer.data(),
                              &impl_->frame_head);
  if (status != CAMERA_STATUS_SUCCESS) {
    const int release_status = CameraReleaseImageBuffer(impl_->h_camera, impl_->raw_buffer);
    impl_->raw_buffer = nullptr;
    if (release_status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraReleaseImageBuffer failed after process error, status={}",
                   release_status);
      return GrabStatus::FATAL;
    }
    MV_LOG_WARN("HAL.Camera.MV", "CameraImageProcess failed, status={}", status);
    return GrabStatus::INVALID_FRAME;
  }

  // 用 IplImage 头包装 rgb_buffer（零拷贝）
  if (impl_->ipl_image != nullptr) {
    cvReleaseImageHeader(&impl_->ipl_image);
  }
  impl_->ipl_image = cvCreateImageHeader(
      cvSize(impl_->frame_head.iWidth, impl_->frame_head.iHeight), IPL_DEPTH_8U, impl_->channel);
  cvSetData(impl_->ipl_image, impl_->rgb_buffer.data(), impl_->frame_head.iWidth * impl_->channel);

  // 深拷贝到 cv::Mat，之后立即归还 DMA buffer
  // 深拷贝原因见文件头注释：尽快释放 SDK buffer 防止丢帧
  cv::Mat image = cv::cvarrToMat(impl_->ipl_image, true);

  const int release_status = CameraReleaseImageBuffer(impl_->h_camera, impl_->raw_buffer);
  impl_->raw_buffer = nullptr;
  if (release_status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraReleaseImageBuffer failed, status={}", release_status);
    return GrabStatus::FATAL;
  }

  if (image.empty() || image.cols != impl_->width || image.rows != impl_->height ||
      image.type() != CV_8UC3) {
    MV_LOG_WARN("HAL.Camera.MV", "invalid frame: {}x{} type={} expected {}x{} CV_8UC3", image.cols,
                image.rows, image.type(), impl_->width, impl_->height);
    return GrabStatus::INVALID_FRAME;
  }

  frame.image = std::move(image);
  frame.timestamp = std::chrono::steady_clock::now();
  frame.sequence = impl_->sequence++;
  return GrabStatus::OK;
#endif
}

CameraInfo MindVisionCamera::Info() const {
  return impl_->info;
}

bool MindVisionCamera::IsOpen() const {
  return impl_->is_open;
}

}  // namespace mv::hal
