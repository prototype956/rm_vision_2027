#include "mindvision_device.hpp"

#include "core/logger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#ifdef MV_HAS_MVSDK
#include <CameraApi.h>
#endif

namespace mv::hal::detail {
namespace {

#ifdef MV_HAS_MVSDK

int InitializeSdkOnce() {
  static std::once_flag flag;
  static int status = CAMERA_STATUS_FAILED;
  std::call_once(flag, [] { status = CameraSdkInit(1); });
  return status;
}

bool IsDisconnectedStatus(int status) {
  return status == CAMERA_STATUS_DEVICE_LOST || status == CAMERA_STATUS_NOT_INITIALIZED ||
         status == CAMERA_STATUS_USB_CONTROL_ERROR || status == CAMERA_STATUS_USB_BULK_ERROR;
}

class RawBufferLease final {
 public:
  RawBufferLease(int camera_handle, BYTE* buffer)
      : camera_handle_(camera_handle), buffer_(buffer) {}

  ~RawBufferLease() {
    if (buffer_ != nullptr) {
      const int STATUS = CameraReleaseImageBuffer(camera_handle_, buffer_);
      if (STATUS != CAMERA_STATUS_SUCCESS) {
        MV_LOG_ERROR("HAL.Camera.MV", "CameraReleaseImageBuffer failed in cleanup, status={}",
                     STATUS);
      }
    }
  }

  RawBufferLease(const RawBufferLease&) = delete;
  RawBufferLease& operator=(const RawBufferLease&) = delete;

  int Release() noexcept {
    if (buffer_ == nullptr)
      return CAMERA_STATUS_SUCCESS;
    const int STATUS = CameraReleaseImageBuffer(camera_handle_, buffer_);
    buffer_ = nullptr;
    return STATUS;
  }

 private:
  int camera_handle_;
  BYTE* buffer_;
};

#endif

}  // namespace

struct MindVisionDevice::Impl {
  bool handle_valid{false};
  bool streaming{false};
  uint64_t sequence{0};
  CameraInfo info{};
  MindVisionConfig config{};
  bool timeout_active{false};
  std::chrono::steady_clock::time_point last_timeout_log{};

#ifdef MV_HAS_MVSDK
  int camera_handle{0};
  tSdkCameraDevInfo device_info{};
  tSdkCameraCapbility capability{};
  tSdkImageResolution resolution{};

  void Reset(bool log_close) noexcept {
    const bool WAS_ACTIVE = handle_valid || streaming;
    if (handle_valid) {
      const int STATUS = CameraUnInit(camera_handle);
      if (STATUS != CAMERA_STATUS_SUCCESS) {
        MV_LOG_ERROR("HAL.Camera.MV", "CameraUnInit failed, status={}", STATUS);
      }
    }
    handle_valid = false;
    streaming = false;
    camera_handle = 0;
    timeout_active = false;
    if (log_close && WAS_ACTIVE)
      MV_LOG_INFO("HAL.Camera.MV", "closed");
  }

  bool SelectDevice() {
    std::vector<tSdkCameraDevInfo> devices(8);
    int camera_count = static_cast<int>(devices.size());
    const int STATUS = CameraEnumerateDevice(devices.data(), &camera_count);
    if (STATUS != CAMERA_STATUS_SUCCESS || camera_count <= 0) {
      MV_LOG_ERROR("HAL.Camera.MV", "no device found (CameraEnumerateDevice returned {})", STATUS);
      return false;
    }
    if (config.device_index >= camera_count) {
      MV_LOG_ERROR("HAL.Camera.MV", "device.index={} out of range; {} camera(s) found",
                   config.device_index, camera_count);
      return false;
    }
    device_info = devices[static_cast<std::size_t>(config.device_index)];
    return true;
  }

  bool InitializeHandle() {
    const int STATUS = CameraInit(&device_info, -1, -1, &camera_handle);
    if (STATUS != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraInit failed, status={}", STATUS);
      return false;
    }
    handle_valid = true;
    return true;
  }

  bool ConfigureOutput() {
    int status = CameraGetCapability(camera_handle, &capability);
    if (status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraGetCapability failed, status={}", status);
      return false;
    }
    if (capability.sIspCapacity.bMonoSensor) {
      MV_LOG_ERROR("HAL.Camera.MV", "selected camera is monochrome but BGR8 was requested");
      return false;
    }
    status = CameraSetIspOutFormat(camera_handle, CAMERA_MEDIA_TYPE_BGR8);
    if (status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraSetIspOutFormat(BGR8) failed, status={}", status);
      return false;
    }
    return true;
  }

  bool ConfigureResolution() {
    const int SENSOR_WIDTH = capability.sResolutionRange.iWidthMax;
    const int SENSOR_HEIGHT = capability.sResolutionRange.iHeightMax;
    const int MIN_WIDTH = capability.sResolutionRange.iWidthMin;
    const int MIN_HEIGHT = capability.sResolutionRange.iHeightMin;
    if (config.width < MIN_WIDTH || config.width > SENSOR_WIDTH || config.height < MIN_HEIGHT ||
        config.height > SENSOR_HEIGHT) {
      MV_LOG_ERROR("HAL.Camera.MV",
                   "requested {}x{} outside camera ROI capability width=[{},{}] height=[{},{}]",
                   config.width, config.height, MIN_WIDTH, SENSOR_WIDTH, MIN_HEIGHT, SENSOR_HEIGHT);
      return false;
    }

    int status = CameraGetImageResolution(camera_handle, &resolution);
    if (status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraGetImageResolution failed, status={}", status);
      return false;
    }
    resolution.iIndex = 0xFF;
    resolution.uBinSumMode = 0;
    resolution.uBinAverageMode = 0;
    resolution.uSkipMode = 0;
    resolution.uResampleMask = 0;
    resolution.iHOffsetFOV = std::max(0, (SENSOR_WIDTH - config.width) / 2);
    resolution.iVOffsetFOV = std::max(0, (SENSOR_HEIGHT - config.height) / 2);
    resolution.iHOffsetFOV -= resolution.iHOffsetFOV % 2;
    resolution.iVOffsetFOV -= resolution.iVOffsetFOV % 2;
    resolution.iWidthFOV = config.width;
    resolution.iHeightFOV = config.height;
    resolution.iWidth = config.width;
    resolution.iHeight = config.height;
    resolution.iWidthZoomHd = 0;
    resolution.iHeightZoomHd = 0;
    resolution.iWidthZoomSw = 0;
    resolution.iHeightZoomSw = 0;

    status = CameraSetImageResolution(camera_handle, &resolution);
    if (status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraSetImageResolution {}x{} ROI=({}, {}) failed, status={}",
                   config.width, config.height, resolution.iHOffsetFOV, resolution.iVOffsetFOV,
                   status);
      return false;
    }

    tSdkImageResolution actual{};
    status = CameraGetImageResolution(camera_handle, &actual);
    if (status != CAMERA_STATUS_SUCCESS || actual.iWidth != config.width ||
        actual.iHeight != config.height) {
      MV_LOG_ERROR("HAL.Camera.MV",
                   "resolution verification failed: requested {}x{}, actual {}x{}, status={}",
                   config.width, config.height, actual.iWidth, actual.iHeight, status);
      return false;
    }
    resolution = actual;
    return true;
  }

  bool ConfigureExposure() {
    int status = CameraSetAeState(camera_handle, config.auto_exposure ? TRUE : FALSE);
    if (status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraSetAeState failed, status={}", status);
      return false;
    }
    if (config.auto_exposure)
      return true;

    status = CameraSetExposureTime(camera_handle, static_cast<double>(config.exposure_us));
    if (status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraSetExposureTime failed, status={}", status);
      return false;
    }
    double actual_exposure_us = 0.0;
    status = CameraGetExposureTime(camera_handle, &actual_exposure_us);
    if (status != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraGetExposureTime failed, status={}", status);
      return false;
    }
    config.exposure_us = static_cast<int>(std::lround(actual_exposure_us));
    return true;
  }

  bool StartStreaming() {
    const int STATUS = CameraPlay(camera_handle);
    if (STATUS != CAMERA_STATUS_SUCCESS) {
      MV_LOG_ERROR("HAL.Camera.MV", "CameraPlay failed, status={}", STATUS);
      return false;
    }
    streaming = true;
    sequence = 0;
    info.device_name = std::string(device_info.acProductName) + " sn=" + device_info.acSn;
    info.sensor_width = capability.sResolutionRange.iWidthMax;
    info.sensor_height = capability.sResolutionRange.iHeightMax;
    info.output_width = config.width;
    info.output_height = config.height;
    info.roi_offset_x = resolution.iHOffsetFOV;
    info.roi_offset_y = resolution.iVOffsetFOV;
    info.exposure_us = config.exposure_us;
    info.grab_timeout_ms = config.grab_timeout_ms;
    info.pixel_format = PixelFormat::BGR8;
    return true;
  }

  void LogTimeout(int status) {
    const auto NOW = std::chrono::steady_clock::now();
    if (!timeout_active || NOW - last_timeout_log >= std::chrono::seconds(1)) {
      MV_LOG_WARN("HAL.Camera.MV", "CameraGetImageBuffer timeout, status={}", status);
      last_timeout_log = NOW;
    }
    timeout_active = true;
  }

  void LogRecovery() {
    if (timeout_active)
      MV_LOG_INFO("HAL.Camera.MV", "image capture recovered after timeout");
    timeout_active = false;
  }
#else
  void Reset(bool) noexcept {
    handle_valid = false;
    streaming = false;
  }
#endif
};

MindVisionDevice::MindVisionDevice() : impl_(std::make_unique<Impl>()) {}

MindVisionDevice::~MindVisionDevice() {
  Close();
}

bool MindVisionDevice::Open(const MindVisionConfig& config) {
#ifndef MV_HAS_MVSDK
  (void)config;
  MV_LOG_WARN("HAL.Camera.MV", "MV_HAS_MVSDK not defined, running in stub mode");
  return false;
#else
  if (impl_->streaming)
    return true;
  impl_->Reset(false);
  impl_->config = config;

  const int SDK_STATUS = InitializeSdkOnce();
  if (SDK_STATUS != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraSdkInit failed, status={}", SDK_STATUS);
    return false;
  }
  if (!impl_->SelectDevice() || !impl_->InitializeHandle() || !impl_->ConfigureOutput() ||
      !impl_->ConfigureResolution() || !impl_->ConfigureExposure() || !impl_->StartStreaming()) {
    impl_->Reset(false);
    return false;
  }

  MV_LOG_INFO("HAL.Camera.MV",
              "opened '{}' sensor={}x{} output={}x{} ROI=({}, {}) BGR8 exposure={}us timeout={}ms",
              impl_->info.device_name, impl_->info.sensor_width, impl_->info.sensor_height,
              impl_->info.output_width, impl_->info.output_height, impl_->info.roi_offset_x,
              impl_->info.roi_offset_y, impl_->info.exposure_us, impl_->info.grab_timeout_ms);
  return true;
#endif
}

void MindVisionDevice::Close() noexcept {
  impl_->Reset(true);
}

GrabStatus MindVisionDevice::Grab(CameraFrame& frame) {
#ifndef MV_HAS_MVSDK
  (void)frame;
  return GrabStatus::FATAL;
#else
  if (!impl_->streaming) {
    MV_LOG_WARN("HAL.Camera.MV", "Grab called on closed camera");
    return GrabStatus::DISCONNECTED;
  }

  cv::Mat image(impl_->config.height, impl_->config.width, CV_8UC3);
  tSdkFrameHead frame_head{};
  BYTE* raw_buffer = nullptr;
  const int GET_STATUS = CameraGetImageBuffer(impl_->camera_handle, &frame_head, &raw_buffer,
                                              impl_->config.grab_timeout_ms);
  if (GET_STATUS != CAMERA_STATUS_SUCCESS) {
    if (GET_STATUS == CAMERA_STATUS_TIME_OUT) {
      impl_->LogTimeout(GET_STATUS);
      return GrabStatus::TIMEOUT;
    }
    MV_LOG_ERROR("HAL.Camera.MV", "CameraGetImageBuffer failed, status={}", GET_STATUS);
    const auto RESULT =
        IsDisconnectedStatus(GET_STATUS) ? GrabStatus::DISCONNECTED : GrabStatus::FATAL;
    impl_->Reset(false);
    return RESULT;
  }

  impl_->LogRecovery();
  RawBufferLease raw_buffer_lease(impl_->camera_handle, raw_buffer);
  GrabStatus frame_status = GrabStatus::OK;
  if (frame_head.iWidth != impl_->config.width || frame_head.iHeight != impl_->config.height) {
    MV_LOG_WARN("HAL.Camera.MV", "invalid raw frame size: {}x{}, expected {}x{}", frame_head.iWidth,
                frame_head.iHeight, impl_->config.width, impl_->config.height);
    frame_status = GrabStatus::INVALID_FRAME;
  } else {
    const int PROCESS_STATUS =
        CameraImageProcess(impl_->camera_handle, raw_buffer, image.data, &frame_head);
    if (PROCESS_STATUS != CAMERA_STATUS_SUCCESS) {
      MV_LOG_WARN("HAL.Camera.MV", "CameraImageProcess failed, status={}", PROCESS_STATUS);
      frame_status = GrabStatus::INVALID_FRAME;
    }
  }

  const int RELEASE_STATUS = raw_buffer_lease.Release();
  if (RELEASE_STATUS != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraReleaseImageBuffer failed, status={}", RELEASE_STATUS);
    impl_->Reset(false);
    return GrabStatus::FATAL;
  }
  if (frame_status != GrabStatus::OK)
    return frame_status;
  if (frame_head.iWidth != image.cols || frame_head.iHeight != image.rows ||
      image.type() != CV_8UC3) {
    MV_LOG_WARN("HAL.Camera.MV", "invalid processed frame: {}x{} type={}", frame_head.iWidth,
                frame_head.iHeight, image.type());
    return GrabStatus::INVALID_FRAME;
  }

  frame.image = std::move(image);
  frame.receive_steady_time = std::chrono::steady_clock::now();
  frame.sequence = impl_->sequence++;
  return GrabStatus::OK;
#endif
}

CameraInfo MindVisionDevice::Info() const {
  return impl_->info;
}

bool MindVisionDevice::IsOpen() const noexcept {
  return impl_->streaming;
}

}  // namespace mv::hal::detail
