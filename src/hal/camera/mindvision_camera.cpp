/**
 * @file mindvision_camera.cpp
 * @brief MindVision 工业相机 Pimpl 实现
 *
 * 此文件是唯一需要 include CameraApi.h 的地方。
 * 通过 CMake 的 target_compile_definitions 在有 SDK 时定义 MV_HAS_MVSDK，
 * 无 SDK 时（CI / 开发机）编译为"始终返回失败"的桩实现，
 * 保证整个项目在无硬件环境下仍可编译通过。
 *
 * 【帧获取流程（有 SDK 时）】
 *   CameraGetImageBuffer()  → 阻塞等待 DMA 帧完成
 *   CameraImageProcess()    → SDK 内部去马赛克 / 白平衡
 *   cvCreateImageHeader()   → 包装为 IplImage（SDK 遗留接口）
 *   cv::cvarrToMat()        → 转为 cv::Mat（深拷贝，释放 SDK buffer）
 *   CameraReleaseImageBuffer() → 归还 DMA buffer，允许 SDK 继续采集
 *
 * 【为什么深拷贝而不是零拷贝？】
 *   SDK 的 DMA buffer 数量有限（通常 2-3 个），
 *   如果上层处理太慢不归还，SDK 会丢帧并报 CAMERA_STATUS_BUFFER_FAILED。
 *   深拷贝到 cv::Mat 后立即归还 buffer，
 *   把"SDK buffer 占用时间"压缩到最短（几微秒），代价是一次内存拷贝。
 *   对于 1280x800 BGR 图像约 3MB，现代 CPU 拷贝耗时 ~0.5ms，可接受。
 */

#include "mindvision_camera.hpp"

#include <string>
#include <vector>

#include <opencv2/core/core_c.h>
#include <opencv2/imgproc.hpp>

#ifdef MV_HAS_MVSDK
#include <CameraApi.h>
#endif

#include "../../core/logger.hpp"

namespace mv::hal {

// ============================================================================
// Impl 定义（所有 SDK 类型都在这里，不污染头文件）
// ============================================================================

struct MindVisionCamera::Impl {
  bool is_open{false};

  // 图像参数
  int width{1280};
  int height{800};
  int exposure_us{5000};
  int channel{3};

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

  // 从 YAML 读取参数，兼容两种分辨率写法：
  //   1) resolution: "1280x800"
  //   2) resolution: { width: 1280, height: 800 }
  impl_->exposure_us = config["exposure_us"].as<int>(config["exposure"].as<int>(5000));

  impl_->width = 1280;
  impl_->height = 800;
  YAML::Node resolution_node = config["resolution"];
  if (resolution_node) {
    if (resolution_node.IsMap()) {
      impl_->width = resolution_node["width"].as<int>(1280);
      impl_->height = resolution_node["height"].as<int>(800);
    } else if (resolution_node.IsScalar()) {
      auto resolution_text = resolution_node.as<std::string>();
      std::size_t separator_pos = resolution_text.find('x');
      if (separator_pos != std::string::npos) {
        try {
          impl_->width = std::stoi(resolution_text.substr(0, separator_pos));
          impl_->height = std::stoi(resolution_text.substr(separator_pos + 1));
        } catch (const std::exception&) {
          MV_LOG_WARN("HAL.Camera.MV", "invalid resolution '{}', fallback to 1280x800",
                      resolution_text);
        }
      } else {
        MV_LOG_WARN("HAL.Camera.MV", "invalid resolution '{}', fallback to 1280x800",
                    resolution_text);
      }
    }
  }

  // channel 由 ISP 输出格式决定；读取该字段仅为兼容旧配置。
  (void)config["channel"];

  CameraSdkInit(1);

  int camera_count = 1;
  int status = CameraEnumerateDevice(&impl_->dev_info, &camera_count);
  if (camera_count == 0) {
    MV_LOG_ERROR("HAL.Camera.MV", "no device found (CameraEnumerateDevice returned {})", status);
    return false;
  }

  status = CameraInit(&impl_->dev_info, -1, -1, &impl_->h_camera);
  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_ERROR("HAL.Camera.MV", "CameraInit failed, status={}", status);
    return false;
  }

  CameraGetCapability(impl_->h_camera, &impl_->capability);

  // 显式固定 ISP 输出格式，避免 SDK 默认格式与上层 BGR 假设不一致。
  if (impl_->capability.sIspCapacity.bMonoSensor) {
    impl_->channel = 1;
    CameraSetIspOutFormat(impl_->h_camera, CAMERA_MEDIA_TYPE_MONO8);
  } else {
    impl_->channel = 3;
    CameraSetIspOutFormat(impl_->h_camera, CAMERA_MEDIA_TYPE_BGR8);
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
  CameraGetImageResolution(impl_->h_camera, &impl_->resolution);
  impl_->resolution.iIndex = 0xFF;  // 0xFF = 自定义分辨率
  impl_->resolution.iWidthFOV = impl_->width;
  impl_->resolution.iHeightFOV = impl_->height;
  CameraSetImageResolution(impl_->h_camera, &impl_->resolution);

  // 设置曝光（关闭 AE，手动控制）
  CameraSetAeState(impl_->h_camera, FALSE);
  CameraSetExposureTime(impl_->h_camera, static_cast<double>(impl_->exposure_us));

  CameraPlay(impl_->h_camera);

  impl_->is_open = true;
  MV_LOG_INFO("HAL.Camera.MV", "opened ({}x{}, exposure={}us)", impl_->width, impl_->height,
              impl_->exposure_us);
  return true;
#endif
}

void MindVisionCamera::Close() {
#ifdef MV_HAS_MVSDK
  if (!impl_->is_open) {
    return;  // 幂等
  }

  CameraUnInit(impl_->h_camera);

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

bool MindVisionCamera::Grab(cv::Mat& frame) {
#ifndef MV_HAS_MVSDK
  (void)frame;
  return false;
#else
  if (!impl_->is_open) {
    MV_LOG_WARN("HAL.Camera.MV", "Grab called on closed camera");
    return false;
  }

  // 超时设为 1000ms：比正常帧周期长，但不会让调用方卡太久
  int timeout_ms = 1000;
  int status =
      CameraGetImageBuffer(impl_->h_camera, &impl_->frame_head, &impl_->raw_buffer, timeout_ms);

  if (status != CAMERA_STATUS_SUCCESS) {
    MV_LOG_WARN("HAL.Camera.MV", "CameraGetImageBuffer timeout/error, status={}", status);
    return false;
  }

  // SDK 做去马赛克 / 颜色空间转换，结果写入 rgb_buffer
  CameraImageProcess(impl_->h_camera, impl_->raw_buffer, impl_->rgb_buffer.data(),
                     &impl_->frame_head);

  // 用 IplImage 头包装 rgb_buffer（零拷贝）
  if (impl_->ipl_image != nullptr) {
    cvReleaseImageHeader(&impl_->ipl_image);
  }
  impl_->ipl_image = cvCreateImageHeader(
      cvSize(impl_->frame_head.iWidth, impl_->frame_head.iHeight), IPL_DEPTH_8U, impl_->channel);
  cvSetData(impl_->ipl_image, impl_->rgb_buffer.data(), impl_->frame_head.iWidth * impl_->channel);

  // 深拷贝到 cv::Mat，之后立即归还 DMA buffer
  // 深拷贝原因见文件头注释：尽快释放 SDK buffer 防止丢帧
  frame = cv::cvarrToMat(impl_->ipl_image, true);

  CameraReleaseImageBuffer(impl_->h_camera, impl_->raw_buffer);
  impl_->raw_buffer = nullptr;

  return true;
#endif
}

bool MindVisionCamera::IsOpen() const {
  return impl_->is_open;
}

}  // namespace mv::hal
