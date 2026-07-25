#include "opencv_camera.hpp"

#include "../../core/config.hpp"
#include "../../core/logger.hpp"

#include <chrono>

#include <opencv2/videoio.hpp>

namespace mv::hal {

// ============================================================================
// Impl 定义
// ============================================================================

struct OpenCvCamera::Impl {
  cv::VideoCapture cap;
  bool is_open{false};
  CameraInfo info;
  uint64_t sequence{0};
};

// ============================================================================
// 构造 / 析构 / 移动
// ============================================================================

OpenCvCamera::OpenCvCamera() : impl_(std::make_unique<Impl>()) {}

OpenCvCamera::~OpenCvCamera() {
  Close();
}

OpenCvCamera::OpenCvCamera(OpenCvCamera&&) noexcept = default;
OpenCvCamera& OpenCvCamera::operator=(OpenCvCamera&&) noexcept = default;

// ============================================================================
// ICamera 接口实现
// ============================================================================

bool OpenCvCamera::Open(const YAML::Node& config) {
  if (impl_->is_open) {
    return true;
  }

  try {
    ConfigLoader::RejectUnknownKeys(config, {"schema_version", "source", "output", "fps"},
                                    "OpenCV camera config");
    if (ConfigLoader::Require<int>(config, "schema_version", "OpenCV camera config") != 1) {
      throw ConfigError("OpenCV camera config schema_version must be 1");
    }
    ConfigLoader::RequireMap(config["output"], "OpenCV camera config.output");
    ConfigLoader::RejectUnknownKeys(config["output"], {"width", "height", "pixel_format"},
                                    "OpenCV camera config.output");
    impl_->info.output_width =
        ConfigLoader::Require<int>(config["output"], "width", "OpenCV camera config.output");
    impl_->info.output_height =
        ConfigLoader::Require<int>(config["output"], "height", "OpenCV camera config.output");
    const auto pixel_format = ConfigLoader::Require<std::string>(
        config["output"], "pixel_format", "OpenCV camera config.output");
    if (impl_->info.output_width <= 0 || impl_->info.output_height <= 0 ||
        pixel_format != "bgr8") {
      throw ConfigError("OpenCV camera requires positive output size and pixel_format=bgr8");
    }
    impl_->info.pixel_format = PixelFormat::BGR8;
  } catch (const std::exception& error) {
    MV_LOG_ERROR("HAL.Camera.OpenCV", "invalid config: {}", error.what());
    return false;
  }

  // source 字段：先尝试解析为 int（设备索引），否则当字符串（路径/URL）
  const YAML::Node SRC_NODE = config["source"];

  if (SRC_NODE && SRC_NODE.IsScalar()) {
    try {
      const int DEVICE_IDX = SRC_NODE.as<int>();
      if (!impl_->cap.open(DEVICE_IDX, cv::CAP_V4L2)) {
        MV_LOG_ERROR("HAL.Camera.OpenCV", "failed to open device index {}", DEVICE_IDX);
        return false;
      }
      impl_->info.device_name = "opencv:" + std::to_string(DEVICE_IDX);
      MV_LOG_INFO("HAL.Camera.OpenCV", "opened device index {}", DEVICE_IDX);
    } catch (const YAML::BadConversion&) {
      const auto DEVICE_PATH = SRC_NODE.as<std::string>();
      if (!impl_->cap.open(DEVICE_PATH)) {
        MV_LOG_ERROR("HAL.Camera.OpenCV", "failed to open '{}'", DEVICE_PATH);
        return false;
      }
      impl_->info.device_name = DEVICE_PATH;
      MV_LOG_INFO("HAL.Camera.OpenCV", "opened '{}'", DEVICE_PATH);
    }
  } else {
    MV_LOG_ERROR("HAL.Camera.OpenCV", "missing or invalid 'source' field in config");
    return false;
  }

  // 设置分辨率和帧率（仅作 hint，硬件可能调整到最近支持的值）
  impl_->cap.set(cv::CAP_PROP_FRAME_WIDTH, impl_->info.output_width);
  impl_->cap.set(cv::CAP_PROP_FRAME_HEIGHT, impl_->info.output_height);
  if (const int TARGET_FPS = config["fps"].as<int>(0); TARGET_FPS > 0) {
    impl_->cap.set(cv::CAP_PROP_FPS, TARGET_FPS);
  }

  // 减少 V4L2 帧缓冲数，获取最新帧（原因见文件头注释）
  impl_->cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

  const int actual_width = static_cast<int>(impl_->cap.get(cv::CAP_PROP_FRAME_WIDTH));
  const int actual_height = static_cast<int>(impl_->cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  impl_->info.sensor_width = actual_width;
  impl_->info.sensor_height = actual_height;
  if (actual_width != impl_->info.output_width || actual_height != impl_->info.output_height) {
    MV_LOG_ERROR("HAL.Camera.OpenCV", "requested {}x{} but backend selected {}x{}",
                 impl_->info.output_width, impl_->info.output_height, actual_width, actual_height);
    impl_->cap.release();
    return false;
  }

  impl_->sequence = 0;
  impl_->is_open = true;
  return true;
}

void OpenCvCamera::Close() {
  if (!impl_->is_open) {
    return;
  }
  impl_->cap.release();
  impl_->is_open = false;
  MV_LOG_INFO("HAL.Camera.OpenCV", "closed");
}

GrabStatus OpenCvCamera::Grab(CameraFrame& frame) {
  if (!impl_->is_open) {
    MV_LOG_WARN("HAL.Camera.OpenCV", "Grab called on closed camera");
    return GrabStatus::DISCONNECTED;
  }

  // cv::VideoCapture::read() 是阻塞调用，超时由驱动决定
  cv::Mat image;
  if (!impl_->cap.read(image)) {
    // 视频文件播放完毕或设备断开
    MV_LOG_WARN("HAL.Camera.OpenCV", "cap.read() returned false (EOF or device error)");
    impl_->is_open = false;
    return GrabStatus::DISCONNECTED;
  }

  if (image.empty() || image.cols != impl_->info.output_width ||
      image.rows != impl_->info.output_height || image.type() != CV_8UC3) {
    return GrabStatus::INVALID_FRAME;
  }
  frame.image = std::move(image);
  frame.timestamp = std::chrono::steady_clock::now();
  frame.sequence = impl_->sequence++;
  return GrabStatus::OK;
}

CameraInfo OpenCvCamera::Info() const {
  return impl_->info;
}

bool OpenCvCamera::IsOpen() const {
  return impl_->is_open;
}

}  // namespace mv::hal
