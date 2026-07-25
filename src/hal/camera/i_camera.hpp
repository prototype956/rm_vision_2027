#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace mv::hal {

enum class PixelFormat : uint8_t { UNKNOWN = 0, BGR8, MONO8 };

enum class GrabStatus : uint8_t { OK = 0, TIMEOUT, DISCONNECTED, INVALID_FRAME, FATAL };

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

struct CameraFrame {
  cv::Mat image;
  std::chrono::steady_clock::time_point timestamp{};
  uint64_t sequence{0};
};

struct CameraInfo {
  std::string device_name;
  int sensor_width{0};
  int sensor_height{0};
  int output_width{0};
  int output_height{0};
  int roi_offset_x{0};
  int roi_offset_y{0};
  int exposure_us{0};
  int grab_timeout_ms{0};
  PixelFormat pixel_format{PixelFormat::UNKNOWN};
};

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

  virtual bool Open(const YAML::Node& config) = 0;

  virtual void Close() = 0;

  virtual GrabStatus Grab(CameraFrame& frame) = 0;

  [[nodiscard]] virtual CameraInfo Info() const = 0;

  [[nodiscard]] virtual bool IsOpen() const = 0;
};

}  // namespace mv::hal
