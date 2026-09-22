#pragma once

#include "frame/frame_packet.hpp"
#include "geometry/gimbal_geometry.hpp"
#include "hal/camera/i_camera.hpp"
#include "hal/imu/serial_imu.hpp"

#include <filesystem>
#include <string>

namespace mv::runtime {
/** @brief 负责配置、IMU 时间匹配与同帧数据装配；坐标数学运算委托给 geometry。 */
class RealFrameGeometry final {
 public:
  RealFrameGeometry(const std::filesystem::path& config_root, const hal::CameraInfo& camera);
  /** @brief 时间或姿态不可用时保持平台运动学为空，使预测器重置历史状态。 */
  void Attach(frame::FramePacket& packet);
  [[nodiscard]] const std::string& Status() const noexcept { return status_; }

 private:
  hal::SerialImu imu_;
  frame::CameraModel camera_model_;
  geometry::GimbalExtrinsics extrinsics_;  ///< 纯几何安装模型，不持有硬件或运行状态。
  bool placeholder_{true};
  bool camera_matches_{false};
  std::uint64_t generation_{0};
  std::chrono::steady_clock::time_point last_log_{};
  std::optional<std::chrono::steady_clock::time_point> last_capture_;
  std::string status_;
};
}  // namespace mv::runtime
