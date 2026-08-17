#pragma once

#include "geometry/rigid_transform.hpp"
#include "hal/gimbal/gimbal_types.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <optional>
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
  /** @brief 与当前图像对应的针孔相机内参和 plumb_bob 畸变参数。 */
  struct Calibration {
    std::uint32_t width{0};              ///< 标定适用的图像宽度，单位为像素。
    std::uint32_t height{0};             ///< 标定适用的图像高度，单位为像素。
    double fx{0.0};                      ///< 水平方向焦距，单位为像素。
    double fy{0.0};                      ///< 垂直方向焦距，单位为像素。
    double cx{0.0};                      ///< 主点横坐标，单位为像素。
    double cy{0.0};                      ///< 主点纵坐标，单位为像素。
    std::array<double, 5> distortion{};  ///< 依次为 k1、k2、p1、p2、k3。
  };

  /** @brief 仿真器在 world 坐标系中给出的单个机器人真值。 */
  struct GroundTruthTarget {
    std::uint64_t id{0};          ///< 本次仿真运行内区分目标的稳定标识。
    std::uint8_t team{0};         ///< 队伍编码：0 为红方，1 为蓝方。
    std::uint8_t armor_label{0};  ///< Talos 协议中的装甲类别编码。
    bool is_outpost{false};       ///< 是否为前哨站等特殊旋转目标。
    geometry::Vector3 position_world{geometry::Vector3::Zero()};  ///< 机器人中心世界位置。
    double yaw{0.0};           ///< 绕 world +Z 轴的航向角，单位为弧度。
    double yaw_velocity{0.0};  ///< 航向角速度，单位为弧度每秒。
  };

  enum class ArmorType : std::uint8_t { SMALL = 0, LARGE = 1 };

  /** @brief 与图像同帧的单块装甲板灯条端点平面真值。 */
  struct GroundTruthArmor {
    std::uint64_t id{0};
    std::uint8_t team{0};
    std::uint8_t label{0};
    ArmorType type{ArmorType::SMALL};
    double width_m{0.0};
    double height_m{0.0};
    geometry::RigidTransform world_t_armor;
    std::array<geometry::Vector3, 4> corners_world{};  ///< TL/TR/BR/BL。
  };

  /**
   * @brief 与图像在同一仿真采集快照中的标定、外参和真值。
   *
   * 仅能和所属 CameraFrame 的 image、capture_timestamp_ns 配套使用，不能跨帧组合。
   */
  struct FrameGeometry {
    Calibration calibration;                  ///< camera_optical 对应的内参与畸变。
    geometry::RigidTransform world_t_gimbal;  ///< gimbal 到 world 的变换。
    geometry::RigidTransform gimbal_t_camera_optical;  ///< camera_optical 到 gimbal 的变换。
    geometry::RigidTransform gimbal_t_muzzle;          ///< muzzle 到 gimbal 的变换。
    std::optional<GimbalActuatorTelemetry> gimbal_actuator;  ///< 与图像同帧的执行器状态。
    std::vector<GroundTruthTarget> targets;  ///< 当前快照中的机器人真值。
    std::vector<GroundTruthArmor> armors;    ///< 当前快照中的单块装甲真值。
  };

  cv::Mat image;                                                ///< OpenCV 图像矩阵。
  std::chrono::steady_clock::time_point receive_steady_time{};  ///< HAL 收帧单调时钟。
  std::optional<std::uint64_t> capture_timestamp_ns;  ///< 数据源采集 Unix epoch 纳秒时间。
  std::optional<FrameGeometry> geometry;  ///< 同一采集快照的空间数据；实机后端通常为空。
  uint64_t sequence{0};                   ///< 本次 Open() 后从 0 递增的帧序号。
  uint64_t source_invalid_frames{0};  ///< 当前数据源自 Open() 以来拒绝的无效帧数。
};

/**
 * @brief 相机成功打开后实际生效的设备与成像参数。
 *
 * 字段由具体后端在 Open() 成功前填充。调用者应使用这里的实际值校验输出，不能
 * 假定驱动一定接受了配置文件中的请求值。
 */
struct CameraInfo {
  std::string device_name;  ///< 后端提供的设备名称或输入源标识。
  int sensor_width{0};      ///< 传感器或后端报告的原始宽度。
  int sensor_height{0};     ///< 传感器或后端报告的原始高度。
  int output_width{0};      ///< 每帧输出宽度。
  int output_height{0};     ///< 每帧输出高度。
  int roi_offset_x{0};      ///< 硬件 ROI 左上角横坐标。
  int roi_offset_y{0};      ///< 硬件 ROI 左上角纵坐标。
  int exposure_us{0};       ///< 曝光时间，单位为微秒。
  int grab_timeout_ms{0};   ///< 单次阻塞取帧超时，单位为毫秒。
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
