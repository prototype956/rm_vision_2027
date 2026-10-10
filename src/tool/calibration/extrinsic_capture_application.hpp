#pragma once

#include "frame/frame_types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace mv::tool::calibration {

/**
 * @brief 仿真外参采集会话的配置。
 */
struct ExtrinsicCaptureSettings {
  int expected_width{1280};          ///< 期望的 Talos 图像宽度，单位为像素。
  int expected_height{720};          ///< 期望的 Talos 图像高度，单位为像素。
  int board_columns{10};             ///< 棋盘横向内角点数量。
  int board_rows{7};                 ///< 棋盘纵向内角点数量。
  double square_size_mm{15.0};       ///< 棋盘方格间距，单位为毫米。
  int solve_min_samples{3};          ///< 手眼求解所需的最少有效样本数。
  std::filesystem::path output_dir;  ///< 会话输出根目录。
};

/**
 * @brief 解析并校验仿真外参采集配置。
 *
 * @param root 已由 ConfigLoader 加载的配置节点。
 * @param project_root 项目根目录，用于解析相对输出路径。
 * @return 可直接用于采集的配置。
 * @throws ConfigError 配置缺失、类型错误或值域非法。
 */
[[nodiscard]] ExtrinsicCaptureSettings ParseExtrinsicCaptureSettings(
    const YAML::Node& root, const std::filesystem::path& project_root);

/**
 * @brief 从 Talos 原始图像采集外参标定样本，并保存同帧空间真值。
 */
class ExtrinsicCaptureApplication final {
 public:
  /** @brief 保存采集设置；Run() 在调用线程处理窗口事件与磁盘写入。 */
  explicit ExtrinsicCaptureApplication(ExtrinsicCaptureSettings settings,
                                       const YAML::Node& talos_config);

  /**
   * @brief 运行 Space/S 保存、U 排除、Q/Esc 结束的交互采集循环。
   * @return 0 表示正常结束，非零值表示相机或采集失败。
   */
  int Run();

 private:
  struct Sample {
    std::size_t id{0};
    bool active{true};
    std::filesystem::path image_path;  ///< 相对于会话目录的图像路径。
    std::uint64_t frame_sequence{0};
    std::uint64_t capture_timestamp_ns{0};
    std::uint64_t simulation_round_id{0};  ///< 防止离线求解混合 R 重置前后的观测。
    frame::CameraModel camera;
    geometry::RigidTransform world_t_gimbal;  ///< 求解输入，gimbal 到 world，平移为米。
    geometry::RigidTransform gimbal_t_camera_optical;  ///< 仅用于最终评估的相机到云台真值。
  };

  /** @brief 原子替换清单；失败时保留此前清单，调用方负责回滚图像移动。 */
  void WriteSession(const std::string& status, const std::string& error = "") const;

  ExtrinsicCaptureSettings settings_;
  YAML::Node talos_config_;
  std::filesystem::path session_dir_;
  std::filesystem::path session_path_;
  std::string device_name_;
  std::vector<Sample> samples_;
  std::size_t next_sample_id_{1};
};

}  // namespace mv::tool::calibration
