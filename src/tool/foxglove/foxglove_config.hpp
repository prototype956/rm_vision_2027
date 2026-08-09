#pragma once

#include <cstdint>
#include <string>

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace mv::tool::foxglove {

/**
 * @brief Foxglove WebSocket 服务监听参数。
 */
struct ServerConfig {
  std::string host{"0.0.0.0"};  ///< 监听地址，0.0.0.0 表示所有网络接口。
  std::uint16_t port{8765};     ///< Foxglove WebSocket 监听端口。
};

/**
 * @brief 调试图像的坐标系、发布频率和 JPEG 参数。
 */
struct ImageConfig {
  std::string frame_id{"camera_optical"};  ///< CompressedImage 使用的坐标系名称。
  double max_fps{20.0};                    ///< 调试流最大发布频率。
  int jpeg_quality{75};                    ///< OpenCV JPEG 质量，范围为 [1, 100]。
};

/**
 * @brief MCAP 调试流录制参数。
 */
struct RecordingConfig {
  bool enabled{false};               ///< 是否在启动时创建 MCAP Writer。
  std::filesystem::path output_dir;  ///< 录制文件输出目录的规范化绝对路径。
};

/**
 * @brief Foxglove 调试模块的完整运行配置。
 */
struct Config {
  bool enabled{true};         ///< 是否启用整个 Foxglove 调试模块。
  ServerConfig server;        ///< 实时 WebSocket 配置。
  ImageConfig image;          ///< 图像编码与限流配置。
  RecordingConfig recording;  ///< MCAP 录制配置。
};

/**
 * @brief 解析并严格校验 Foxglove 调试配置。
 *
 * recording.output_dir 相对于配置文件所在目录解析。
 *
 * @param root 已由 ConfigLoader 加载且包含 schema_version 的 YAML 根节点。
 * @param config_path 配置文件路径，用于解析相对输出目录。
 * @return 完成值域检查和路径规范化的配置。
 * @throws ConfigError 配置缺失、包含未知键或字段值超出允许范围。
 */
[[nodiscard]] Config ParseConfig(const YAML::Node& root, const std::filesystem::path& config_path);

}  // namespace mv::tool::foxglove
