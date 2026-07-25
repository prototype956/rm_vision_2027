#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

// 前置声明
namespace foxglove {
class WebSocketServer;
}

namespace uart {
struct Receive_Data;
struct Write_Data;
}  // namespace uart

namespace predictor {
struct Target;
}

namespace basic_armor {
struct Armor_Data;
}

class FoxglovePublisher {
 public:
  explicit FoxglovePublisher(const std::string& config_file);
  ~FoxglovePublisher();

  // 启动/停止服务
  void start();
  void stop();

  // === MCAP 录制 ===
  // 开始录制到指定文件
  void startRecording(const std::string& file_path);
  // 停止录制
  void stopRecording();

  // === 数据发布 ===
  // 发布图像（支持原始图、处理图等，通过 topic 区分）
  // topic 示例: "camera/raw", "camera/bin", "camera/annotated"
  void publishImage(const cv::Mat& image, const std::string& topic);

  // 发布装甲板检测信息
  void publishArmorDetection(const std::vector<basic_armor::Armor_Data>& armors);

  // 发布预测跟踪信息（目标位置、预测点等）
  void publishTracking(const predictor::Target& target);

  // 发布串口通信数据（接收和发送）
  // 改为接收 JSON 格式避免头文件依赖
  void publishSerialData(const nlohmann::json& serial_data);

  // 发布 TF 坐标变换（例如：云台相对于底盘，相机相对于云台）
  void publishTransform(const std::string& parent_frame, const std::string& child_frame,
                        const Eigen::Matrix4d& transform, int64_t timestamp_ns);

  // 发布性能指标 (FPS, Latency, etc)
  void publishPerformanceMetrics(const nlohmann::json& metrics);

  // === 反向调参 ===
  // 注册参数回调。当 Foxglove 端修改参数时调用。
  // callback: (param_name, new_value) -> void
  using ParameterCallback = std::function<void(const std::string&, const nlohmann::json&)>;
  void setParameterCallback(ParameterCallback cb);

  // 发送当前参数状态给 Foxglove (初始化或更新时调用)
  void updateParameters(const nlohmann::json& params);

  // 查询参数值 (应用层主动调用)
  nlohmann::json getParameterValue(const std::string& name);

 private:
  struct Impl;
  std::unique_ptr<Impl> pImpl_;

  // 如果需要保留配置路径
  std::string config_file_;
};
