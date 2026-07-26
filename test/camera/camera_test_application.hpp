#pragma once

#include "hal/camera/i_camera.hpp"

#include <filesystem>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace mv::test {

/**
 * @brief 相机测试的运行参数。
 *
 * 参数由 test/camera_test.yaml 提供，并在创建 CameraTestApplication 前完成校验。
 */
struct CameraTestSettings {
  int duration_sec{0};                ///< 长时稳定性测试总时长。
  int warmup_sec{0};                  ///< 建立基准帧率的预热时长。
  bool preview{false};                ///< 是否显示带实时指标的预览窗口。
  int report_interval_sec{0};         ///< CSV 和日志指标的输出周期。
  int save_sample_interval_sec{0};    ///< 样本帧保存周期，0 表示不保存。
  int restart_cycles{0};              ///< 重复启停次数，非 0 时不执行长时测试。
  int frames_per_restart_cycle{100};  ///< 每次启停周期内需要成功抓取的帧数。
  std::filesystem::path output_dir;   ///< CSV、JSON 和样本图像的输出目录。
};

/**
 * @brief 执行 MindVision 相机的重复启停或长时稳定性验收。
 *
 * 类持有相机接口及两类配置。Run() 根据 restart_cycles 选择测试模式，并将测试
 * 指标和最终 PASS/FAIL 结果写入 output_dir。
 */
class CameraTestApplication final {
 public:
  /**
   * @brief 创建相机测试应用。
   *
   * @param camera 待测试的相机实现，所有权转移给本对象。
   * @param camera_config 传递给 ICamera::Open() 的相机配置。
   * @param settings 已完成校验的测试参数。
   */
  CameraTestApplication(std::unique_ptr<hal::ICamera> camera, YAML::Node camera_config,
                        CameraTestSettings settings);
  ~CameraTestApplication();

  /**
   * @brief 按配置执行测试并生成测试报告。
   *
   * @return 可直接作为进程退出状态使用的结果码，具体含义见 RunCameraTest()。
   */
  int Run();

 private:
  std::unique_ptr<hal::ICamera> camera_;  ///< 被测相机实例。
  YAML::Node camera_config_;              ///< 每次 Open() 使用的相机配置。
  CameraTestSettings settings_;           ///< 测试模式、时长及报告参数。
};

}  // namespace mv::test
