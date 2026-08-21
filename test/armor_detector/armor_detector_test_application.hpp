#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "tool/foxglove/foxglove_config.hpp"

#include <memory>

#include <filesystem>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace mv::test {

/**
 * @brief 装甲检测实机测试的运行参数。
 *
 * 参数由 test/armor_detector_test.yaml 提供，并在创建
 * ArmorDetectorTestApplication 前完成校验。
 */
struct ArmorDetectorTestSettings {
  int duration_sec{0};  ///< 测试总时长。
  int warmup_sec{0};    ///< 建立循环帧率基线且不统计检测延迟的预热时长。
  bool preview{false};  ///< 是否显示带检测结果和实时指标的预览窗口。
  int report_interval_sec{0};        ///< CSV 和日志指标的输出周期。
  int save_sample_interval_sec{0};   ///< 检测叠加样本保存周期，0 表示不保存。
  std::filesystem::path output_dir;  ///< CSV、JSON 和样本图像的输出目录。
};

/**
 * @brief 执行 MindVision 相机与 YOLO 0526 检测器的长时实机验收。
 *
 * 测试对每个有效相机帧同步执行检测，统计抓帧质量、检测稳定性、分阶段延迟、
 * 循环帧率和资源增长，并将最终 PASS/FAIL 写入汇总 JSON。
 */
class ArmorDetectorTestApplication final {
 public:
  /**
   * @brief 创建装甲检测实机测试应用。
   *
   * @param camera 待测试的相机实现，所有权转移给本对象。
   * @param detector 已完成初始化的检测器，所有权转移给本对象。
   * @param camera_config 传递给 ICamera::Open() 的相机配置。
   * @param settings 已完成校验的测试参数。
   * @param foxglove_config 可选调试发布配置；缺省时不启动 Foxglove。
   */
  ArmorDetectorTestApplication(std::unique_ptr<hal::ICamera> camera,
                               std::unique_ptr<modules::YoloArmorDetector> detector,
                               const YAML::Node& camera_config, ArmorDetectorTestSettings settings,
                               std::optional<tool::foxglove::Config> foxglove_config);
  ~ArmorDetectorTestApplication();

  /**
   * @brief 运行实机测试并生成测试报告。
   *
   * @return 0 表示通过，3 表示相机打开失败，4 表示相机格式错误，5 表示验收失败。
   */
  int Run();

 private:
  std::unique_ptr<hal::ICamera> camera_;                   ///< 被测相机实例。
  std::unique_ptr<modules::YoloArmorDetector> detector_;   ///< 被测检测器实例。
  YAML::Node camera_config_;                               ///< Open() 使用的相机配置。
  ArmorDetectorTestSettings settings_;                     ///< 时长及报告参数。
  std::optional<tool::foxglove::Config> foxglove_config_;  ///< 可选 Foxglove 调试配置。
};

}  // namespace mv::test
