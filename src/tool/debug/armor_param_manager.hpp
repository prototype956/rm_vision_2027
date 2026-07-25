/**
 * @file armor_param_manager.hpp
 * @brief 装甲检测器参数管理器（热调 + 配置写回）
 *
 * 【设计意图】
 *   将 BasicArmorDetector 的参数热调集中管理，支持：
 *   - Foxglove 参数面板双向同步
 *   - 参数热应用（无需重启算法）
 *   - 参数保存到 debug_override.yaml
 *   - YAML 注入（影响后续 Init() 调用）
 *
 * 【线程安全】
 *   params_ 由 mtx_ 保护。Foxglove 回调写入 params_ 后自动应用到 detector。
 *   主循环可安全读取 GetParams()。
 *
 * 【整数流程】
 * @code
 *   // 初始化
 *   ArmorDetectorParamManager pm;
 *   pm.Register(sink);               // 注册 Foxglove 参数回调
 *   pm.PushToFoxglove(sink);         // 推送初始参数到面板
 *
 *   // 主循环
 *   // 用户在 Foxglove 修改参数时自动应用到 detector
 *   // （回调中 detector.SetParams() 立即执行）
 *
 *   // 保存
 *   if (user_pressed_save) {
 *       pm.SaveToFile(CONFIG_FILE_PATH "/debug/debug_override.yaml",
 *                     CONFIG_FILE_PATH "/vision.yaml");
 *   }
 * @endcode
 */
#pragma once

#include "modules/armor_detector/basic_armor_detector.hpp"
#include "tool/foxglove/foxglove_sink.hpp"

#include <memory>
#include <mutex>
#include <string>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace mv::tool {

/**
 * @brief 装甲检测器参数管理器
 */
class ArmorDetectorParamManager {
 public:
  explicit ArmorDetectorParamManager(mv::modules::BasicArmorDetector* detector) noexcept;
  ~ArmorDetectorParamManager() = default;

  ArmorDetectorParamManager(const ArmorDetectorParamManager&) = delete;
  ArmorDetectorParamManager& operator=(const ArmorDetectorParamManager&) = delete;
  ArmorDetectorParamManager(ArmorDetectorParamManager&&) = delete;
  ArmorDetectorParamManager& operator=(ArmorDetectorParamManager&&) = delete;

  /**
   * @brief 获取当前参数的线程安全快照
   */
  [[nodiscard]] mv::modules::BasicArmorDetector::Params GetParams() const noexcept;

  /**
   * @brief 设置参数并应用到检测器
   */
  void SetParams(const mv::modules::BasicArmorDetector::Params& params) noexcept;

  /**
   * @brief 注册 Foxglove 参数回调（WebSocket 线程调用）
   *
   * 用户在 Foxglove 面板修改参数时，自动应用到检测器。
   */
  void Register(FoxgloveSink& sink) noexcept;

  /**
   * @brief 处理单个 Foxglove 参数更新
   * @return true 表示该参数由装甲检测器参数管理器处理
   */
  [[nodiscard]] bool HandleParameter(FoxgloveSink& sink, const std::string& name) noexcept;

  /**
   * @brief 推送当前参数到 Foxglove 参数面板
   */
  void PushToFoxglove(FoxgloveSink& sink) const noexcept;

  /**
    * @brief 将当前参数注入到 YAML 根节点的 detector 子树
    */
  void InjectParamsToYaml(YAML::Node& root) const;

  /**
   * @brief 保存参数到 debug_override.yaml
   *
    * 若目标文件不存在且提供了 fallback_yaml_path，则先从 fallback 文件加载
    * 一份配置，再将 detector 参数覆盖进去。
   *
    * @param yaml_path          目标 YAML 文件路径
    * @param fallback_yaml_path 目标文件不存在时的回退模板文件路径
   * @return 是否成功保存
   */
  [[nodiscard]] bool SaveToFile(const std::string& yaml_path,
                                          const std::string& fallback_yaml_path = "") const noexcept;

 private:
  mv::modules::BasicArmorDetector* detector_;
  mutable std::mutex mtx_;
  mv::modules::BasicArmorDetector::Params params_;

  /**
   * @brief 从参数对象生成 JSON（用于推送和保存）
   */
  [[nodiscard]] static nlohmann::json ParamsToJson(
      const mv::modules::BasicArmorDetector::Params& params) noexcept;

  /**
   * @brief 从 JSON 解析参数对象
   */
  [[nodiscard]] static mv::modules::BasicArmorDetector::Params JsonToParams(
      const nlohmann::json& json) noexcept;
};

}  // namespace mv::tool
