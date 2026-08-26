#pragma once

#include <chrono>

#include <yaml-cpp/yaml.h>

namespace mv::runtime {

/** @brief 运行时瞬态错误、控制通道和可选组件的统一降级门限。 */
struct RuntimeErrorPolicy {
  std::chrono::milliseconds camera_no_valid_frame_timeout{2000};
  std::chrono::milliseconds command_sink_unhealthy_timeout{1000};
  int evaluation_disable_after_consecutive_errors{1};
  int debug_window_disable_after_consecutive_errors{1};
};

/**
 * @brief 严格解析运行时错误策略配置。
 * @param root `runtime/error_policy.yaml` 的根节点。
 * @return 已完成类型和值域校验的策略。
 * @throws ConfigError Schema、字段或值域非法。
 */
[[nodiscard]] RuntimeErrorPolicy ParseRuntimeErrorPolicy(const YAML::Node& root);

}  // namespace mv::runtime
