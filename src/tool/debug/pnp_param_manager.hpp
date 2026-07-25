/**
 * @file pnp_param_manager.hpp
 * @brief PnpSolver 参数管理器（Foxglove 热调 + YAML 注入）
 */
#pragma once

#include "tool/foxglove/foxglove_sink.hpp"

#include <atomic>
#include <mutex>

#include <yaml-cpp/yaml.h>

namespace mv::tool {

class PnpParamManager {
 public:
  struct Params {
    float small_half_w{0.0675F};
    float big_half_w{0.115F};
    float half_h{0.0275F};
    double t_camera_to_gimbal_x{0.0};
    double t_camera_to_gimbal_y{0.0};
    double t_camera_to_gimbal_z{0.0};
  };

  PnpParamManager() = default;
  ~PnpParamManager() = default;

  PnpParamManager(const PnpParamManager&) = delete;
  PnpParamManager& operator=(const PnpParamManager&) = delete;
  PnpParamManager(PnpParamManager&&) = delete;
  PnpParamManager& operator=(PnpParamManager&&) = delete;

  void LoadFromYaml(const YAML::Node& root) noexcept;

  [[nodiscard]] Params GetParams() const noexcept;

  void SetParams(const Params& params, bool request_reinit = true) noexcept;

  [[nodiscard]] bool ConsumeReinit() noexcept;

  [[nodiscard]] bool HandleParameter(FoxgloveSink& sink, const std::string& name) noexcept;

  void PushToFoxglove(FoxgloveSink& sink) const noexcept;

  void InjectParamsToYaml(YAML::Node& root) const;

 private:
  mutable std::mutex mutex_;
  Params params_;
  std::atomic<bool> reinit_required_{false};
};

}  // namespace mv::tool
