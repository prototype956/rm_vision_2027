/**
 * @file shooter_param_manager.hpp
 * @brief RmShooter 参数管理器（Foxglove 热调 + YAML 写回）
 */
#pragma once

#include "tool/foxglove/foxglove_sink.hpp"

#include <atomic>
#include <mutex>

#include <yaml-cpp/yaml.h>

namespace mv::tool {

class ShooterParamManager {
 public:
  struct Params {
    bool enable_ballistic_compensation{false};
    float bullet_speed{15.0F};
    float gravity{9.8F};
  };

  ShooterParamManager() = default;
  ~ShooterParamManager() = default;

  ShooterParamManager(const ShooterParamManager&) = delete;
  ShooterParamManager& operator=(const ShooterParamManager&) = delete;
  ShooterParamManager(ShooterParamManager&&) = delete;
  ShooterParamManager& operator=(ShooterParamManager&&) = delete;

  void LoadFromYaml(const YAML::Node& root) noexcept;

  [[nodiscard]] Params GetParams() const noexcept;

  void SetParams(const Params& params, bool request_reinit = true) noexcept;

  void SetBulletSpeed(float bullet_speed, bool request_reinit = false) noexcept;

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
