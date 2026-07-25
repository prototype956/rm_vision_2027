/**
 * @file shooter_param_manager.cpp
 * @brief RmShooter 参数管理器实现
 */
#include "tool/debug/shooter_param_manager.hpp"

#include <spdlog/spdlog.h>

namespace mv::tool {

void ShooterParamManager::LoadFromYaml(const YAML::Node& root) noexcept {
  Params params = GetParams();
  if (root && root["auto_aim"] && root["auto_aim"]["shooter"]) {
    const auto& shooter = root["auto_aim"]["shooter"];
    params.enable_ballistic_compensation =
        shooter["enable_ballistic_compensation"].as<bool>(params.enable_ballistic_compensation);
    params.bullet_speed = shooter["bullet_speed"].as<float>(params.bullet_speed);
    params.gravity = shooter["gravity"].as<float>(params.gravity);
  }
  SetParams(params, false);
}

ShooterParamManager::Params ShooterParamManager::GetParams() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return params_;
}

void ShooterParamManager::SetParams(const Params& params, bool request_reinit) noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
  }
  if (request_reinit) {
    reinit_required_.store(true, std::memory_order_release);
  }
}

void ShooterParamManager::SetBulletSpeed(float bullet_speed, bool request_reinit) noexcept {
  auto params = GetParams();
  params.bullet_speed = bullet_speed;
  SetParams(params, request_reinit);
}

bool ShooterParamManager::ConsumeReinit() noexcept {
  return reinit_required_.exchange(false, std::memory_order_acq_rel);
}

bool ShooterParamManager::HandleParameter(FoxgloveSink& sink, const std::string& name) noexcept {
  const auto VALUE = sink.GetParameter(name);
  if (VALUE.is_null() || name.rfind("shooter.", 0) != 0) {
    return false;
  }

  auto params = GetParams();
  if (name == "shooter.enable_ballistic_compensation" && VALUE.is_boolean()) {
    params.enable_ballistic_compensation = VALUE.get<bool>();
  } else if (name == "shooter.bullet_speed" && VALUE.is_number()) {
    params.bullet_speed = VALUE.get<float>();
  } else if (name == "shooter.gravity" && VALUE.is_number()) {
    params.gravity = VALUE.get<float>();
  } else {
    return false;
  }

  SetParams(params, true);
  spdlog::info("[ShooterParamManager] Updated {}: {}", name, VALUE.dump());
  return true;
}

void ShooterParamManager::PushToFoxglove(FoxgloveSink& sink) const noexcept {
  const auto PARAMS = GetParams();
  sink.UpdateParameters(
      {{"shooter.enable_ballistic_compensation", PARAMS.enable_ballistic_compensation},
       {"shooter.bullet_speed", PARAMS.bullet_speed},
       {"shooter.gravity", PARAMS.gravity}});
}

void ShooterParamManager::InjectParamsToYaml(YAML::Node& root) const {
  const auto PARAMS = GetParams();
  YAML::Node shooter = root["auto_aim"]["shooter"];
  shooter["enable_ballistic_compensation"] = PARAMS.enable_ballistic_compensation;
  shooter["bullet_speed"] = PARAMS.bullet_speed;
  shooter["gravity"] = PARAMS.gravity;
}

}  // namespace mv::tool
