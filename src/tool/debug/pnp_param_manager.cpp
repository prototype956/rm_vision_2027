/**
 * @file pnp_param_manager.cpp
 * @brief PnpSolver 参数管理器实现
 */
#include "tool/debug/pnp_param_manager.hpp"

#include <spdlog/spdlog.h>

namespace mv::tool {

void PnpParamManager::LoadFromYaml(const YAML::Node& root) noexcept {
  Params params = GetParams();

  if (root && root["armor"]) {
    const auto& armor = root["armor"];
    params.small_half_w = armor["small_half_w"].as<float>(params.small_half_w);
    params.big_half_w = armor["big_half_w"].as<float>(params.big_half_w);
    params.half_h = armor["half_h"].as<float>(params.half_h);
  }

  if (root && root["calibration"] && root["calibration"]["t_camera_to_gimbal"]) {
    const auto& translation_node = root["calibration"]["t_camera_to_gimbal"];
    try {
      const auto TRANSLATION_VALUES = translation_node.as<std::vector<double>>();
      if (TRANSLATION_VALUES.size() == 3) {
        params.t_camera_to_gimbal_x = TRANSLATION_VALUES[0];
        params.t_camera_to_gimbal_y = TRANSLATION_VALUES[1];
        params.t_camera_to_gimbal_z = TRANSLATION_VALUES[2];
      }
    } catch (...) {
    }
  }

  SetParams(params, false);
}

PnpParamManager::Params PnpParamManager::GetParams() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return params_;
}

void PnpParamManager::SetParams(const Params& params, bool request_reinit) noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
  }
  if (request_reinit) {
    reinit_required_.store(true, std::memory_order_release);
  }
}

bool PnpParamManager::ConsumeReinit() noexcept {
  return reinit_required_.exchange(false, std::memory_order_acq_rel);
}

bool PnpParamManager::HandleParameter(FoxgloveSink& sink, const std::string& name) noexcept {
  const auto VALUE = sink.GetParameter(name);
  if (VALUE.is_null() || name.rfind("pnp.", 0) != 0) {
    return false;
  }

  auto params = GetParams();
  if (name == "pnp.small_half_w" && VALUE.is_number()) {
    params.small_half_w = VALUE.get<float>();
  } else if (name == "pnp.big_half_w" && VALUE.is_number()) {
    params.big_half_w = VALUE.get<float>();
  } else if (name == "pnp.half_h" && VALUE.is_number()) {
    params.half_h = VALUE.get<float>();
  } else if (name == "pnp.t_camera_to_gimbal_x" && VALUE.is_number()) {
    params.t_camera_to_gimbal_x = VALUE.get<double>();
  } else if (name == "pnp.t_camera_to_gimbal_y" && VALUE.is_number()) {
    params.t_camera_to_gimbal_y = VALUE.get<double>();
  } else if (name == "pnp.t_camera_to_gimbal_z" && VALUE.is_number()) {
    params.t_camera_to_gimbal_z = VALUE.get<double>();
  } else {
    return false;
  }

  SetParams(params, true);
  spdlog::info("[PnpParamManager] Updated {}: {}", name, VALUE.dump());
  return true;
}

void PnpParamManager::PushToFoxglove(FoxgloveSink& sink) const noexcept {
  const auto PARAMS = GetParams();
  sink.UpdateParameters({{"pnp.small_half_w", PARAMS.small_half_w},
                         {"pnp.big_half_w", PARAMS.big_half_w},
                         {"pnp.half_h", PARAMS.half_h},
                         {"pnp.t_camera_to_gimbal_x", PARAMS.t_camera_to_gimbal_x},
                         {"pnp.t_camera_to_gimbal_y", PARAMS.t_camera_to_gimbal_y},
                         {"pnp.t_camera_to_gimbal_z", PARAMS.t_camera_to_gimbal_z}});
}

void PnpParamManager::InjectParamsToYaml(YAML::Node& root) const {
  const auto PARAMS = GetParams();

  YAML::Node armor = root["armor"];
  armor["small_half_w"] = PARAMS.small_half_w;
  armor["big_half_w"] = PARAMS.big_half_w;
  armor["half_h"] = PARAMS.half_h;

  YAML::Node calibration = root["calibration"];
  calibration["t_camera_to_gimbal"] =
      std::vector<double>{PARAMS.t_camera_to_gimbal_x, PARAMS.t_camera_to_gimbal_y,
                          PARAMS.t_camera_to_gimbal_z};
}

}  // namespace mv::tool
