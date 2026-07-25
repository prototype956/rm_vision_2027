/**
 * @file armor_param_manager.cpp
 * @brief 装甲检测器参数管理器实现
 */
#include "tool/debug/armor_param_manager.hpp"

#include <fstream>

#include <filesystem>
#include <spdlog/spdlog.h>

namespace mv::tool {

ArmorDetectorParamManager::ArmorDetectorParamManager(
    mv::modules::BasicArmorDetector* detector) noexcept
    : detector_(detector) {
  if (detector_) {
    params_ = detector_->GetParams();
  }
}

mv::modules::BasicArmorDetector::Params ArmorDetectorParamManager::GetParams() const noexcept {
  std::lock_guard<std::mutex> lock(mtx_);
  return params_;
}

void ArmorDetectorParamManager::SetParams(
    const mv::modules::BasicArmorDetector::Params& params) noexcept {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    params_ = params;
  }
  if (detector_) {
    detector_->SetParams(params);
  }
}

void ArmorDetectorParamManager::Register(FoxgloveSink& sink) noexcept {
  sink.SetParameterCallback([this, &sink](const std::string& name, const nlohmann::json& /*raw*/) {
    (void)HandleParameter(sink, name);
  });
}

bool ArmorDetectorParamManager::HandleParameter(FoxgloveSink& sink,
                                                const std::string& name) noexcept {
  const auto VALUE = sink.GetParameter(name);
  if (VALUE.is_null() || name.rfind("armor.", 0) != 0) {
    return false;
  }

  auto params = this->GetParams();

  if (name == "armor.light_thresh" && VALUE.is_number_integer()) {
    params.light_thresh = VALUE.get<int>();
  } else if (name == "armor.green_thresh" && VALUE.is_number_integer()) {
    params.green_thresh = VALUE.get<int>();
  } else if (name == "armor.white_thresh" && VALUE.is_number_integer()) {
    params.white_thresh = VALUE.get<int>();
  } else if (name == "armor.min_light_ratio" && VALUE.is_number()) {
    params.min_light_ratio = VALUE.get<float>();
  } else if (name == "armor.max_light_ratio" && VALUE.is_number()) {
    params.max_light_ratio = VALUE.get<float>();
  } else if (name == "armor.max_light_angle" && VALUE.is_number()) {
    params.max_light_angle = VALUE.get<float>();
  } else if (name == "armor.min_armor_ratio" && VALUE.is_number()) {
    params.min_armor_ratio = VALUE.get<float>();
  } else if (name == "armor.max_armor_ratio" && VALUE.is_number()) {
    params.max_armor_ratio = VALUE.get<float>();
  } else if (name == "armor.max_angle_diff" && VALUE.is_number()) {
    params.max_angle_diff = VALUE.get<float>();
  } else if (name == "armor.min_area" && VALUE.is_number()) {
    params.min_area = VALUE.get<float>();
  } else {
    return false;
  }

  this->SetParams(params);
  spdlog::info("[ArmorParamManager] Updated {}: {}", name, VALUE.dump());
  return true;
}

void ArmorDetectorParamManager::PushToFoxglove(FoxgloveSink& sink) const noexcept {
  auto params = GetParams();
  auto json_params = ParamsToJson(params);

  // 推送所有参数到 Foxglove
  for (const auto& [key, val] : json_params.items()) {
    nlohmann::json param_update;
    param_update["armor." + key] = val;
    sink.UpdateParameters(param_update);
  }

  spdlog::info("[ArmorParamManager] Pushed parameters to Foxglove");
}

void ArmorDetectorParamManager::InjectParamsToYaml(YAML::Node& root) const {
  auto params = GetParams();

  if (!root["detector"] || !root["detector"].IsMap()) {
    root["detector"] = YAML::Node(YAML::NodeType::Map);
  }

  YAML::Node detector = root["detector"];
  detector["light_thresh"] = params.light_thresh;
  detector["green_thresh"] = params.green_thresh;
  detector["white_thresh"] = params.white_thresh;
  detector["min_light_ratio"] = params.min_light_ratio;
  detector["max_light_ratio"] = params.max_light_ratio;
  detector["max_light_angle"] = params.max_light_angle;
  detector["min_armor_ratio"] = params.min_armor_ratio;
  detector["max_armor_ratio"] = params.max_armor_ratio;
  detector["max_angle_diff"] = params.max_angle_diff;
  detector["min_area"] = params.min_area;
}

bool ArmorDetectorParamManager::SaveToFile(const std::string& yaml_path,
                                           const std::string& fallback_yaml_path) const noexcept {
  try {
    YAML::Node root;

    if (std::filesystem::exists(yaml_path)) {
      root = YAML::LoadFile(yaml_path);
    } else if (!fallback_yaml_path.empty() && std::filesystem::exists(fallback_yaml_path)) {
      root = YAML::LoadFile(fallback_yaml_path);
    }

    InjectParamsToYaml(root);

    std::filesystem::path output_path(yaml_path);
    if (!output_path.parent_path().empty()) {
      std::filesystem::create_directories(output_path.parent_path());
    }

    std::ofstream ofs(yaml_path);
    if (!ofs.is_open()) {
      spdlog::error("[ArmorParamManager] Failed to open {} for writing", yaml_path);
      return false;
    }

    ofs << root;
    spdlog::info("[ArmorParamManager] Saved parameters to {}", yaml_path);
    return true;
  } catch (const std::exception& e) {
    spdlog::error("[ArmorParamManager] Failed to save parameters: {}", e.what());
    return false;
  }
}

nlohmann::json ArmorDetectorParamManager::ParamsToJson(
    const mv::modules::BasicArmorDetector::Params& params) noexcept {
  nlohmann::json json;
  json["light_thresh"] = params.light_thresh;
  json["green_thresh"] = params.green_thresh;
  json["white_thresh"] = params.white_thresh;
  json["min_light_ratio"] = params.min_light_ratio;
  json["max_light_ratio"] = params.max_light_ratio;
  json["max_light_angle"] = params.max_light_angle;
  json["min_armor_ratio"] = params.min_armor_ratio;
  json["max_armor_ratio"] = params.max_armor_ratio;
  json["max_angle_diff"] = params.max_angle_diff;
  json["min_area"] = params.min_area;
  return json;
}

mv::modules::BasicArmorDetector::Params ArmorDetectorParamManager::JsonToParams(
    const nlohmann::json& json) noexcept {
  mv::modules::BasicArmorDetector::Params params;
  if (json.contains("light_thresh") && json["light_thresh"].is_number_integer()) {
    params.light_thresh = json["light_thresh"].get<int>();
  }
  if (json.contains("green_thresh") && json["green_thresh"].is_number_integer()) {
    params.green_thresh = json["green_thresh"].get<int>();
  }
  if (json.contains("white_thresh") && json["white_thresh"].is_number_integer()) {
    params.white_thresh = json["white_thresh"].get<int>();
  }
  if (json.contains("min_light_ratio") && json["min_light_ratio"].is_number()) {
    params.min_light_ratio = json["min_light_ratio"].get<float>();
  }
  if (json.contains("max_light_ratio") && json["max_light_ratio"].is_number()) {
    params.max_light_ratio = json["max_light_ratio"].get<float>();
  }
  if (json.contains("max_light_angle") && json["max_light_angle"].is_number()) {
    params.max_light_angle = json["max_light_angle"].get<float>();
  }
  if (json.contains("min_armor_ratio") && json["min_armor_ratio"].is_number()) {
    params.min_armor_ratio = json["min_armor_ratio"].get<float>();
  }
  if (json.contains("max_armor_ratio") && json["max_armor_ratio"].is_number()) {
    params.max_armor_ratio = json["max_armor_ratio"].get<float>();
  }
  if (json.contains("max_angle_diff") && json["max_angle_diff"].is_number()) {
    params.max_angle_diff = json["max_angle_diff"].get<float>();
  }
  if (json.contains("min_area") && json["min_area"].is_number()) {
    params.min_area = json["min_area"].get<float>();
  }
  return params;
}

}  // namespace mv::tool
