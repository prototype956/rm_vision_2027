/**
 * @file predict_param_manager.cpp
 * @brief PredictParamManager 实现
 */
#include "tool/debug/predict_param_manager.hpp"

#include <cmath>
#include <string>

namespace mv::tool {

// ============================================================================
// JSON 类型转换辅助
// ============================================================================

double PredictParamManager::JsonToDouble(const nlohmann::json& v, double fallback) noexcept {
  if (v.is_number())
    return v.get<double>();
  return fallback;
}

int PredictParamManager::JsonToInt(const nlohmann::json& v, int fallback) noexcept {
  if (v.is_number_integer())
    return v.get<int>();
  if (v.is_number())
    return static_cast<int>(v.get<double>());
  return fallback;
}

bool PredictParamManager::JsonToBool(const nlohmann::json& v, bool fallback) noexcept {
  if (v.is_boolean())
    return v.get<bool>();
  return fallback;
}

// ============================================================================
// Foxglove 参数面板
// ============================================================================

void PredictParamManager::Register(FoxgloveSink& sink) {
  sink.SetParameterCallback([this, &sink](const std::string& name, const nlohmann::json& /*raw*/) {
    (void)HandleParameter(sink, name);
  });
}

bool PredictParamManager::HandleParameter(FoxgloveSink& sink, const std::string& name) {
  const auto val = sink.GetParameter(name);
  if (val.is_null()) {
    return false;
  }

  std::lock_guard<std::mutex> lk(mutex_);

  if (name == "enemy_color" && val.is_string()) {
    state_.enemy_color =
        (val.get<std::string>() == "blue") ? mv::ArmorColor::BLUE : mv::ArmorColor::RED;
    return true;
  }
  if (name == "loop_video") {
    state_.loop_video = JsonToBool(val, state_.loop_video);
    return true;
  }
  if (name == "playback_fps") {
    state_.playback_fps = JsonToDouble(val, state_.playback_fps);
    return true;
  }

  bool ekf_changed = false;
  bool voter_changed = false;
  bool handled = false;

  if (name == "ekf.min_detect_count") {
    state_.ekf_min_detect_count = JsonToInt(val, state_.ekf_min_detect_count);
    ekf_changed = true;
    handled = true;
  } else if (name == "ekf.max_detecting_lost_count") {
    state_.ekf_max_detecting_lost_count = JsonToInt(val, state_.ekf_max_detecting_lost_count);
    ekf_changed = true;
    handled = true;
  } else if (name == "ekf.max_temp_lost_count") {
    state_.ekf_max_temp_lost_count = JsonToInt(val, state_.ekf_max_temp_lost_count);
    ekf_changed = true;
    handled = true;
  } else if (name == "ekf.process_noise_pos") {
    state_.ekf_process_noise_pos = JsonToDouble(val, state_.ekf_process_noise_pos);
    ekf_changed = true;
    handled = true;
  } else if (name == "ekf.process_noise_ang") {
    state_.ekf_process_noise_ang = JsonToDouble(val, state_.ekf_process_noise_ang);
    ekf_changed = true;
    handled = true;
  } else if (name == "ekf.yaw_offset_deg") {
    state_.ekf_yaw_offset_deg = JsonToDouble(val, state_.ekf_yaw_offset_deg);
    ekf_changed = true;
    handled = true;
  } else if (name == "ekf.pitch_offset_deg") {
    state_.ekf_pitch_offset_deg = JsonToDouble(val, state_.ekf_pitch_offset_deg);
    ekf_changed = true;
    handled = true;
  } else if (name == "ekf.low_speed_delay_ms") {
    state_.ekf_low_speed_delay_ms = JsonToDouble(val, state_.ekf_low_speed_delay_ms);
    ekf_changed = true;
    handled = true;
  } else if (name == "ekf.high_speed_delay_ms") {
    state_.ekf_high_speed_delay_ms = JsonToDouble(val, state_.ekf_high_speed_delay_ms);
    ekf_changed = true;
    handled = true;
  } else if (name == "ekf.bullet_speed") {
    state_.ekf_bullet_speed = JsonToDouble(val, state_.ekf_bullet_speed);
    ekf_changed = true;
    handled = true;
  } else if (name == "sim.yaw_deg") {
    state_.sim_yaw_deg = JsonToDouble(val, state_.sim_yaw_deg);
    handled = true;
  } else if (name == "sim.pitch_deg") {
    state_.sim_pitch_deg = JsonToDouble(val, state_.sim_pitch_deg);
    handled = true;
  } else if (name == "sim.roll_deg") {
    state_.sim_roll_deg = JsonToDouble(val, state_.sim_roll_deg);
    handled = true;
  } else if (name == "voter.auto_fire") {
    state_.voter_auto_fire = JsonToBool(val, state_.voter_auto_fire);
    voter_changed = true;
    handled = true;
  } else if (name == "voter.min_lock_frames") {
    state_.voter_min_lock_frames = JsonToInt(val, state_.voter_min_lock_frames);
    voter_changed = true;
    handled = true;
  } else if (name == "voter.first_tolerance_rad") {
    state_.voter_first_tolerance_rad = JsonToDouble(val, state_.voter_first_tolerance_rad);
    voter_changed = true;
    handled = true;
  } else if (name == "voter.fire_tolerate_frames") {
    state_.voter_fire_tolerate_frames = JsonToInt(val, state_.voter_fire_tolerate_frames);
    voter_changed = true;
    handled = true;
  }

  if (ekf_changed) {
    reinit_ekf_.store(true, std::memory_order_release);
  }
  if (voter_changed) {
    reinit_voter_.store(true, std::memory_order_release);
  }
  return handled;
}

void PredictParamManager::PushToFoxglove(FoxgloveSink& sink) const {
  nlohmann::json p;
  std::lock_guard<std::mutex> lk(mutex_);

  p["enemy_color"] = (state_.enemy_color == mv::ArmorColor::BLUE) ? "blue" : "red";
  p["loop_video"] = state_.loop_video;
  p["playback_fps"] = state_.playback_fps;

  p["ekf.min_detect_count"] = state_.ekf_min_detect_count;
  p["ekf.max_detecting_lost_count"] = state_.ekf_max_detecting_lost_count;
  p["ekf.max_temp_lost_count"] = state_.ekf_max_temp_lost_count;
  p["ekf.process_noise_pos"] = state_.ekf_process_noise_pos;
  p["ekf.process_noise_ang"] = state_.ekf_process_noise_ang;
  p["ekf.yaw_offset_deg"] = state_.ekf_yaw_offset_deg;
  p["ekf.pitch_offset_deg"] = state_.ekf_pitch_offset_deg;
  p["ekf.low_speed_delay_ms"] = state_.ekf_low_speed_delay_ms;
  p["ekf.high_speed_delay_ms"] = state_.ekf_high_speed_delay_ms;
  p["ekf.bullet_speed"] = state_.ekf_bullet_speed;

  p["voter.auto_fire"] = state_.voter_auto_fire;
  p["voter.min_lock_frames"] = state_.voter_min_lock_frames;
  p["voter.first_tolerance_rad"] = state_.voter_first_tolerance_rad;
  p["voter.fire_tolerate_frames"] = state_.voter_fire_tolerate_frames;

  p["sim.yaw_deg"] = state_.sim_yaw_deg;
  p["sim.pitch_deg"] = state_.sim_pitch_deg;
  p["sim.roll_deg"] = state_.sim_roll_deg;

  sink.UpdateParameters(p);
}

// ============================================================================
// YAML 注入
// ============================================================================

void PredictParamManager::InjectParamsToYaml(YAML::Node& root) const {
  std::lock_guard<std::mutex> lk(mutex_);

  root["auto_aim"]["enemy_color"] = (state_.enemy_color == mv::ArmorColor::BLUE) ? "blue" : "red";

  // ── EKF 预测器字段 ────────────────────────────────────────────────────────
  auto ekf = root["auto_aim"]["ekf_predictor"];
  ekf["min_detect_count"] = state_.ekf_min_detect_count;
  ekf["max_detecting_lost_count"] = state_.ekf_max_detecting_lost_count;
  ekf["max_temp_lost_count"] = state_.ekf_max_temp_lost_count;
  ekf["process_noise_pos"] = state_.ekf_process_noise_pos;
  ekf["process_noise_ang"] = state_.ekf_process_noise_ang;
  ekf["process_noise_outpost_pos"] = 10.0;
  ekf["process_noise_outpost_ang"] = 0.1;
  ekf["outpost_max_temp_lost_count"] = 30;
  ekf["max_dt_sec"] = 0.1;
  ekf["init_radius_small"] = 0.27;
  ekf["init_radius_big"] = 0.27;
  ekf["init_radius_outpost"] = 0.26;
  ekf["divergence_threshold"] = 1.0e6;
  ekf["yaw_offset_deg"] = state_.ekf_yaw_offset_deg;
  ekf["pitch_offset_deg"] = state_.ekf_pitch_offset_deg;
  ekf["low_speed_delay_ms"] = state_.ekf_low_speed_delay_ms;
  ekf["high_speed_delay_ms"] = state_.ekf_high_speed_delay_ms;
  ekf["decision_speed"] = 25.0;
  ekf["bullet_speed"] = state_.ekf_bullet_speed;

  // EKF 初始协方差（11 维），使用典型值
  if (!ekf["P0_diag"] || !ekf["P0_diag"].IsSequence()) {
    ekf["P0_diag"] = std::vector<double>{1, 1, 1, 1, 1, 1, 1, 1, 0.05, 1e-3, 1e-3};
  }

  // ── CooldownVoter 字段 ────────────────────────────────────────────────────
  auto voter = root["auto_aim"]["cooldown_voter"];
  voter["auto_fire"] = state_.voter_auto_fire;
  voter["min_lock_frames"] = state_.voter_min_lock_frames;
  voter["first_tolerance_rad"] = state_.voter_first_tolerance_rad;
  voter["fire_tolerate_frames"] = state_.voter_fire_tolerate_frames;

  // 兼容旧键（SimpleVoter / shooter 节点）
  root["auto_aim"]["shooter"]["auto_fire"] = state_.voter_auto_fire;
}

// ============================================================================
// JSON 构建器
// ============================================================================

nlohmann::json PredictParamManager::MakeTargetStateJson(const mv::TrackTarget& t,
                                                        const mv::GimbalControl& ctrl) {
  constexpr double RAD2DEG = 180.0 / M_PI;
  nlohmann::json j;
  j["is_tracking"] = t.is_tracking;
  j["tracker_state"] = t.tracker_state;

  // ArmorNumber → 字符串
  auto num_str = [](mv::ArmorNumber n) -> std::string {
    switch (n) {
      case mv::ArmorNumber::ONE:
        return "1";
      case mv::ArmorNumber::TWO:
        return "2";
      case mv::ArmorNumber::THREE:
        return "3";
      case mv::ArmorNumber::FOUR:
        return "4";
      case mv::ArmorNumber::FIVE:
        return "5";
      case mv::ArmorNumber::SENTRY:
        return "S";
      case mv::ArmorNumber::OUTPOST:
        return "OP";
      case mv::ArmorNumber::BASE:
        return "BS";
      default:
        return "?";
    }
  };

  j["number"] = num_str(t.number);
  j["color"] = (t.color == mv::ArmorColor::RED) ? "RED" : "BLUE";

  j["position"]["x"] = t.position.x();
  j["position"]["y"] = t.position.y();
  j["position"]["z"] = t.position.z();
  j["velocity"]["x"] = t.velocity.x();
  j["velocity"]["y"] = t.velocity.y();
  j["velocity"]["z"] = t.velocity.z();
  j["velocity_norm"] = t.velocity.norm();
  j["yaw_predicted_deg"] = t.yaw_predicted * RAD2DEG;
  j["pitch_predicted_deg"] = t.pitch_predicted * RAD2DEG;
  j["armor_count"] = static_cast<int>(t.armor_positions.size());

  nlohmann::json armor_arr = nlohmann::json::array();
  for (size_t i = 0; i < t.armor_positions.size(); ++i) {
    const auto& v = t.armor_positions[i];
    armor_arr.push_back({{"id", static_cast<int>(i)},
                         {"x", v.x()},
                         {"y", v.y()},
                         {"z", v.z()},
                         {"yaw_deg", v.w() * RAD2DEG}});
  }
  j["armor_positions"] = armor_arr;

  j["ctrl_yaw_deg"] = ctrl.yaw * RAD2DEG;
  j["ctrl_pitch_deg"] = ctrl.pitch * RAD2DEG;
  j["ctrl_distance_m"] = ctrl.distance;
  j["ctrl_tracking"] = ctrl.tracking;
  return j;
}

nlohmann::json PredictParamManager::MakeVoterJson(bool fire_permitted, const mv::TrackTarget& t,
                                                  bool voter_auto_fire) {
  nlohmann::json j;
  j["fire_permitted"] = fire_permitted;
  j["is_tracking"] = t.is_tracking;
  j["tracker_state"] = t.tracker_state;
  j["auto_fire_enabled"] = voter_auto_fire;
  return j;
}

nlohmann::json PredictParamManager::MakeParamsSnapshotJson() const {
  std::lock_guard<std::mutex> lk(mutex_);
  nlohmann::json j;

  j["enemy_color"] = (state_.enemy_color == mv::ArmorColor::BLUE) ? "blue" : "red";
  j["loop_video"] = state_.loop_video;
  j["playback_fps"] = state_.playback_fps;

  j["ekf"]["min_detect_count"] = state_.ekf_min_detect_count;
  j["ekf"]["max_detecting_lost_count"] = state_.ekf_max_detecting_lost_count;
  j["ekf"]["max_temp_lost_count"] = state_.ekf_max_temp_lost_count;
  j["ekf"]["process_noise_pos"] = state_.ekf_process_noise_pos;
  j["ekf"]["process_noise_ang"] = state_.ekf_process_noise_ang;
  j["ekf"]["yaw_offset_deg"] = state_.ekf_yaw_offset_deg;
  j["ekf"]["pitch_offset_deg"] = state_.ekf_pitch_offset_deg;
  j["ekf"]["low_speed_delay_ms"] = state_.ekf_low_speed_delay_ms;
  j["ekf"]["high_speed_delay_ms"] = state_.ekf_high_speed_delay_ms;
  j["ekf"]["bullet_speed"] = state_.ekf_bullet_speed;

  j["voter"]["auto_fire"] = state_.voter_auto_fire;
  j["voter"]["min_lock_frames"] = state_.voter_min_lock_frames;
  j["voter"]["first_tolerance_rad"] = state_.voter_first_tolerance_rad;
  j["voter"]["fire_tolerate_frames"] = state_.voter_fire_tolerate_frames;

  j["sim"]["yaw_deg"] = state_.sim_yaw_deg;
  j["sim"]["pitch_deg"] = state_.sim_pitch_deg;
  j["sim"]["roll_deg"] = state_.sim_roll_deg;

  return j;
}

}  // namespace mv::tool
