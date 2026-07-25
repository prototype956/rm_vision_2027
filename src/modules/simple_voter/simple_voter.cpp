/**
 * @file simple_voter.cpp
 * @brief 简单开火投票器实现
 */
#include "simple_voter.hpp"

#include "core/logger.hpp"
#include "factory/factory.hpp"

namespace mv::modules {

namespace {
const bool SIMPLE_VOTER_REGISTERED = [] {
  ::mv::Factory<::mv::IVoter>::Register("simple", [] { return std::make_unique<SimpleVoter>(); });
  return true;
}();
}  // namespace

SimpleVoter::SimpleVoter() = default;
SimpleVoter::~SimpleVoter() = default;

bool SimpleVoter::Init(const YAML::Node& config) {
  if (config && config["auto_aim"] && config["auto_aim"]["shooter"]) {
    const auto& shooter = config["auto_aim"]["shooter"];
    if (shooter["auto_fire"]) {
      auto_fire_ = shooter["auto_fire"].as<bool>();
    }
  }
  initialized_ = true;
  MV_LOG_INFO("SimpleVoter", "Init OK — auto_fire={}", auto_fire_);
  return true;
}

bool SimpleVoter::Vote(const TrackTarget& target, const GimbalControl& /*control*/) {
  return auto_fire_ && target.is_tracking;
}

void SimpleVoter::Reset() {
  MV_LOG_DEBUG("SimpleVoter", "Reset");
}

}  // namespace mv::modules
