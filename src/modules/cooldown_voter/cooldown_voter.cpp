/**
 * @file cooldown_voter.cpp
 * @brief CooldownVoter Pimpl 实现与工厂注册
 *
 * 实现阶段：
 *   Stage 8-E  Impl 结构体 + YAML 解析 + Vote() 状态机
 */

#include "cooldown_voter.hpp"

#include "factory/factory.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

namespace mv::modules {

// ── Pimpl 私有数据 ───────────────────────────────────────────────────────────

/**
 * @brief CooldownVoter 全部私有成员
 *
 * 为什么用 Pimpl 而非直接把字段写进类？
 *   此处成员均为基础类型，Pimpl 的编译隔离收益不如 EkfPredictor 明显；
 *   但保持风格统一（项目约定所有正式模块使用 Pimpl），
 *   且未来若引入计时器等重型依赖，可无缝扩展。
 */
struct CooldownVoter::Impl {
  // ── 配置参数 ──────────────────────────────────────────────────────────
  bool auto_fire{false};
  int min_lock_frames{5};
  float first_tolerance_rad{0.05F};
  int fire_tolerate_frames{3};

  // ── 运行时状态 ────────────────────────────────────────────────────────
  int consecutive_track_frames{0};  ///< 连续处于 TRACKING 状态的帧数
  bool fire_permit{false};          ///< 当前是否处于允许开火状态
  int lost_fire_frames{0};          ///< 上次丢失跟踪后经过的帧数

  bool initialized{false};
};

// ── 构造 / 析构 ──────────────────────────────────────────────────────────────

CooldownVoter::CooldownVoter() : impl_(std::make_unique<Impl>()) {}
CooldownVoter::~CooldownVoter() = default;

// ── IVoter 接口实现 ───────────────────────────────────────────────────────────

bool CooldownVoter::Init(const YAML::Node& config) {
  // 兼容两种 YAML 布局：根节点直接含字段，或通过 cooldown_voter 子节点
  const YAML::Node& n = config["cooldown_voter"].IsDefined() ? config["cooldown_voter"] : config;

  impl_->auto_fire = n["auto_fire"].as<bool>(false);
  impl_->min_lock_frames = n["min_lock_frames"].as<int>(5);
  impl_->first_tolerance_rad = n["first_tolerance_rad"].as<float>(0.05F);
  impl_->fire_tolerate_frames = n["fire_tolerate_frames"].as<int>(3);

  impl_->initialized = true;
  return true;
}

bool CooldownVoter::Vote(const TrackTarget& target, const GimbalControl& control) {
  if (target.is_tracking) {
    ++impl_->consecutive_track_frames;
    impl_->lost_fire_frames = 0;

    // 瞄准误差：规划 yaw 与目标 yaw 之差（绝对值）
    const float angular_err =
        std::abs(static_cast<float>(control.yaw) - static_cast<float>(target.yaw_predicted));

    if (impl_->consecutive_track_frames >= impl_->min_lock_frames &&
        angular_err <= impl_->first_tolerance_rad) {
      impl_->fire_permit = true;
    }
  } else {
    impl_->consecutive_track_frames = 0;
    ++impl_->lost_fire_frames;

    // 宽容窗口耗尽后撤销开火许可
    if (impl_->lost_fire_frames > impl_->fire_tolerate_frames) {
      impl_->fire_permit = false;
    }
  }

  return impl_->auto_fire && impl_->fire_permit;
}

void CooldownVoter::Reset() {
  impl_->consecutive_track_frames = 0;
  impl_->fire_permit = false;
  impl_->lost_fire_frames = 0;
}

bool CooldownVoter::IsInitialized() const noexcept {
  return impl_->initialized;
}

// ── 工厂注册 ─────────────────────────────────────────────────────────────────

namespace {
const bool COOLDOWN_VOTER_REGISTERED = [] {
  ::mv::Factory<::mv::IVoter>::Register("cooldown",
                                        [] { return std::make_unique<CooldownVoter>(); });
  return true;
}();
}  // namespace

}  // namespace mv::modules
