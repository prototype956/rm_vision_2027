/**
 * @file mpc_voter.cpp
 * @brief MpcVoter Pimpl 实现与工厂注册
 *
 * 实现阶段：
 *   Stage 8-E  Impl 结构体 + YAML 解析 + Vote() 轨迹生成 → MpcPlanner 调用
 */

#include "mpc_voter.hpp"

#include "detail/mpc_planner.hpp"
#include "factory/factory.hpp"

#include <cmath>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace mv::modules {

// ── Pimpl 私有数据 ───────────────────────────────────────────────────────────

struct MpcVoter::Impl {
  bool auto_fire{false};
  bool initialized{false};

  /**
   * MpcPlanner 持有 TinySolver 的所有权，Init() 时根据 YAML 参数实例化。
   * 使用 unique_ptr 是因为 MpcPlanner 构造时需要 YAML 参数，
   * 不能在 Impl 默认构造时初始化。
   */
  std::unique_ptr<detail::MpcPlanner> planner;
};

// ── 构造 / 析构 ──────────────────────────────────────────────────────────────

MpcVoter::MpcVoter() : impl_(std::make_unique<Impl>()) {}
MpcVoter::~MpcVoter() = default;

// ── IVoter 接口实现 ───────────────────────────────────────────────────────────

bool MpcVoter::Init(const YAML::Node& config) {
  // 支持两种 YAML 布局：
  //   a) config["mpc_voter"]["..."]   (由 EkfPredictor 传入 sub-node)
  //   b) config["auto_aim"]["mpc_voter"]["..."]  (直接传入根节点)
  YAML::Node node;  // 值语义，避免悬空引用
  if (config["mpc_voter"].IsDefined()) {
    node = config["mpc_voter"];
  } else if (config["auto_aim"].IsDefined() && config["auto_aim"]["mpc_voter"].IsDefined()) {
    node = config["auto_aim"]["mpc_voter"];
  } else {
    node = config;
  }

  impl_->auto_fire = node["auto_fire"].as<bool>(false);

  detail::MpcPlannerParams p;
  p.dt = node["dt"].as<double>(0.01);
  p.horizon = node["horizon"].as<int>(100);
  p.fire_thresh = node["fire_thresh"].as<double>(0.05);
  p.max_iter = node["max_iter"].as<int>(10);
  p.rho = node["rho"].as<double>(1.0);
  p.Q_yaw = node["Q_yaw"].as<double>(1.0);
  p.Q_yaw_vel = node["Q_yaw_vel"].as<double>(1.0);
  p.Q_pitch = node["Q_pitch"].as<double>(1.0);
  p.Q_pitch_vel = node["Q_pitch_vel"].as<double>(1.0);
  p.R_yaw = node["R_yaw"].as<double>(1.0);
  p.R_pitch = node["R_pitch"].as<double>(1.0);
  p.max_yaw_acc = node["max_yaw_acc"].as<double>(200.0);
  p.max_pitch_acc = node["max_pitch_acc"].as<double>(200.0);

  impl_->planner = std::make_unique<detail::MpcPlanner>(p);

  impl_->initialized = true;
  return true;
}

bool MpcVoter::Vote(const TrackTarget& target, const GimbalControl& control) {
  if (!impl_->auto_fire || !target.is_tracking)
    return false;
  if (!impl_->planner)
    return false;

  // ── 参考轨迹生成（线性外推）────────────────────────────────────────────────
  // 使用目标中心点的位置 + 速度向量进行等速外推，每步 dt 秒。
  // pitch 使用仰角公式：pitch = -atan2(z, hypot(x, y))
  const auto& p = impl_->planner->params();
  const int N = p.horizon;
  const double dt = p.dt;

  Eigen::VectorXd ref_yaw(N);
  Eigen::VectorXd ref_pitch(N);

  for (int i = 0; i < N; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double cx = target.position.x() + target.velocity.x() * t;
    const double cy = target.position.y() + target.velocity.y() * t;
    const double cz = target.position.z() + target.velocity.z() * t;
    ref_yaw[i] = std::atan2(cy, cx);
    ref_pitch[i] = -std::atan2(cz, std::hypot(cx, cy));
  }

  // ── MPC 求解 ─────────────────────────────────────────────────────────────
  const auto result = impl_->planner->Plan(control.yaw, /*yaw_vel=*/0.0, control.pitch,
                                           /*pitch_vel=*/0.0, ref_yaw, ref_pitch);

  return result.fire;
}

void MpcVoter::Reset() {
  // TODO(Stage 8-E): 若 MpcPlanner 有需要重置的内部状态，在此调用。
  //   当前 TinyMPC 求解器无持久化状态，Reset() 可为空操作。
}

bool MpcVoter::IsInitialized() const noexcept {
  return impl_->initialized;
}

// ── 工厂注册 ─────────────────────────────────────────────────────────────────

namespace {
const bool MPC_VOTER_REGISTERED = [] {
  ::mv::Factory<::mv::IVoter>::Register("mpc", [] { return std::make_unique<MpcVoter>(); });
  return true;
}();
}  // namespace

}  // namespace mv::modules
