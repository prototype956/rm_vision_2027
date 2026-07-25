/**
 * @file ekf_predictor.cpp
 * @brief EkfPredictor Pimpl 实现与工厂注册（Stage 8-C/D）
 *
 * Stage 8-C: Impl 结构体 + YAML 解析 + EkfTracker/TrajectorySolver 初始化
 * Stage 8-D: Predict() 核心流程（坐标系变换 + 追踪 + 弹道求解）
 */

#include "ekf_predictor.hpp"

#include "core/logger.hpp"
#include "detail/ekf_track_target.hpp"
#include "detail/ekf_tracker.hpp"
#include "detail/trajectory_solver.hpp"
#include "factory/factory.hpp"

#include <chrono>

#include <Eigen/Geometry>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace mv::modules {

// ── Pimpl 私有数据 ───────────────────────────────────────────────────────────

struct EkfPredictor::Impl {
  // ── 核心算法组件 ───────────────────────────────────────────────────────
  detail::EkfTracker tracker;
  detail::TrajectorySolver solver;

  // ── 坐标系变换 ─────────────────────────────────────────────────────────
  /// 云台系 → 世界系旋转矩阵（由 SetGimbalOrientation() 每帧更新）
  Eigen::Matrix3d R_gimbal2world{Eigen::Matrix3d::Identity()};

  // ── 弹速 ───────────────────────────────────────────────────────────────
  /// 从 YAML 读取的默认弹速（m/s）；Stage 8-F 接入串口后可改为原子量
  double bullet_speed{23.0};

  // ── 杂项 ───────────────────────────────────────────────────────────────
  TrackTarget last_track_target{};
  YAML::Node config{};
  bool initialized{false};

  // ── 构造（需要 EkfTracker/TrajectorySolver 的默认参数先就绪）─────────
  Impl() : tracker(detail::EkfTrackerParams{}), solver(detail::TrajectorySolverParams{}) {}
};

// ── 构造 / 析构 ──────────────────────────────────────────────────────────────

EkfPredictor::EkfPredictor() : impl_(std::make_unique<Impl>()) {}

EkfPredictor::~EkfPredictor() = default;

// ── Init ─────────────────────────────────────────────────────────────────────

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool EkfPredictor::Init(const YAML::Node& config) {
  impl_->config = config;

  // 兼容三种 YAML 布局：
  //   1. 传入 vision.yaml 根节点：root.auto_aim.ekf_predictor
  //   2. 传入 auto_aim 子树：auto_aim.ekf_predictor
  //   3. 传入 ekf_predictor 子树：字段直接位于当前节点
  YAML::Node n = config;
  if (config["auto_aim"].IsDefined() && config["auto_aim"]["ekf_predictor"].IsDefined()) {
    n = config["auto_aim"]["ekf_predictor"];
  } else if (config["ekf_predictor"].IsDefined()) {
    n = config["ekf_predictor"];
  }
  YAML::Node config_node = n;

  auto read_double_key = [&](const char* key, double fallback) {
    if (config_node.IsMap() && config_node[key].IsDefined()) {
      return config_node[key].as<double>(fallback);
    }
    return fallback;
  };

  auto read_int_key = [&](const char* key, int fallback) {
    if (config_node.IsMap() && config_node[key].IsDefined()) {
      return config_node[key].as<int>(fallback);
    }
    return fallback;
  };

  // ── 从 YAML 填充 EkfTrackerParams ──────────────────────────────────────
  detail::EkfTrackerParams tp;
  tp.min_detect_count = read_int_key("min_detect_count", 5);
  tp.max_detecting_lost_count = read_int_key("max_detecting_lost_count", 2);
  tp.max_temp_lost_count = read_int_key("max_temp_lost_count", 15);
  tp.outpost_max_temp_lost_count = read_int_key("outpost_max_temp_lost_count", 30);
  tp.max_dt_sec = read_double_key("max_dt_sec", 0.1);

  tp.init_radius_small = read_double_key("init_radius_small", 0.27);
  tp.init_radius_big = read_double_key("init_radius_big", 0.27);
  tp.init_radius_outpost = read_double_key("init_radius_outpost", 0.26);

  YAML::Node process_noise_node;
  if (config_node.IsMap() && config_node["process_noise"].IsDefined() &&
      config_node["process_noise"].IsMap()) {
    process_noise_node = config_node["process_noise"];
  }

  if (process_noise_node.IsMap() && process_noise_node["normal"].IsDefined() &&
      process_noise_node["normal"].IsMap()) {
    YAML::Node normal_noise = process_noise_node["normal"];
    tp.process_noise_pos = normal_noise["pos"].as<double>(100.0);
    tp.process_noise_ang = normal_noise["ang"].as<double>(400.0);
  } else {
    tp.process_noise_pos = read_double_key("process_noise_pos", 100.0);
    tp.process_noise_ang = read_double_key("process_noise_ang", 400.0);
  }

  if (process_noise_node.IsMap() && process_noise_node["outpost"].IsDefined() &&
      process_noise_node["outpost"].IsMap()) {
    YAML::Node outpost_noise = process_noise_node["outpost"];
    tp.process_noise_outpost_pos = outpost_noise["pos"].as<double>(10.0);
    tp.process_noise_outpost_ang = outpost_noise["ang"].as<double>(0.1);
  } else {
    tp.process_noise_outpost_pos = read_double_key("process_noise_outpost_pos", 10.0);
    tp.process_noise_outpost_ang = read_double_key("process_noise_outpost_ang", 0.1);
  }

  tp.divergence_threshold = read_double_key("divergence_threshold", 1e6);

  // 初始协方差对角（11 维）
  YAML::Node p0_diag_node;
  if (config_node.IsMap() && config_node["p0_diag"].IsDefined()) {
    p0_diag_node = config_node["p0_diag"];
  }

  if (p0_diag_node.IsMap() && p0_diag_node["default"].IsDefined()) {
    auto vec = p0_diag_node["default"].as<std::vector<double>>();
    if (vec.size() == 11) {
      tp.P0_diag = Eigen::Map<Eigen::VectorXd>(vec.data(), 11);
    }
  } else if (config_node.IsMap() && config_node["P0_diag"].IsDefined()) {
    auto vec = config_node["P0_diag"].as<std::vector<double>>();
    if (vec.size() == 11) {
      tp.P0_diag = Eigen::Map<Eigen::VectorXd>(vec.data(), 11);
    }
  }

  // ── 从 YAML 填充 TrajectorySolverParams ────────────────────────────────
  detail::TrajectorySolverParams sp;
  sp.yaw_offset_rad = read_double_key("yaw_offset_deg", 0.0) / 57.295779513;
  sp.pitch_offset_rad = read_double_key("pitch_offset_deg", 0.0) / 57.295779513;
  sp.low_speed_delay_ms = read_double_key("low_speed_delay_ms", 100.0);
  sp.high_speed_delay_ms = read_double_key("high_speed_delay_ms", 70.0);
  sp.decision_speed = read_double_key("decision_speed", 25.0);
  sp.max_iter = read_int_key("max_iter", 10);
  sp.iter_converge_ms = read_double_key("iter_converge_ms", 1.0);
  sp.max_approaching_angle = read_double_key("max_approaching_angle", 1.047);  // 60 deg
  sp.max_leaving_angle = read_double_key("max_leaving_angle", 0.349);          // 20 deg

  // ── 弹速默认值 ─────────────────────────────────────────────────────────
  impl_->bullet_speed = read_double_key("bullet_speed", 23.0);

  // ── 实例化算法组件 ─────────────────────────────────────────────────────
  impl_->tracker = detail::EkfTracker(tp);
  impl_->solver = detail::TrajectorySolver(sp);

  impl_->initialized = true;
  bool grouped_noise_defined = process_noise_node.IsMap();
  bool grouped_p0_defined = p0_diag_node.IsMap();
  bool has_outpost_radius_key =
      config_node.IsMap() && config_node["init_radius_outpost"].IsDefined();
  MV_LOG_INFO("EkfPredictor",
              "Init OK (min_detect={}, bullet_speed={:.1f} m/s, r_small={:.3f}, r_big={:.3f}, "
              "r_outpost={:.3f}, has_outpost_key={}, grouped_noise={}, grouped_p0={})",
              tp.min_detect_count, impl_->bullet_speed, tp.init_radius_small, tp.init_radius_big,
              tp.init_radius_outpost, has_outpost_radius_key, grouped_noise_defined,
              grouped_p0_defined);
  return true;
}

// ── SetGimbalOrientation ─────────────────────────────────────────────────────

void EkfPredictor::SetGimbalOrientation(const Eigen::Quaterniond& q) {
  // q 为"从云台系到 IMU 绝对系（世界系）"的旋转四元数
  // xyz_world = R_gimbal2world · xyz_gimbal
  impl_->R_gimbal2world = q.normalized().toRotationMatrix();
}

// ── Predict ──────────────────────────────────────────────────────────────────

GimbalControl EkfPredictor::Predict(const std::vector<Detection>& detections,
                                    std::chrono::steady_clock::time_point timestamp,
                                    ArmorColor enemy_color) {
  // 1. 将 xyz_in_gimbal 旋转到世界系，存回 xyz_in_gimbal 字段
  //    （EkfTrackTarget / EkfTracker 约定：传入的 xyz_in_gimbal 已是世界系）
  std::vector<Detection> world_dets;
  world_dets.reserve(detections.size());
  for (Detection det : detections) {
    if (!det.is_solved)
      continue;  // 未完成 PnP 解算的板子无 3D 坐标
    det.xyz_in_gimbal = impl_->R_gimbal2world * det.xyz_in_gimbal;
    // yaw_angle 也需要旋转（偏航角加 R 的 yaw 分量）
    // 取旋转矩阵对应的 yaw（绕 Z 轴的分量）
    const double world_yaw = std::atan2(impl_->R_gimbal2world(1, 0), impl_->R_gimbal2world(0, 0));
    det.yaw_angle += world_yaw;
    world_dets.push_back(std::move(det));
  }

  // 2. 调用 EkfTracker 状态机
  const std::optional<detail::EkfTrackTarget> maybe_target =
      impl_->tracker.Track(world_dets, timestamp, enemy_color);

  // 3. 无追踪目标
  if (!maybe_target.has_value()) {
    impl_->last_track_target.is_tracking = false;
    impl_->last_track_target.tracker_state =
        std::string(detail::EkfTracker::StateName(impl_->tracker.GetState()));
    impl_->last_track_target.armor_positions.clear();
    GimbalControl ctrl{};
    ctrl.tracking = false;
    ctrl.fire = false;
    ctrl.timestamp = timestamp;
    return ctrl;
  }

  // 4. 弹道求解（操作副本，不修改 tracker 内部状态）
  GimbalControl ctrl = impl_->solver.Solve(*maybe_target, timestamp, impl_->bullet_speed);

  // 5. 更新 last_track_target（供 GetTrackTarget() 使用）
  const Eigen::VectorXd& x = maybe_target->ekf_x();
  impl_->last_track_target.is_tracking = ctrl.tracking;
  impl_->last_track_target.number = maybe_target->name;
  impl_->last_track_target.color = enemy_color;
  impl_->last_track_target.yaw_predicted = ctrl.yaw;
  impl_->last_track_target.pitch_predicted = ctrl.pitch;
  impl_->last_track_target.tracker_state =
      std::string(detail::EkfTracker::StateName(impl_->tracker.GetState()));
  // position = 旋转中心（世界系，cx/cy/cz 分别在 x[0]/x[2]/x[4]）
  impl_->last_track_target.position = Eigen::Vector3d(x[0], x[2], x[4]);
  // velocity = 旋转中心速度（dcx/dcy/dcz 在 x[1]/x[3]/x[5]）
  impl_->last_track_target.velocity = Eigen::Vector3d(x[1], x[3], x[5]);
  // armor_positions = EKF 估计的所有装甲板世界坐标 [x,y,z,yaw]
  impl_->last_track_target.armor_positions = maybe_target->ArmorXyzaList();

  return ctrl;
}

// ── GetTrackTarget / Reset / IsInitialized ────────────────────────────────────

TrackTarget EkfPredictor::GetTrackTarget() const {
  return impl_->last_track_target;
}

void EkfPredictor::Reset() {
  impl_->tracker.Reset();
  impl_->last_track_target = TrackTarget{};
}

bool EkfPredictor::IsInitialized() const noexcept {
  return impl_->initialized;
}

// ── 工厂注册 ─────────────────────────────────────────────────────────────────

namespace {
const bool EKF_PREDICTOR_REGISTERED = [] {
  ::mv::Factory<::mv::IPredictor>::Register("ekf", [] { return std::make_unique<EkfPredictor>(); });
  return true;
}();
}  // namespace

}  // namespace mv::modules
