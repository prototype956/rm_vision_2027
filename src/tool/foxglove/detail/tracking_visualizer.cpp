/**
 * @file tracking_visualizer.cpp
 * @brief TrackingVisualizer 实现：懒创建三个 SceneUpdate channels + 构建 3D 实体
 */
#include "tool/foxglove/detail/tracking_visualizer.hpp"

#include "tool/foxglove/detail/utils.hpp"

#include <algorithm>
#include <cmath>

#include <spdlog/spdlog.h>

namespace mv::tool::detail {

TrackingVisualizer::TrackingVisualizer(foxglove::Context ctx) : ctx_(std::move(ctx)) {}

void TrackingVisualizer::EnsureChannels() {
  auto ensure = [&](std::optional<foxglove::schemas::SceneUpdateChannel>& ch,
                    const std::string& topic) {
    if (ch.has_value())
      return;
    auto res = foxglove::schemas::SceneUpdateChannel::create(topic, ctx_);
    if (res.has_value()) {
      ch.emplace(std::move(res.value()));
    } else {
      spdlog::warn("[TrackingVisualizer] Failed to create channel '{}': {}", topic,
                   foxglove::strerror(res.error()));
    }
  };
  ensure(armor_pos_ch_, "tracking/armor_positions");
  ensure(rot_center_ch_, "tracking/rotation_center");
  ensure(aim_point_ch_, "tracking/aim_point");
}

void TrackingVisualizer::Publish(const mv::TrackTarget& target, const Eigen::Vector3d& aim_xyz,
                                 const std::string& frame_id, uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);
  EnsureChannels();

  const foxglove::schemas::Timestamp fox_ts = detail::ToTs(ts_ns);

  // ── 1. tracking/armor_positions ──────────────────────────────────────────
  if (armor_pos_ch_.has_value()) {
    foxglove::schemas::SceneUpdate scene;
    {
      foxglove::schemas::SceneEntityDeletion del;
      del.type = foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::ALL;
      del.timestamp = fox_ts;
      scene.deletions.push_back(std::move(del));
    }

    if (target.is_tracking) {
      for (int i = 0; i < static_cast<int>(target.armor_positions.size()); ++i) {
        const auto& ap = target.armor_positions[i];
        foxglove::schemas::SceneEntity ent;
        ent.timestamp = fox_ts;
        ent.frame_id = frame_id;
        ent.id = "armor_" + std::to_string(i);
        ent.lifetime = {0, 300'000'000};  // 0.3 s
        ent.frame_locked = false;

        foxglove::schemas::SpherePrimitive sp;
        sp.pose = detail::MakePose(ap.x(), ap.y(), ap.z(), 0, 0, 0, 1);
        sp.size = foxglove::schemas::Vector3{0.12, 0.12, 0.04};
        sp.color =
            (i % 2 == 0) ? detail::ColorYellow() : foxglove::schemas::Color{1.0f, 0.5f, 0.1f, 0.9f};
        ent.spheres.push_back(std::move(sp));

        foxglove::schemas::TextPrimitive txt;
        txt.pose = detail::MakePose(ap.x(), ap.y(), ap.z() + 0.1, 0, 0, 0, 1);
        txt.text = std::to_string(i);
        txt.font_size = 0.06;
        txt.color = detail::ColorWhite();
        ent.texts.push_back(std::move(txt));

        scene.entities.push_back(std::move(ent));
      }
    }
    armor_pos_ch_->log(scene, ts_ns);
  }

  // ── 2. tracking/rotation_center ──────────────────────────────────────────
  if (rot_center_ch_.has_value()) {
    foxglove::schemas::SceneUpdate scene;
    {
      foxglove::schemas::SceneEntityDeletion del;
      del.type = foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::ALL;
      del.timestamp = fox_ts;
      scene.deletions.push_back(std::move(del));
    }

    if (target.is_tracking) {
      foxglove::schemas::SceneEntity ent;
      ent.timestamp = fox_ts;
      ent.frame_id = frame_id;
      ent.id = "center";
      ent.lifetime = {0, 300'000'000};
      ent.frame_locked = false;

      // 旋转中心——蓝球
      foxglove::schemas::SpherePrimitive sp_c;
      sp_c.pose = detail::MakePose(target.position.x(), target.position.y(), target.position.z(), 0,
                                   0, 0, 1);
      sp_c.size = foxglove::schemas::Vector3{0.08, 0.08, 0.08};
      sp_c.color = detail::ColorBlue();
      ent.spheres.push_back(std::move(sp_c));

      // 速度箭头——绿色（方向 = velocity，长度 = |v| * 0.5s 预测臂长）
      const double vel_len = target.velocity.norm();
      if (vel_len > 0.01) {
        const Eigen::Vector3d dir = target.velocity.normalized();
        const double arr_len = std::clamp(vel_len * 0.5, 0.05, 1.0);
        ent.arrows.push_back(
            detail::MakeArrow(target.position, dir, detail::ColorGreen(), arr_len));
      }

      scene.entities.push_back(std::move(ent));
    }
    rot_center_ch_->log(scene, ts_ns);
  }

  // ── 3. tracking/aim_point ────────────────────────────────────────────────
  if (aim_point_ch_.has_value()) {
    foxglove::schemas::SceneUpdate scene;
    {
      foxglove::schemas::SceneEntityDeletion del;
      del.type = foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::ALL;
      del.timestamp = fox_ts;
      scene.deletions.push_back(std::move(del));
    }

    if (target.is_tracking) {
      foxglove::schemas::SceneEntity ent;
      ent.timestamp = fox_ts;
      ent.frame_id = frame_id;
      ent.id = "aim";
      ent.lifetime = {0, 300'000'000};
      ent.frame_locked = false;

      foxglove::schemas::SpherePrimitive sp;
      sp.pose = detail::MakePose(aim_xyz.x(), aim_xyz.y(), aim_xyz.z(), 0, 0, 0, 1);
      sp.size = foxglove::schemas::Vector3{0.06, 0.06, 0.06};
      sp.color = detail::ColorGreen();
      ent.spheres.push_back(std::move(sp));

      scene.entities.push_back(std::move(ent));
    }
    aim_point_ch_->log(scene, ts_ns);
  }
}

}  // namespace mv::tool::detail
