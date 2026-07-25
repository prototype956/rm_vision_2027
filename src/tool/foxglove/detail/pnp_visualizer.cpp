/**
 * @file pnp_visualizer.cpp
 * @brief PnpVisualizer 实现（三层 PnP 调试输出）
 *
 * 【三层输出详解】
 *
 * Layer 1 — pnp/debug_image
 *   在底图安全备份（clone 或 cvtColor）上依次绘制：
 *   - 绿色实心圆：4 个原始角点检测位置
 *   - 绿色轮廓线：角点间 LINE_LOOP 连线
 *   - 青色实心圆并连接：主解的重投影角点（察观 PnP 残差）
 *   - 橙色十字：第二 IPPE 解的重投影角点（察观解的歧义性）
 *   - 标签文字："#N d.ddm err:x.xpx" 或 "#N no pnp"
 *
 * Layer 2 — pnp/axes_3d
 *   对每块已解算装甲板在云台坐标系中放置 RGB XYZ 坐标轴，
 *   轴长按 dist*0.12 自适应，适配远/近目标的可视性。
 *   若存在第二 IPPE 解，另行推送橙色树形展示解的歧义性。
 *
 * Layer 3 — pnp/residuals
 *   JSON 格式输出每块装甲板的位姿数据，提供给
 *   Foxglove 原始 JSON 面板或外部日志分析。
 */
#include "tool/foxglove/detail/pnp_visualizer.hpp"

#include "tool/foxglove/detail/utils.hpp"

#include <cstring>
#include <sstream>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace mv::tool::detail {

namespace {

foxglove::schemas::Quaternion FacingOriginQuaternion(const Eigen::Vector3d& point_in_gimbal) {
  if (point_in_gimbal.norm() < 1e-6) {
    return {0.0, 0.0, 0.0, 1.0};
  }

  const Eigen::Vector3d NORMAL_TO_GIMBAL = (-point_in_gimbal).normalized();
  const Eigen::Quaterniond Q =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), NORMAL_TO_GIMBAL);
  return {Q.x(), Q.y(), Q.z(), Q.w()};
}

}  // namespace

PnpVisualizer::PnpVisualizer(foxglove::Context ctx) : ctx_(std::move(ctx)) {
  // 在构造时立即注册所有频道，确保 Foxglove 客户端连接时已完成 advertise。
  // 若延迟到第一次 Publish() 才注册，Foxglove 握手后才 advertise 的频道
  // 不会出现在话题面板中（WebSocket 协议限制）。
  EnsureChannels();
}

void PnpVisualizer::EnsureChannels() {
  if (!debug_img_ch_.has_value()) {
    auto res = foxglove::schemas::RawImageChannel::create("pnp/debug_image", ctx_);
    if (res.has_value()) {
      debug_img_ch_.emplace(std::move(res.value()));
    } else {
      spdlog::error("[PnpVisualizer] Failed to create pnp/debug_image: {}",
                    foxglove::strerror(res.error()));
    }
  }
  if (!axes_ch_.has_value()) {
    auto res = foxglove::schemas::SceneUpdateChannel::create("pnp/axes_3d", ctx_);
    if (res.has_value()) {
      axes_ch_.emplace(std::move(res.value()));
    } else {
      spdlog::error("[PnpVisualizer] Failed to create pnp/axes_3d: {}",
                    foxglove::strerror(res.error()));
    }
  }
  if (!residuals_ch_.has_value()) {
    static const std::string SCHEMA_STR =
        R"({"type":"object","properties":{"timestamp_ns":{"type":"integer"},"count":{"type":"integer"},"armors":{"type":"array","items":{"type":"object","properties":{"index":{"type":"integer"},"solved":{"type":"boolean"},"label":{"type":"string"},"x_m":{"type":"number"},"y_m":{"type":"number"},"z_m":{"type":"number"},"dist_m":{"type":"number"},"yaw_deg":{"type":"number"},"pitch_deg":{"type":"number"},"reproj_error_px":{"type":"number"}}}}}})";
    foxglove::Schema schema;
    schema.name = "PnpResiduals";
    schema.encoding = "jsonschema";
    schema.data = reinterpret_cast<const std::byte*>(SCHEMA_STR.data());
    schema.data_len = SCHEMA_STR.size();
    auto res = foxglove::RawChannel::create("pnp/residuals", "json", schema, ctx_);
    if (res.has_value()) {
      residuals_ch_.emplace(std::move(res.value()));
    } else {
      spdlog::error("[PnpVisualizer] Failed to create pnp/residuals: {}",
                    foxglove::strerror(res.error()));
    }
  }
}

void PnpVisualizer::Publish(const std::vector<mv::Detection>& dets, const cv::Mat& frame,
                            uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);
  EnsureChannels();

  // ── Layer 1: debug_image ─────────────────────────────────────────────────

  if (debug_img_ch_.has_value() && !frame.empty()) {
    cv::Mat annotated;
    if (frame.channels() == 1) {
      cv::cvtColor(frame, annotated, cv::COLOR_GRAY2BGR);
    } else {
      annotated = frame.clone();
    }

    for (size_t i = 0; i < dets.size(); ++i) {
      const auto& det = dets[i];

      // 4 个角点（绿色实心圆）
      for (const auto& pt : det.points) {
        cv::circle(annotated, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)), 5,
                   cv::Scalar(0, 255, 0), -1);
      }

      // 中心点（黄色）
      cv::Point2f ctr = det.Center();
      cv::circle(annotated, cv::Point(static_cast<int>(ctr.x), static_cast<int>(ctr.y)), 4,
                 cv::Scalar(0, 255, 255), -1);

      // 角点间连线（绿色虚线轮廓）
      for (int k = 0; k < 4; ++k) {
        cv::line(annotated,
                 cv::Point(static_cast<int>(det.points[k].x), static_cast<int>(det.points[k].y)),
                 cv::Point(static_cast<int>(det.points[(k + 1) % 4].x),
                           static_cast<int>(det.points[(k + 1) % 4].y)),
                 cv::Scalar(0, 200, 0), 1);
      }

      // 重投影角点（青色——主解，仓考 Z>0 且误差更小）
      if (det.is_solved) {
        for (int k = 0; k < 4; ++k) {
          cv::circle(annotated,
                     cv::Point(static_cast<int>(det.reprojected_points[k].x),
                               static_cast<int>(det.reprojected_points[k].y)),
                     4, cv::Scalar(255, 255, 0), 2);
        }
        for (int k = 0; k < 4; ++k) {
          cv::line(annotated,
                   cv::Point(static_cast<int>(det.reprojected_points[k].x),
                             static_cast<int>(det.reprojected_points[k].y)),
                   cv::Point(static_cast<int>(det.reprojected_points[(k + 1) % 4].x),
                             static_cast<int>(det.reprojected_points[(k + 1) % 4].y)),
                   cv::Scalar(200, 200, 0), 1);
        }
      }

      // 重投影角点（橙色虹——第二 IPPE 解，是 IPPE 的备选解）
      if (det.has_alt_solution) {
        for (int k = 0; k < 4; ++k) {
          cv::drawMarker(annotated,
                         cv::Point(static_cast<int>(det.reprojected_points_alt[k].x),
                                   static_cast<int>(det.reprojected_points_alt[k].y)),
                         cv::Scalar(0, 100, 255), cv::MARKER_CROSS, 8, 1);
        }
        for (int k = 0; k < 4; ++k) {
          cv::line(annotated,
                   cv::Point(static_cast<int>(det.reprojected_points_alt[k].x),
                             static_cast<int>(det.reprojected_points_alt[k].y)),
                   cv::Point(static_cast<int>(det.reprojected_points_alt[(k + 1) % 4].x),
                             static_cast<int>(det.reprojected_points_alt[(k + 1) % 4].y)),
                   cv::Scalar(0, 80, 200), 1, cv::LINE_AA);
        }
      }

      // 标签（距离 / 重投影误差 / 未解算提示）
      std::string text;
      if (det.is_solved) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "#%zu %.2fm err:%.1fpx", i, det.xyz_in_gimbal.norm(),
                      det.reproj_error);
        text = buf;
      } else {
        text = "#" + std::to_string(i) + " no pnp";
      }
      cv::putText(annotated, text,
                  cv::Point(static_cast<int>(ctr.x) + 5, static_cast<int>(ctr.y) - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(100, 255, 100), 1, cv::LINE_AA);
    }

    // 发布标注图
    foxglove::schemas::RawImage img_msg;
    img_msg.frame_id = "camera";
    img_msg.width = static_cast<uint32_t>(annotated.cols);
    img_msg.height = static_cast<uint32_t>(annotated.rows);
    img_msg.step = static_cast<uint32_t>(annotated.step);
    img_msg.encoding = "bgr8";
    img_msg.timestamp = ToTs(ts_ns);
    const size_t data_sz = annotated.total() * annotated.elemSize();
    img_msg.data.resize(data_sz);
    std::memcpy(img_msg.data.data(), annotated.data, data_sz);
    debug_img_ch_->log(img_msg, ts_ns);
  }

  // ── Layer 2: axes_3d ─────────────────────────────────────────────────────

  if (axes_ch_.has_value()) {
    foxglove::schemas::SceneUpdate scene;

    for (size_t i = 0; i < dets.size(); ++i) {
      const auto& det = dets[i];
      if (!det.is_solved) {
        continue;
      }

      Eigen::Vector3d origin = det.xyz_in_gimbal;
      // 轴长按距离自适应：远目标轴更长便于辨认，近目标不至遮盖装甲板本身
      // 公式：dist * 0.12，限制在 [0.08, 0.4] m 内防止极端值
      const double dist = origin.norm();
      const double ax_len = std::clamp(dist * 0.12, 0.08, 0.4);

      foxglove::schemas::SceneEntity entity;
      entity.frame_id = "gimbal";
      entity.id = "pnp_axes_" + std::to_string(i);
      entity.timestamp = ToTs(ts_ns);
      entity.lifetime = foxglove::schemas::Duration{0, 400'000'000};  // 400ms
      entity.frame_locked = false;

      // 主解：RGB XYZ 坐标轴（鲜色）
      entity.arrows.push_back(MakeArrow(origin, Eigen::Vector3d::UnitX(), ColorRed(), ax_len));
      entity.arrows.push_back(MakeArrow(origin, Eigen::Vector3d::UnitY(), ColorGreen(), ax_len));
      entity.arrows.push_back(MakeArrow(origin, Eigen::Vector3d::UnitZ(), ColorBlue(), ax_len));

      // 装甲板平面（半透明浅蓝矩形，展示装甲板在 3D 场景中的实际尺寸）
      foxglove::schemas::CubePrimitive armor_plane;
      const double hw = (det.type == mv::ArmorType::BIG) ? big_half_w_ : small_half_w_;
      const auto ORIENTATION = FacingOriginQuaternion(origin);
      armor_plane.pose = MakePose(origin.x(), origin.y(), origin.z(), ORIENTATION.x, ORIENTATION.y,
                                  ORIENTATION.z, ORIENTATION.w);
      armor_plane.size = foxglove::schemas::Vector3{hw * 2.0, half_h_ * 2.0, 0.004};
      foxglove::schemas::Color plane_color = {0.3, 0.7, 1.0, 0.45};
      armor_plane.color = plane_color;
      entity.cubes.push_back(std::move(armor_plane));

      scene.entities.push_back(std::move(entity));

      // 第二 IPPE 解：橙色半透明轴（识别解的歧义性）
      if (det.has_alt_solution) {
        Eigen::Vector3d alt_origin = det.xyz_in_gimbal_alt;
        const double alt_dist = alt_origin.norm();
        const double alt_len = std::clamp(alt_dist * 0.12, 0.08, 0.4);

        foxglove::schemas::SceneEntity alt_entity;
        alt_entity.frame_id = "gimbal";
        alt_entity.id = "pnp_axes_alt_" + std::to_string(i);
        alt_entity.timestamp = ToTs(ts_ns);
        alt_entity.lifetime = foxglove::schemas::Duration{0, 400'000'000};
        alt_entity.frame_locked = false;

        foxglove::schemas::Color orange = {1.0, 0.5, 0.0, 0.45};
        alt_entity.arrows.push_back(
            MakeArrow(alt_origin, Eigen::Vector3d::UnitX(), orange, alt_len));
        alt_entity.arrows.push_back(
            MakeArrow(alt_origin, Eigen::Vector3d::UnitY(), orange, alt_len));
        alt_entity.arrows.push_back(
            MakeArrow(alt_origin, Eigen::Vector3d::UnitZ(), orange, alt_len));

        // 第二解的装甲板平面（橙色半透明）
        foxglove::schemas::CubePrimitive alt_plane;
        const double ahw = (det.type == mv::ArmorType::BIG) ? big_half_w_ : small_half_w_;
        const auto ALT_ORIENTATION = FacingOriginQuaternion(alt_origin);
        alt_plane.pose = MakePose(alt_origin.x(), alt_origin.y(), alt_origin.z(), ALT_ORIENTATION.x,
                                  ALT_ORIENTATION.y, ALT_ORIENTATION.z, ALT_ORIENTATION.w);
        alt_plane.size = foxglove::schemas::Vector3{ahw * 2.0, half_h_ * 2.0, 0.004};
        alt_plane.color = {1.0, 0.45, 0.0, 0.3};
        alt_entity.cubes.push_back(std::move(alt_plane));

        scene.entities.push_back(std::move(alt_entity));
      }
    }

    axes_ch_->log(scene, ts_ns);
  }

  // ── Layer 3: residuals (JSON) ────────────────────────────────────────────

  if (residuals_ch_.has_value()) {
    nlohmann::json doc;
    doc["timestamp_ns"] = ts_ns;
    doc["count"] = static_cast<int>(dets.size());
    nlohmann::json armors = nlohmann::json::array();

    for (size_t i = 0; i < dets.size(); ++i) {
      const auto& det = dets[i];
      nlohmann::json a;
      a["index"] = static_cast<int>(i);
      a["solved"] = det.is_solved;
      a["label"] = ArmorLabel(det);
      if (det.is_solved) {
        double dist = det.xyz_in_gimbal.norm();
        a["x_m"] = det.xyz_in_gimbal.x();
        a["y_m"] = det.xyz_in_gimbal.y();
        a["z_m"] = det.xyz_in_gimbal.z();
        a["dist_m"] = dist;
        // rad → deg
        constexpr double RAD_TO_DEG = 180.0 / std::numbers::pi;
        a["yaw_deg"] = det.yaw_angle * RAD_TO_DEG;
        a["pitch_deg"] = det.pitch_angle * RAD_TO_DEG;
        a["reproj_error_px"] = det.reproj_error;
      }
      armors.push_back(std::move(a));
    }
    doc["armors"] = std::move(armors);

    std::string json_str = doc.dump();
    residuals_ch_->log(reinterpret_cast<const std::byte*>(json_str.data()), json_str.size(), ts_ns);
  }
}

}  // namespace mv::tool::detail
