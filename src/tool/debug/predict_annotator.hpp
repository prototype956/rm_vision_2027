/**
 * @file predict_annotator.hpp
 * @brief 预测调试视图注解工具（header-only）
 *
 * 职责：
 *   - ArmorNumberStr()：mv::ArmorNumber 枚举 → 可读字符串
 *   - MakeDetLabel()  ：单条检测结果 → 含 PnP 距离/重投影误差的标签字符串
 *   - DrawAnnotated() ：将检测框、重投影、EKF 状态栏、开火许可叠加到帧上
 *   - SimGimbalQuaternion() / WorldToGimbalTF()：模拟云台姿态辅助
 *   - ThrottlePlayback()：按目标帧率 sleep 节流
 *
 * 设计：
 *   header-only，无编译单元。
 *   仅依赖 interfaces/types.hpp 和 OpenCV / Eigen，
 *   不引入 foxglove SDK 或 yaml-cpp。
 */
#pragma once

#include "interfaces/types.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::tool {

// ============================================================================
// ArmorNumber 工具
// ============================================================================

/** @brief ArmorNumber 枚举 → 可读字符串（"1"~"5" / "S" / "OP" / "BS"）*/
inline std::string ArmorNumberStr(mv::ArmorNumber n) {
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
}

/**
 * @brief 生成单条检测结果的显示标签
 *
 * 格式（已解算）：`R-2 1.23m err:0.5px`
 * 格式（未解算）：`R-2 noPnP`
 */
inline std::string MakeDetLabel(const mv::Detection& det) {
  const std::string prefix =
      ((det.color == mv::ArmorColor::RED) ? "R-" : "B-") + ArmorNumberStr(det.number);
  if (det.is_solved) {
    char buf[80];
    std::snprintf(buf, sizeof(buf), "%s %.2fm err:%.1fpx", prefix.c_str(), det.xyz_in_gimbal.norm(),
                  det.reproj_error);
    return buf;
  }
  return prefix + " noPnP";
}

// ============================================================================
// DrawAnnotated
// ============================================================================

/**
 * @brief 在帧上叠加检测框、重投影点、EKF 状态栏和开火许可指示
 *
 * 叠加内容：
 *   - 检测角点（绿色多边形）
 *   - 重投影角点（青色：主 PnP 解；橙色：IPPE 第二候选）
 *   - 中心点（颜色：红/蓝对应敌方颜色）
 *   - 标签（MakeDetLabel()）
 *   - 状态栏第一行：检测数、已解算数、FPS、云台 yaw/pitch、是否跟踪
 *   - 状态栏第二行：EKF 状态、装甲板编号、开火许可（绿字/灰字）
 *
 * @param frame         原始帧（可为 MONO8 或 BGR8）
 * @param dets          本帧检测结果
 * @param ctrl          EKF 预测输出的云台控制量
 * @param target        EKF 跟踪目标状态
 * @param fire_permitted Voter 开火许可
 * @param fps           当前帧率（用于状态栏显示）
 * @return              叠加注解后的 BGR8 图像（不修改原始帧）
 */
inline cv::Mat DrawAnnotated(const cv::Mat& frame, const std::vector<mv::Detection>& dets,
                             const mv::GimbalControl& ctrl, const mv::TrackTarget& target,
                             bool fire_permitted, double fps) {
  cv::Mat img;
  if (frame.channels() == 1) {
    cv::cvtColor(frame, img, cv::COLOR_GRAY2BGR);
  } else {
    img = frame.clone();
  }

  int solved_count = 0;
  for (const auto& det : dets) {
    if (det.is_solved)
      ++solved_count;
  }

  // ── 检测框与标签 ──────────────────────────────────────────────────────────
  for (const auto& det : dets) {
    const cv::Scalar BOX_CLR =
        (det.color == mv::ArmorColor::RED) ? cv::Scalar(0, 0, 220) : cv::Scalar(220, 80, 0);

    // 检测角点（绿色框）
    std::vector<cv::Point> poly;
    for (const auto& pt : det.points) {
      poly.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
    }
    cv::polylines(img, poly, true, cv::Scalar(0, 215, 0), 2, cv::LINE_AA);

    // 重投影角点（青色，主 PnP 解）
    if (det.is_solved) {
      std::vector<cv::Point> rp;
      for (const auto& pt : det.reprojected_points) {
        rp.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
      }
      cv::polylines(img, rp, true, cv::Scalar(255, 220, 0), 1, cv::LINE_AA);
    }

    // 重投影角点（橙色，IPPE 第二候选解）
    if (det.has_alt_solution) {
      std::vector<cv::Point> ap;
      for (const auto& pt : det.reprojected_points_alt) {
        ap.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
      }
      cv::polylines(img, ap, true, cv::Scalar(0, 130, 255), 1, cv::LINE_AA);
    }

    // 中心点
    const cv::Point2f CTR = det.Center();
    cv::circle(img, cv::Point(static_cast<int>(CTR.x), static_cast<int>(CTR.y)), 5, BOX_CLR, -1);

    // 标签
    cv::putText(img, MakeDetLabel(det),
                cv::Point(static_cast<int>(CTR.x) + 6, static_cast<int>(CTR.y) - 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, BOX_CLR, 1, cv::LINE_AA);
  }

  // ── 状态栏第一行：检测 + FPS 概览 ────────────────────────────────────────
  {
    constexpr double RAD2DEG = 180.0 / M_PI;
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "DET:%zu SLV:%d FPS:%.0f | yaw:%.1f\xC2\xB0 pit:%.1f\xC2\xB0 trk:%s", dets.size(),
                  solved_count, fps, ctrl.yaw * RAD2DEG, ctrl.pitch * RAD2DEG,
                  ctrl.tracking ? "Y" : "N");
    cv::rectangle(img, cv::Point(0, 0), cv::Point(img.cols, 32), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(img, buf, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 240, 0),
                1, cv::LINE_AA);
  }

  // ── 状态栏第二行：EKF 跟踪状态 + Voter 决策 ──────────────────────────────
  {
    const cv::Scalar FIRE_CLR = fire_permitted ? cv::Scalar(0, 80, 255) : cv::Scalar(80, 80, 80);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "EKF:%s No.%s | FIRE:%s", target.tracker_state.c_str(),
                  ArmorNumberStr(target.number).c_str(), fire_permitted ? "YES" : " NO");
    cv::rectangle(img, cv::Point(0, 32), cv::Point(img.cols, 62), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(img, buf, cv::Point(8, 54), cv::FONT_HERSHEY_SIMPLEX, 0.50, FIRE_CLR, 1,
                cv::LINE_AA);
  }

  return img;
}

// ============================================================================
// 模拟云台辅助
// ============================================================================

/** @brief 从模拟 yaw/pitch/roll 构造云台四元数（ZYX 欧拉角顺序）*/
inline Eigen::Quaterniond SimGimbalQuaternion(double yaw_deg, double pitch_deg, double roll_deg) {
  constexpr double DEG2RAD = M_PI / 180.0;
  return Eigen::AngleAxisd(yaw_deg * DEG2RAD, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch_deg * DEG2RAD, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll_deg * DEG2RAD, Eigen::Vector3d::UnitX());
}

/**
 * @brief 根据云台 yaw/pitch 构造 world → gimbal 的 4×4 变换矩阵（用于 /tf 可视化）
 *
 * 约定：world 为 Z-up 右手系，gimbal 为 Z-forward（OpenCV 惯例）。
 * 固定基底旋转 R_FIX（绕 X 轴 -90°）使 Foxglove 3D 面板中"正前方"对应 Z 轴正向。
 */
inline Eigen::Matrix4d WorldToGimbalTF(double yaw_rad, double pitch_rad) {
  static const Eigen::Matrix3d R_FIX =
      Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = (Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(-pitch_rad, Eigen::Vector3d::UnitY()))
                            .toRotationMatrix() *
                        R_FIX;
  return T;
}

// ============================================================================
// EKF 预测装甲板重投影
// ============================================================================

/**
 * @brief 将 EKF 预测的所有装甲板重投影到图像上（就地修改 img）
 *
 * 坐标变换链：
 *   世界系（= 云台系，sim 角度=0 时） → 相机系（R_c2g.T * pt） → 像素（K, D）
 *
 * 绘制内容：
 *   - 带编号的彩色矩形框（对应 target.armor_positions 中每块板子）
 *   - 中心十字（始终显示，即使角点出视野时也可见）
 *   - 右上角 "EKF yaw_predicted" 文字标注
 *
 * @param img        待叠加图像（BGR8，会被就地修改）
 * @param target     EKF 跟踪目标状态（需包含 armor_positions）
 * @param K          3×3 相机内参矩阵（CV_64F）
 * @param D          畸变系数（1×N，CV_64F）
 * @param R_c2g      相机→云台旋转矩阵 3×3（Eigen::Matrix3d）；
 *                   将云台坐标系到相机坐标系的变换为 R_c2g.transpose()
 * @param t_c2g      相机→云台平移向量（m），通常为零向量
 * @param armor_half_w  装甲板半宽（m），默认小装甲 0.0675m
 * @param armor_half_h  装甲板半高（m），默认 0.0275m
 */
inline void DrawPredictedArmors(cv::Mat& img, const mv::TrackTarget& target, const cv::Mat& K,
                                const cv::Mat& D, const Eigen::Matrix3d& R_c2g,
                                const Eigen::Vector3d& t_c2g = Eigen::Vector3d::Zero(),
                                float armor_half_w = 0.0675f, float armor_half_h = 0.0275f) {
  if (!target.is_tracking || target.armor_positions.empty())
    return;

  // 每块装甲板的颜色（BGR）
  static const std::array<cv::Scalar, 4> kColors = {
      cv::Scalar{0, 230, 230},  // 0: 黄色
      cv::Scalar{255, 140, 0},  // 1: 天蓝色
      cv::Scalar{0, 255, 100},  // 2: 亮绿
      cv::Scalar{200, 0, 255},  // 3: 紫色
  };

  // R_gimbal_to_camera = R_c2g.transpose()
  const Eigen::Matrix3d R_g2c = R_c2g.transpose();

  for (size_t i = 0; i < target.armor_positions.size(); ++i) {
    const Eigen::Vector4d& ap = target.armor_positions[i];
    const double gx = ap.x();   // gimbal X (right)
    const double gy = ap.y();   // gimbal Y (up/vertical)
    const double gz = ap.z();   // gimbal Z (forward/depth)
    const double yaw = ap.w();  // 装甲板法向 yaw（= atan2(gx, gz) 约定）

    // 装甲板水平切向（垂直于法向在 XZ 平面内的投影）
    // yaw = atan2(gx, gz)  →  法向方向 = (sin(yaw), 0, cos(yaw))
    //                        切向方向 = (cos(yaw), 0, -sin(yaw))
    const Eigen::Vector3d T_h(std::cos(yaw), 0.0, -std::sin(yaw));  // 板面水平切向（gimbal）
    const Eigen::Vector3d T_v(0.0, 1.0, 0.0);                       // 竖直方向（gimbal Y）

    const Eigen::Vector3d center_g(gx, gy, gz);

    // 在云台系中构造 4 个角点：TL / TR / BR / BL（顺时针，facing camera）
    const std::array<Eigen::Vector3d, 4> corners_g = {
        center_g + T_h * armor_half_w + T_v * armor_half_h,  // TL
        center_g - T_h * armor_half_w + T_v * armor_half_h,  // TR
        center_g - T_h * armor_half_w - T_v * armor_half_h,  // BR
        center_g + T_h * armor_half_w - T_v * armor_half_h,  // BL
    };

    // 将 4 角点从云台系转到相机系
    std::vector<cv::Point3f> pts_cam;
    pts_cam.reserve(4);
    bool all_valid = true;
    for (const auto& cg : corners_g) {
      const Eigen::Vector3d cam = R_g2c * (cg - t_c2g);
      if (cam.z() < 0.05) {
        all_valid = false;
        break;
      }
      pts_cam.push_back(
          {static_cast<float>(cam.x()), static_cast<float>(cam.y()), static_cast<float>(cam.z())});
    }

    // 中心点投影（无论角点是否有效都要画）
    const Eigen::Vector3d cam_ctr = R_g2c * (center_g - t_c2g);
    cv::Point2f center_px{-1, -1};
    if (cam_ctr.z() > 0.05) {
      std::vector<cv::Point3f> ctr_3d = {{static_cast<float>(cam_ctr.x()),
                                          static_cast<float>(cam_ctr.y()),
                                          static_cast<float>(cam_ctr.z())}};
      std::vector<cv::Point2f> ctr_2d;
      cv::projectPoints(ctr_3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D, ctr_2d);
      if (!ctr_2d.empty()) {
        center_px = ctr_2d[0];
        // 检查是否在图像内
        const bool in_img = (center_px.x >= 0 && center_px.x < img.cols && center_px.y >= 0 &&
                             center_px.y < img.rows);
        if (in_img) {
          const cv::Scalar clr = kColors[i % kColors.size()];
          const cv::Point cp(static_cast<int>(center_px.x), static_cast<int>(center_px.y));
          cv::drawMarker(img, cp, clr, cv::MARKER_CROSS, 14, 2, cv::LINE_AA);

          // 编号标签
          char label[8];
          std::snprintf(label, sizeof(label), "P%zu", i);
          cv::putText(img, label, cp + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX, 0.45, clr, 1,
                      cv::LINE_AA);
        }
      }
    }

    // 投影 4 个角点并绘制矩形
    if (all_valid) {
      std::vector<cv::Point2f> pts_2d;
      cv::projectPoints(pts_cam, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D, pts_2d);
      if (pts_2d.size() == 4) {
        const cv::Scalar clr = kColors[i % kColors.size()];
        // 检查所有投影点是否在扩展图像边界内
        bool pts_ok = true;
        const int margin = 100;
        for (const auto& p : pts_2d) {
          if (p.x < -margin || p.x > img.cols + margin || p.y < -margin ||
              p.y > img.rows + margin) {
            pts_ok = false;
            break;
          }
        }
        if (pts_ok) {
          for (int k = 0; k < 4; ++k) {
            cv::line(img, pts_2d[k], pts_2d[(k + 1) % 4], clr, 2, cv::LINE_AA);
          }
        }
      }
    }
  }

  // 右上角提示文字
  {
    constexpr double RAD2DEG = 180.0 / M_PI;
    char buf[80];
    std::snprintf(buf, sizeof(buf), "EKF yaw_pred: %.1f deg", target.yaw_predicted * RAD2DEG);
    const int tx = img.cols - 280;
    cv::putText(img, buf, cv::Point(tx > 0 ? tx : 0, 22), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(180, 230, 255), 1, cv::LINE_AA);
  }
}

// ============================================================================
// 播放节流
// ============================================================================

/**
 * @brief 在主循环末尾调用，按目标 playback_fps 进行 sleep 节流
 *
 * 若 target_fps <= 0，则不节流（全速运行）。
 *
 * @param frame_start  本帧开始的时间点
 * @param target_fps   目标帧率（0 = 不限速）
 */
inline void ThrottlePlayback(std::chrono::steady_clock::time_point frame_start, double target_fps) {
  if (target_fps <= 0.0)
    return;
  const double target_dt_us = 1.0e6 / target_fps;
  const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - frame_start)
                              .count();
  const auto sleep_us = static_cast<long long>(target_dt_us) - elapsed_us;
  if (sleep_us > 0) {
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
  }
}

}  // namespace mv::tool
