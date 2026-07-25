/**
 * @file detection_publisher.hpp
 * @brief 装甲板检测结果发布子模块
 *
 * 【发布流程】Publish() 每召一次同时向两个 topic 推送数据：
 *
 *   detections/annotations（ImageAnnotations）
 *     • 为每个装甲板生成一个 LINE_LOOP PointsAnnotation（4 角点）
 *     • 颜色进行 RED/BLUE 分辨，叠加标签文字（ "R-1 120cm" 格式）
 *     • 2D 图像面板→1右键 → Image 面板叠加可见
 *
 *   detections/3d（SceneUpdate）
 *     • 仅 is_solved=true 的装甲板参与。将装甲板表示为物理尺寸（SMALL/BIG）
 *       的 CubePrimitive 放置在云台坐标系
 *     • entity.lifetime = 300ms：若目标抡丢 (300ms 内未更新) 模型自动消失
 *
 * 线程安全：mtx_ 保护 channel 初始化和消息发送。
 */
#pragma once

#include "interfaces/types.hpp"

#include <mutex>
#include <vector>

#include <foxglove/context.hpp>
#include <foxglove/schemas.hpp>
#include <optional>

namespace mv::tool::detail {

class DetectionPublisher {
 public:
  /** @param ctx  foxglove 全局上下文，由 FoxgloveSink::Impl 传入并共享 */
  explicit DetectionPublisher(foxglove::Context ctx);

  /**
   * @brief 发布检测结果
   *
   * 若无任何检测结果，会推送空的 ImageAnnotations 和 SceneUpdate，
   * 确保 Foxglove 客户端加载的旧实体被清除。
   *
   * @param dets    检测结果列表（含未解算和已解算）
   * @param ts_ns   时间戳（纳秒，须已由 ResolveTs 解析）
   */
  void Publish(const std::vector<mv::Detection>& dets, uint64_t ts_ns);

 private:
  /**
   * @brief 懒创建两个频道（首次 Publish 时调用）
   *
   * 两个 channel 分开 advertise 为了让 Foxglove 客户端可以独立订阅
   * 2D 标注或只播放 3D 场景，不强制两个客户端同时订阅。
   */
  void EnsureChannels();

  // ── 成员变量 ─────────────────────────────────────────────────────────────
  foxglove::Context ctx_;                                                   ///< SDK 全局上下文
  std::optional<foxglove::schemas::ImageAnnotationsChannel> annot_ch_;     ///< 2D 标注 channel（懒创建）
  std::optional<foxglove::schemas::SceneUpdateChannel> scene_ch_;          ///< 3D 场景 channel（懒创建）
  std::mutex mtx_;                                                          ///< 保护 channel 初始化和消息发送
};

}  // namespace mv::tool::detail
