#pragma once

#include "modules/armor_pnp/armor_pnp_config.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"

#include <memory>

#include <span>

namespace mv::modules {

/**
 * @brief 使用 IPPE 解算装甲位姿，并维护仿真真值基准与检测精度累计统计。
 *
 * 类本身不执行角点精修、滤波或跟踪。ProcessFrame() 消费同帧精修结果，并让真值投影
 * 与正式检测共用同一个 IPPE 求解器；累计统计每 100 帧生成一次原子快照。
 */
class ArmorPnp final {
 public:
  /** @brief 保存已校验的不可变 PnP 参数并创建空统计状态。 */
  explicit ArmorPnp(ArmorPnpConfig config);
  ~ArmorPnp();

  ArmorPnp(const ArmorPnp& other);
  ArmorPnp& operator=(const ArmorPnp& other);
  ArmorPnp(ArmorPnp&& other) noexcept;
  ArmorPnp& operator=(ArmorPnp&& other) noexcept;

  /**
   * @brief 对 TL、TR、BR、BL 顺序的四个图像点运行 IPPE 并选择有效最小 RMSE 候选。
   * @param image_corners TL、TR、BR、BL 顺序的二维输入角点。
   * @param type 决定物点宽度的大、小装甲类型。
   * @param calibration 与输入角点同帧的相机内参与畸变参数。
   * @param source 输入角点来源。
   * @param input_index 来源数组中的索引。
   * @param label 检测或真值标签的数值表示。
   */
  [[nodiscard]] ArmorPnpAttempt Solve(std::span<const cv::Point2f, 4> image_corners,
                                      hal::CameraFrame::ArmorType type,
                                      const hal::CameraFrame::Calibration& calibration,
                                      PnpInputSource source, std::size_t input_index,
                                      std::uint8_t label = 0) const;

  /**
   * @brief 运行同帧真值基准链和正式检测链，并更新累计统计。
   *
   * frame 缺少 geometry 或 refinements 与 detections 数量不一致时返回空结果且不更新统计。
   *
   * @param frame 提供标定、坐标变换、仿真真值和帧序号的相机帧。
   * @param detections 当前帧装甲检测结果。
   * @param refinements 与 detections 按索引一一对应的角点精修结果。
   */
  [[nodiscard]] ArmorPnpFrameResult ProcessFrame(
      const hal::CameraFrame& frame, std::span<const ArmorDetection> detections,
      std::span<const CornerRefinementResult> refinements);

 private:
  struct Impl;                  ///< 隔离 OpenCV 求解器、真值评估和累计统计实现。
  std::unique_ptr<Impl> impl_;  ///< 当前 PnP 实例唯一拥有的可复制实现状态。
};

}  // namespace mv::modules
