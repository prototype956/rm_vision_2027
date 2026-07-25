/**
 * @file traditional_vision_debug_publisher.hpp
 * @brief 传统视觉调试图发布子模块（第三阶段：diff / binary / lights / roi）
 *
 * 【Topic 说明（Foxglove 使用方法）】
 *
 * 1) vision/debug/diff  (RawImage/mono8)
 *    用途：观察双通道差分结果，判断颜色阈值是否过严或过松。
 *    在 Foxglove 中：添加 Image 面板并订阅该 topic。
 *
 * 2) vision/debug/binary (RawImage/mono8)
 *    用途：观察形态学后的二值图，判断亮度阈值与形态学参数是否合理。
 *    在 Foxglove 中：添加 Image 面板并订阅该 topic。
 *
 * 3) vision/debug/lights (RawImage/bgr8)
 *    用途：保留终端调试中的灯条颜色提示，辅助调参。
 *    颜色语义：
 *      绿色 = 通过过滤；橙色 = 宽高比不符；紫色 = 倾角超限。
 *    在 Foxglove 中：添加 Image 面板并订阅该 topic。
 *
 * 4) vision/debug/roi (RawImage/bgr8)
 *    用途：观察 ROI 状态机是否正确锁定与回退。
 *    显示语义：
 *      ROI 激活时绘制黄色矩形框并标注 "ROI"；
 *      ROI 未激活时左上角显示 "ROI: FULL FRAME"。
 *    在 Foxglove 中：添加 Image 面板并订阅该 topic。
 *
 * 【输入约定】
 *   - diff 和 binary 允许为空；为空时对应 topic 不发布。
 *   - 当检测在 ROI 子图上运行时，调用方传入 roi_rect + frame_size，
 *     本模块会自动还原到全图尺寸后再发布，便于与 camera/raw 对齐显示。
 *   - lights 图由 binary + raw_frame + LightVisParams 共同生成。
 *
 * 【边界说明】
 *   - 当前阶段已包含 ROI 叠加图，可直接用于观察裁剪行为。
 *   - 参数热调与配置写回由后续阶段实现。
 */
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>

#include <foxglove/context.hpp>
#include <opencv2/core.hpp>

namespace mv::tool::detail {

class ImagePublisher;

class TraditionalVisionDebugPublisher {
 public:
  struct LightVisParams {
    float min_light_ratio{0.07F};
    float max_light_ratio{0.95F};
    float max_light_angle{40.0F};
    float min_area{10.0F};
  };

  explicit TraditionalVisionDebugPublisher(foxglove::Context ctx);
  ~TraditionalVisionDebugPublisher();

  TraditionalVisionDebugPublisher(const TraditionalVisionDebugPublisher&) = delete;
  TraditionalVisionDebugPublisher& operator=(const TraditionalVisionDebugPublisher&) = delete;
  TraditionalVisionDebugPublisher(TraditionalVisionDebugPublisher&&) = delete;
  TraditionalVisionDebugPublisher& operator=(TraditionalVisionDebugPublisher&&) = delete;

  /**
   * @brief 发布传统视觉调试图（diff / binary / lights / roi）
   *
   * @param diff       双通道差分图（通常为单通道 8bit）
   * @param binary     形态学后的二值图（通常为单通道 8bit）
   * @param roi_rect   当前 ROI（全图坐标；area==0 表示全图）
   * @param frame_size 全图尺寸
   * @param raw_frame  当前原始彩色帧（用于绘制 lights 色码图）
   * @param light_params 灯条色码图阈值参数
   * @param ts_ns      时间戳（纳秒）
   */
  void Publish(const cv::Mat& diff, const cv::Mat& binary, const cv::Rect2i& roi_rect,
               const cv::Size& frame_size, const cv::Mat& raw_frame,
               const LightVisParams& light_params, uint64_t ts_ns);

 private:
  /** @brief 将 ROI 局部图还原到全图尺寸（若本身是全图则直接返回原图） */
  [[nodiscard]] static cv::Mat RestoreToFullFrame(const cv::Mat& src, const cv::Rect2i& roi_rect,
                                                  const cv::Size& frame_size);

  /** @brief 按过滤原因绘制灯条色码图（绿色通过、橙色比例失败、紫色角度失败） */
  [[nodiscard]] static cv::Mat BuildLightsVisualization(const cv::Mat& binary_full,
                                                        const cv::Mat& raw_frame,
                                                        const LightVisParams& light_params);

  /** @brief 绘制 ROI 可视化图（激活时画框，未激活时显示 FULL FRAME） */
  [[nodiscard]] static cv::Mat BuildRoiVisualization(const cv::Mat& raw_frame,
                                                     const cv::Rect2i& roi_rect);

  foxglove::Context ctx_;
  std::unique_ptr<ImagePublisher> image_pub_;
  std::mutex mtx_;
};

}  // namespace mv::tool::detail
