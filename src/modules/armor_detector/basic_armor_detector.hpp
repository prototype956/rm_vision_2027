/**
 * @file basic_armor_detector.hpp
 * @brief 传统视觉装甲板检测器（灯条轮廓法）
 *
 * 【算法流程】
 *   1. 灰度阈值掩码（含白色过曝中心）+ 双通道差值颜色掩码取交集 → 实心灯条二值图；
 *   2. findContours → fitEllipse 求倾斜角 → 按面积/宽高比/倾斜角过滤灯条；
 *   3. 两两配对：检查角度差、高度比、装甲宽高比；
 *   4. 输出 Detection（2D 四角点 + 置信度 = 1.0，3D 部分由 ISolver 填充）。
 *
 * 【YAML 配置字段（均有默认值，可不提供）】
 * @code
 *   detector:
 *     light_thresh:    160    # 灰度亮度阈值（所有亮像素，含白色中心）
 *     green_thresh:     30    # 双通道差值阈值（R-B AND R-G，识别灯条彩色边缘）
 *     white_thresh:    200    # 预留（当前未使用）
 *     min_light_ratio: 0.07   # 灯条短/长轴最小比（原项目 RATIO_W_H_MAX=15 → 1/15≈0.07）
 *     max_light_ratio: 0.95   # 灯条短/长轴最大比（原项目 RATIO_W_H_MIN=1  → 1/1 =1.0）
 *     max_light_angle: 40.0   # 灯条允许的最大倾斜度（相对垂直，°）
 *     min_armor_ratio: 1.0    # 装甲板宽高比下界（档板间距 / 平均灯条长）
 *     max_armor_ratio: 5.5    # 装甲板宽高比上界
 *     max_angle_diff:  8.0    # 两灯条角度之差上界（°）
 *     min_area:        10.0   # 灯条轮廓最小面积（px²，过滤噪点）
 * @endcode
 *
 * 工厂键：`"basic"`
 *
 * 【设计模式】
 *   采用 Pimpl（impl 指针）模式将所有私有实现细节（数据成员、内部算法函数、
 *   LightBar 结构体）完全隐藏在 .cpp 中，头文件仅暴露公开接口与公开类型，
 *   消除上游编译对 OpenCV 内部类型的直接依赖。
 *
 * 【ROI 自适应裁剪】
 *   ROI 状态机已迁移至 RoiManager（roi_manager.hpp），BasicArmorDetector
 *   现在是纯无状态检测器：每次 Detect(frame) 接受任意输入（全图或裁剪片），
 *   返回相对于该输入的坐标，坐标恢复由调用方（RoiManager）负责。
 */
#pragma once

#include "factory/factory.hpp"
#include "interfaces/i_detector.hpp"

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

namespace mv::modules {

class BasicArmorDetector final : public IDetector {
 public:
  BasicArmorDetector();
  ~BasicArmorDetector() override;

  BasicArmorDetector(const BasicArmorDetector&) = delete;
  BasicArmorDetector& operator=(const BasicArmorDetector&) = delete;
  BasicArmorDetector(BasicArmorDetector&&) noexcept;
  BasicArmorDetector& operator=(BasicArmorDetector&&) noexcept;

  // ── IDetector 接口实现 ────────────────────────────────────────────────────

  bool Init(const YAML::Node& config) override;

  [[nodiscard]] std::vector<Detection> Detect(const cv::Mat& frame,
                                              ArmorColor enemy_color) override;

  [[nodiscard]] bool IsInitialized() const noexcept override;

  // ── 可热调参数（调试工具读写）────────────────────────────────────────────

  /** 所有可调参数聚合在此，便于 SetParams/GetParams 原子传递 */
  struct Params {
    int light_thresh{160};  ///< 灰度亮度二值阈值（捕获所有亮像素含白色中心）
    int green_thresh{30};  ///< 双通道差值阈值（识别彩色边缘；R-B AND R-G 统一用此值）
    int white_thresh{200};         ///< 预留：灰度硬屏蔽阈值（当前未启用）
    float min_light_ratio{0.07F};  ///< 灯条短/长轴比下界，对应原项目 LIGHT_RATIO_W_H_MAX=15
    float max_light_ratio{0.95F};  ///< 灯条短/长轴比上界，对应原项目 LIGHT_RATIO_W_H_MIN=1
    float max_light_angle{40.0F};  ///< 灯条最大倾斜角（°，0=垂直，90=水平）
    float min_armor_ratio{1.0F};   ///< 装甲宽高比下界
    float max_armor_ratio{5.5F};   ///< 装甲宽高比上界
    float max_angle_diff{8.0F};    ///< 两灯条倾斜角最大差值（°）
    float min_area{10.0F};         ///< 灯条轮廓最小面积（px²）
  };

  void SetParams(const Params& params) noexcept;
  [[nodiscard]] const Params& GetParams() const noexcept;

  // ── 调试中间数据（仅 EnableDebug(true) 后有效）──────────────────────────

  /**
   * 每帧 Detect() 的中间图像，供 tool/debug 的可视化工具消费。
   *
   * lights_vis 已移出：由 mv::tool::PaintLightBarsVis(binary, frame, params)
   * 在 tool/debug 模块按需生成，与检测逻辑解耦。
   */
  struct DebugData {
    cv::Mat diff;    ///< 双通道差值颜色掩码（灰度单通道）
    cv::Mat binary;  ///< 形态学处理后的二值图（相对于传入帧，可能是 ROI 片）
  };

  void EnableDebug(bool enabled) noexcept;
  [[nodiscard]] const DebugData& GetDebugData() const noexcept;

 private:
  /** Pimpl：所有私有实现细节（数据成员、LightBar、算法辅助函数）在 .cpp 中定义 */
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::modules
