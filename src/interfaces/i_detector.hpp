/**
 * @file i_detector.hpp
 * @brief 目标检测器抽象接口 (IDetector)
 *
 * 【职责边界】
 *   IDetector 只负责"从图像找出目标的 2D 信息"：
 *   - 输入：相机帧（cv::Mat）+ 敌方颜色
 *   - 输出：Detection 列表（含角点、置信度、类型）—— 不含 3D 坐标
 *
 *   3D 解算由 ISolver 完成，预测由 IPredictor 完成。
 *   三者串联形成检测→解算→预测的流水线，互不依赖实现。
 *
 * 【实现约定】
 *   - Init() 失败时返回 false，不抛异常；
 *   - Detect() 在无目标或输入帧无效的非异常路径下返回空向量；
 *   - Detect() 在输入帧坐标系下工作，ROI 坐标还原由检测器外部负责；
 *   - 接口非线程安全——调用方负责在同一线程顺序调用，
 *     或将实例隔离到专属检测线程。
 *
 * 【可扩展点】
 *   传统视觉实现：BasicArmorDetector（基于灯条轮廓）
 *   深度学习实现：DnnArmorDetector（YOLOv8 / ONNX Runtime）
 *   单元测试桩：MockDetector（直接返回预设列表）
 */
#pragma once

#include "types.hpp"

#include <vector>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace mv {

class IDetector {
 public:
  // ── 生命周期 ─────────────────────────────────────────────────────────────
  IDetector() = default;
  virtual ~IDetector() = default;

  // 禁止拷贝（内部状态含图像缓冲，拷贝语义不明确）
  IDetector(const IDetector&) = delete;
  IDetector& operator=(const IDetector&) = delete;

 protected:
  // 移动降为 protected —— 派生类可用，外部不允许切片（C.67）
  IDetector(IDetector&&) = default;
  IDetector& operator=(IDetector&&) = default;

 public:
  // ── 核心接口 ─────────────────────────────────────────────────────────────

  /**
   * @brief 初始化检测器（加载模型/读取阈值）
   * @param config  YAML 配置节点，由调用方从 ConfigManager 中取出后传入
   * @return true 成功；false 失败（日志由实现内部记录，不抛异常）
   * @thread_safety Not thread-safe
   *
   * 约定：Init() 失败后对象处于"未初始化"状态，
   * 可以重新调用 Init()；不可以调用 Detect()。
   */
  virtual bool Init(const YAML::Node& config) = 0;

  /**
   * @brief 在图像中检测装甲板
   * @param frame         相机帧（BGR，CV_8UC3，可为全图或 ROI 局部图）
   * @param enemy_color   只返回此颜色的装甲板（Unknown 则返回全部）
   * @return Detection 列表；无目标或输入帧无效的非异常路径返回空向量
   * @thread_safety Not thread-safe
   *
   * 返回的 Detection 中：
   *   - points/box/confidence/color/type/number 已填充；
   *   - points 位于当前输入帧坐标系（input-frame）；
   *   - points 顺序固定为 BL, BR, TR, TL（顺时针，从左下开始）；
   *   - ROI 坐标还原由检测器外部模块负责（不在本接口内）；
   *   - is_solved = false，xyz_in_gimbal = {0,0,0}（需 ISolver 填充）。
   */
  [[nodiscard]] virtual std::vector<Detection> Detect(const cv::Mat& frame,
                                                      ArmorColor enemy_color) = 0;

  /**
   * @return 检测器是否已完成初始化
   * @thread_safety Not thread-safe
   */
  [[nodiscard]] virtual bool IsInitialized() const noexcept = 0;
};

}  // namespace mv
