/**
 * @file foxglove_sink.hpp
 * @brief Foxglove 可视化调试入口（PImpl 门面）
 *
 * 【职责】
 *   聚合五个子发布器：图像、检测结果、PnP 调试、TF 坐标系、线程健康，
 *   通过单一 FoxgloveSink 对象向 Foxglove Studio 推送所有调试数据。
 *
 * 【PImpl 设计】
 *   - foxglove SDK 头文件（channel.hpp / schemas.hpp / server.hpp）
 *     完全隔离在 foxglove_sink.cpp 与 detail/ 各子模块中；
 *   - 本头文件仅依赖标准库、OpenCV 基础类型、Eigen、nlohmann_json
 *     和 interfaces/types.hpp，上游目标零 SDK 编译代价。
 *
 * 【典型用法】
 * @code
 *   mv::tool::FoxgloveSink sink;          // 默认 0.0.0.0:8765
 *   sink.Start();
 *
 *   // 主循环
 *   sink.PublishImage(frame, "camera/raw");
 *   sink.PublishDetections(detections);
 *   sink.PublishPnpResult(detections, frame);
 *   sink.PublishTransform("world", "gimbal", T_wg, ts_ns);
 *   sink.PublishThreadMetrics(metrics);
 *   sink.PublishGimbalControl(ctrl);
 * @endcode
 */
#pragma once

#include "interfaces/types.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

namespace mv::tool {

// ── 前向声明（供 thread_monitor.hpp 使用）────────────────────────────────────
struct FoxgloveSinkConfig {
  std::string name = "MiracleVision";  ///< 显示在 Foxglove 中的服务名称
  std::string host = "0.0.0.0";        ///< 监听地址
  uint16_t port = 8765;                ///< WebSocket 端口

  // ── 图像发布优化选项 ──────────────────────────────────────────────────────
  /**
   * @brief 使用 JPEG 压缩图像（foxglove.CompressedImage），默认 false（原始 BGR8）
   *
   * 启用后图像数据量可降低 50~100 倍，WebSocket 占用从 ~300MB/s 降至 ~3MB/s，
   * 可显著减少主循环因网络发送产生的阻塞延迟，提升处理帧率。
   */
  bool use_jpeg = false;

  /** @brief JPEG 编码质量 [1, 100]，默认 80（画质/带宽平衡值） */
  int jpeg_quality = 80;

  /**
   * @brief 发布到 Foxglove 时的目标宽度（像素），0 = 保持原始分辨率
   *
   * 例如设置 640 配合 publish_height=480 可将 1280×720 图像缩至四分之一面积，
   * 进一步降低编码和传输耗时。仅在 use_jpeg=true 时生效。
   */
  int publish_width = 0;

  /** @brief 发布到 Foxglove 时的目标高度（像素），0 = 保持原始分辨率 */
  int publish_height = 0;

  // ── 装甲板可视化尺寸（用于 PnpVisualizer 3D 面板渲染，单位：m）────────────
  /** @brief 小装甲板半宽，默认与 vision.yaml armor.small_half_w 一致 */
  double armor_small_half_w = 0.0675;
  /** @brief 大装甲板半宽，默认与 vision.yaml armor.big_half_w 一致 */
  double armor_big_half_w = 0.115;
  /** @brief 装甲板半高，大小装甲通用，默认与 vision.yaml armor.half_h 一致 */
  double armor_half_h = 0.0275;
};

/**
 * @brief Foxglove 数据发布入口
 *
 * 线程安全性：各 Publish* 方法内部加锁，可从 Pipeline 不同线程并发调用。
 */
class FoxgloveSink {
 public:
  // 为就地公开 API 和内部子模块使用
  using Config = FoxgloveSinkConfig;

  struct TraditionalVisionLightVisParams {
    float min_light_ratio{0.07F};
    float max_light_ratio{0.95F};
    float max_light_angle{40.0F};
    float min_area{10.0F};
  };

  // ── 线程健康指标（由 PipelineNode 填充并上报）─────────────────────────

  struct ThreadMetrics {
    std::string node_name;   ///< 节点名称（"CaptureNode" 等）
    double fps{0.0};         ///< 当前处理帧率
    double latency_ms{0.0};  ///< 本节点平均处理延迟（ms）
    uint64_t drop_count{0};  ///< 累计丢帧数
    bool is_alive{true};     ///< 线程是否仍在运行
    std::string error_msg;   ///< 最近一条错误信息（空=无错）
  };

  // ── 参数双向调节回调 ─────────────────────────────────────────────────────

  /** Foxglove 端修改参数时触发：(参数名, 新值 JSON) → void */
  using ParameterCallback = std::function<void(const std::string&, const nlohmann::json&)>;

  // ── 生命周期 ──────────────────────────────────────────────────────────────

  FoxgloveSink();                            ///< 使用默认配置（host=0.0.0.0, port=8765）
  explicit FoxgloveSink(const Config& cfg);  ///< 使用自定义配置
  ~FoxgloveSink();

  FoxgloveSink(const FoxgloveSink&) = delete;
  FoxgloveSink& operator=(const FoxgloveSink&) = delete;
  FoxgloveSink(FoxgloveSink&&) noexcept;
  FoxgloveSink& operator=(FoxgloveSink&&) noexcept;

  /** 启动 WebSocket Server，开始接受客户端连接 */
  void Start();
  /** 停止 Server，断开所有客户端 */
  void Stop();

  /**
   * @brief 是否有客户端当前连接（无锁读，O(1)）
   *
   * 可在发布前用于外部门控，避免自行序列化昂贵数据：
   * @code
   *   if (sink.HasClients()) {
   *       auto debug_img = DrawDebug(frame, dets);
   *       sink.PublishImage(debug_img, "camera/debug");
   *   }
   * @endcode
   * @note PublishImage / PublishPnpResult 内部已自动门控，
   *       只有需要在外部避免绘图开销时才需要手动调用本方法。
   */
  [[nodiscard]] bool HasClients() const noexcept;

  // ── 图像发布 ──────────────────────────────────────────────────────────────

  /**
   * @brief 发布图像帧
   * @param img      OpenCV Mat（BGR8 / MONO8 / 16UC1 自动识别编码）
   * @param topic    Topic 名称，如 "camera/raw"、"camera/debug"
   * @param frame_id 坐标系名称（默认 "camera"）
   * @param ts_ns    时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishImage(const cv::Mat& img, const std::string& topic,
                    const std::string& frame_id = "camera", int64_t ts_ns = 0);

  // ── 检测结果发布 ──────────────────────────────────────────────────────────

  /**
   * @brief 发布装甲板检测结果
   *
   * 同时推送：
   * - `detections/annotations`（ImageAnnotations）：2D 角点多边形 + 标签
   * - `detections/3d`（SceneUpdate）：已解算装甲板在云台坐标系中的 3D 立方体
   *
   * @param dets     检测结果列表（含未解算和已解算的）
   * @param ts_ns    时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishDetections(const std::vector<mv::Detection>& dets, int64_t ts_ns = 0);

  // ── PnP 专项调试 ──────────────────────────────────────────────────────────

  /**
   * @brief 发布 PnP 解算调试信息（三层）
   *
   * - `pnp/debug_image`（RawImage）：底图 + 原始角点（绿圈）标注
   * - `pnp/axes_3d`（SceneUpdate）：每个已解算装甲板的 RGB XYZ 坐标轴箭头
   * - `pnp/residuals`（JSON）：每块装甲板的位姿和深度信息
   *
   * @param dets     检测结果（仅 is_solved=true 的会产生 3D 输出）
   * @param frame    底图（用于 debug_image 绘制）
   * @param ts_ns    时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishPnpResult(const std::vector<mv::Detection>& dets, const cv::Mat& frame,
                        int64_t ts_ns = 0);

  // ── 传统视觉调试图（第一阶段）────────────────────────────────────────────

  /**
   * @brief 发布传统视觉中间图（diff / binary / lights）到 Foxglove
   *
   * 发布 topic：
   * - vision/debug/diff（RawImage/mono8）
   * - vision/debug/binary（RawImage/mono8）
   * - vision/debug/lights（RawImage/bgr8）
   *
   * 当检测器在 ROI 子图运行时，会按 roi_rect 自动还原到全图尺寸再发布。
   *
   * @param diff       双通道差分图（可为空）
   * @param binary     形态学二值图（可为空）
   * @param roi_rect   当前 ROI（全图坐标；area==0 表示全图）
   * @param frame_size 全图尺寸
   * @param raw_frame  当前原始彩色帧（用于绘制 lights 色码图）
   * @param light_vis_params 灯条色码图阈值参数
   * @param ts_ns      时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishTraditionalVisionDebug(const cv::Mat& diff, const cv::Mat& binary,
                                     const cv::Rect2i& roi_rect, const cv::Size& frame_size,
                                     const cv::Mat& raw_frame,
                                     const TraditionalVisionLightVisParams& light_vis_params,
                                     int64_t ts_ns = 0);

  // ── HUD 状态同步（TerminalHUD 远程观看）─────────────────────────────────

  /**
   * @brief 发布 HUD 状态到 Foxglove（FPS、检测数、跟踪、串口连接状态）
   *
   * 将实机终端 HUD 的关键状态同步到 Foxglove Studio，使远程观众能实时监测
   * 算法性能指标与通信状态。
   *
   * Topic: hud/status（JSON）
   * 内容示例：
   * {
   *   "timestamp_us": 1234567890,
   *   "fps": 143.5,
   *   "detection_count": 2,
   *   "tracking": true,
   *   "serial_alive": true,
   *   "enemy_color": "blue",
   *   "target_yaw_deg": -3.2,
   *   "target_pitch_deg": 1.1,
   *   "target_distance_m": 4.20
   * }
   *
   * @note TerminalHUD 本身继续独立运行（文本输出），此接口仅发送数据到 Foxglove，
   *       两者并行不冲突。
   *
   * @param fps              当前算法帧率
   * @param detection_count  当前检测数（无检测时为 0）
   * @param tracking         是否已锁定跟踪
   * @param serial_alive     串口是否连接
   * @param enemy_color      敌方颜色（"red" / "blue" / "none"）
   * @param target_yaw_deg   目标云台 YAW（度）
   * @param target_pitch_deg 目标云台 PITCH（度）
   * @param target_distance_m 目标距离（米）
   * @param ts_ns            时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishHudStatus(double fps, int detection_count, bool tracking, bool serial_alive,
                        const std::string& enemy_color, double target_yaw_deg,
                        double target_pitch_deg, double target_distance_m, int64_t ts_ns = 0);

  // ── TF 坐标系 ─────────────────────────────────────────────────────────────

  /**
   * @brief 发布坐标系变换到 Foxglove /tf topic
   *
   * 在 Foxglove 3D 面板中可直观看到各坐标系相对关系。
   * 建议调用：
   *   - ("world", "gimbal", T_wg)：每帧从预测器推送
   *   - ("gimbal", "camera", T_gc)：Init() 时发布一次（固定外参）
   *
   * @param parent   父坐标系名称
   * @param child    子坐标系名称
   * @param T        4×4 齐次变换矩阵（parent→child）
   * @param ts_ns    时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishTransform(const std::string& parent, const std::string& child,
                        const Eigen::Matrix4d& transform, int64_t ts_ns = 0);

  // ── 线程健康监控 ──────────────────────────────────────────────────────────

  /**
   * @brief 发布 Pipeline 各节点健康指标
   *
   * 推送到 `pipeline/nodes`（JSON），可在 Foxglove 原始 JSON 面板查看。
   *
   * 建议由 VisionPipeline 定时（100ms）汇聚各节点的 ThreadMetrics 后调用。
   *
   * @param metrics  各节点指标列表
   * @param ts_ns    时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishThreadMetrics(const std::vector<ThreadMetrics>& metrics, int64_t ts_ns = 0);

  // ── 云台控制指令 ──────────────────────────────────────────────────────────

  /**
   * @brief 发布云台控制量到 `control/gimbal`（JSON）
   * @param ctrl   云台指令
   * @param ts_ns  时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishGimbalControl(const mv::GimbalControl& ctrl, int64_t ts_ns = 0);

  // ── 调试 / 预测跟踪专项 ───────────────────────────────────────────────────

  /**
   * @brief 发布任意 JSON 数据到指定 Topic（首次调用时懒创建 Channel）
   *
   * 适用于调试面板中的数值展示，如 tracking/state、voter/decision。
   * Foxglove Studio 中使用 "Raw Message" 面板订阅即可查看。
   *
   * @param topic   Topic 名称
   * @param data    nlohmann::json 对象（含任意嵌套结构）
   * @param ts_ns   时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishJson(const std::string& topic, const nlohmann::json& data, int64_t ts_ns = 0);

  /**
   * @brief 发布预测跟踪三层 3D 可视化
   *
   * 同时推送三个 SceneUpdate Topic：
   *   - tracking/armor_positions : 所有装甲板预测位置（主板=黄球，侧板=橙球）
   *   - tracking/rotation_center : 旋转中心（蓝球）+ 速度方向箭头（绿）
   *   - tracking/aim_point       : 弹道求解最终瞄准点（绿球）
   *
   * 无跟踪时（target.is_tracking == false）推送空实体，清除上一帧残影。
   *
   * @param target    IPredictor::GetTrackTarget() 输出（armor_positions 已由
   *                  EkfPredictor 填充；SimplePredictor 时该字段为空）
   * @param aim_xyz   最终瞄准点坐标（世界坐标系，m）
   * @param frame_id  坐标系名称（默认 "world"）
   * @param ts_ns     时间戳（纳秒，0 = 使用当前时间）
   */
  void PublishTrackingVisuals(const mv::TrackTarget& target, const Eigen::Vector3d& aim_xyz,
                              const std::string& frame_id = "world", int64_t ts_ns = 0);

  // ── 参数双向调节 ──────────────────────────────────────────────────────────

  /** 注册参数修改回调（Foxglove 端修改参数时被调用）*/
  void SetParameterCallback(ParameterCallback callback);

  /**
   * @brief 向 Foxglove 推送当前参数快照（初始化或值变更时调用）
   * @param params  JSON 对象，key = 参数名，value = 参数值
   */
  void UpdateParameters(const nlohmann::json& params);

  /** 查询本地缓存的参数值（应用层主动拉取）*/
  [[nodiscard]] nlohmann::json GetParameter(const std::string& name) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::tool
