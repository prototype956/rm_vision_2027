/**
 * @file hud_status_publisher.hpp
 * @brief HUD 状态 JSON 同步子模块（第四阶段：fps / detection / tracking / serial status）
 *
 * 【Purpose】
 *   将终端 HUD（TerminalHUD）的关键状态同步到 Foxglove，使远程观众能实时监测：
 *   - 算法帧率（FPS）与平均延迟
 *   - 当前检测数量
 *   - 跟踪是否锁定
 *   - 串口连接状态
 *   - 目标云台角度 / 距离
 *   - 敌方颜色识别结果
 *
 * 【Topic 说明（Foxglove 使用方法）】
 *
 *   Topic: hud/status (JSON)
 *   用途：实时监测实机工作状态（FPS、检测、跟踪、通信）。
 *   示例内容：
 *   {
 *     "timestamp_us": 1234567890,
 *     "fps": 143.5,
 *     "detection_count": 2,
 *     "tracking": true,
 *     "serial_alive": true,
 *     "enemy_color": "blue",
 *     "target_yaw_deg": -3.2,
 *     "target_pitch_deg": 1.1,
 *     "target_distance_m": 4.20
 *   }
 *
 *   在 Foxglove 中：
 *   1. 添加 JSON state 面板，订阅 hud/status topic
 *   2. 可配置警报규则，如 serial_alive==false 时高亮提示
 *   3. 与 TerminalHUD 并行运行，不相互影响
 *
 * 【与 TerminalHUD 的关系】
 *   - TerminalHUD：直接向终端输出，自动 200ms 节流，用于本地赛场调试
 *   - HudStatusPublisher：向 Foxglove 发送 JSON，由主循环控制节流，用于远程观看
 *   - 两者独立运行，互不替代；调用方需显式调用本发布器
 *
 * 【设计约束】
 *   - 本阶段不涉及参数热调（Foxglove 端修改后推送参数回 c++）
 *   - 不实现配置写回，参数仅在内存中应用
 *   - 所有字段均为只读观测数据，无专用参数接口
 */
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <foxglove/context.hpp>
#include <nlohmann/json.hpp>

namespace mv::tool::detail {

class HudStatusPublisher {
 public:
  /**
   * @brief HUD 状态数据结构
   *
   * 包含算法帧率、检测数、跟踪状态、通信状态等关键指标。
   */
  struct HudStatus {
    uint64_t timestamp_us{0};         ///< 时间戳（微秒）
    double fps{0.0};                  ///< 当前算法帧率
    int detection_count{0};           ///< 当前检测数（无检测时为 0）
    bool tracking{false};             ///< 是否锁定跟踪
    bool serial_alive{false};         ///< 串口是否连接
    std::string enemy_color{"none"};  ///< 敌方颜色（"red" / "blue" / "none"）
    double target_yaw_deg{0.0};       ///< 目标云台 YAW（度）
    double target_pitch_deg{0.0};     ///< 目标云台 PITCH（度）
    double target_distance_m{0.0};    ///< 目标距离（米）
  };

  explicit HudStatusPublisher(foxglove::Context ctx);
  ~HudStatusPublisher();

  HudStatusPublisher(const HudStatusPublisher&) = delete;
  HudStatusPublisher& operator=(const HudStatusPublisher&) = delete;
  HudStatusPublisher(HudStatusPublisher&&) = delete;
  HudStatusPublisher& operator=(HudStatusPublisher&&) = delete;

  /**
   * @brief 发布 HUD 状态到 Foxglove
   *
   * @param status 包含 fps、detection_count、tracking、serial_alive 等状态
   * @param ts_ns  时间戳（纳秒），0 = 自动使用当前时间
   */
  void Publish(const HudStatus& status, uint64_t ts_ns);

 private:
  /** @brief 将 HudStatus 转换为 JSON */
  [[nodiscard]] static nlohmann::json StatusToJson(const HudStatus& status);

  foxglove::Context ctx_;
  std::mutex mtx_;
};

}  // namespace mv::tool::detail
