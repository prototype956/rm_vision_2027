#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_detector/armor_detector.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <optional>

namespace mv::modules {

/** @brief 单目标跟踪状态机状态。 */
enum class TrackerState : std::uint8_t {
  LOST = 0,   ///< 尚未选定目标，等待有效观测初始化。
  DETECTING,  ///< 已初始化，等待连续匹配确认目标。
  TRACKING,   ///< 已确认目标并正常执行预测和量测更新。
  TEMP_LOST,  ///< 短时无匹配观测，仅依靠运动模型外推。
};

/** @brief 将跟踪状态转换为稳定的日志与 Foxglove 字段名称。 */
[[nodiscard]] const char* TrackerStateName(TrackerState state) noexcept;

/** @brief 单个 PnP 观测与预测装甲槽位的关联诊断。 */
struct ArmorAssociation {
  std::size_t input_index{0};    ///< 对应 ArmorPoseEstimate::input_index。
  int slot{-1};                  ///< 匹配的四装甲槽位；-1 表示未通过关联。
  double position_error_m{0.0};  ///< 观测与预测槽位的世界系位置误差。
  double yaw_error_rad{0.0};     ///< 观测 yaw 减预测槽位 yaw 的包角误差。
  geometry::Vector3 observed_position_world{geometry::Vector3::Zero()};  ///< PnP 观测位置。
  geometry::Vector3 predicted_position_world{geometry::Vector3::Zero()};  ///< 关联前预测位置。
  std::string rejection_reason;  ///< 未关联原因；成功关联时为空。
};

/** @brief 指定预测时域和槽位下的世界系装甲位姿。 */
struct PredictedArmorPose {
  int slot{0};                             ///< 绕车辆中心依次相差 90° 的槽位编号。
  geometry::RigidTransform world_t_armor;  ///< armor 到 world 的预测变换。
};

/** @brief 某一未来时刻的车辆中心、航向及四块装甲预测。 */
struct PredictionHorizon {
  double seconds{0.0};  ///< 相对当前帧的预测时间，单位为秒。
  geometry::Vector3 center_world{geometry::Vector3::Zero()};  ///< 车辆几何中心世界坐标。
  double yaw{0.0};                             ///< 槽位 0 对应的世界系航向角。
  std::array<PredictedArmorPose, 4> armors{};  ///< 四个固定槽位的预测位姿。
};

/** @brief 单帧跟踪输出及供日志、Foxglove 使用的完整滤波诊断。 */
struct ArmorPredictionResult {
  std::uint64_t sequence{0};  ///< 对应输入 CameraFrame::sequence。
  std::optional<std::uint64_t> source_capture_timestamp_ns;  ///< 状态对应的数据源采集时刻。
  std::chrono::steady_clock::time_point source_receive_steady_time{};  ///< 本机收到源帧的时刻。
  TrackerState state{TrackerState::LOST};  ///< 当前帧处理完成后的状态机状态。
  std::optional<ArmorLabel> label;         ///< 当前跟踪标签；LOST 时为空。
  std::optional<hal::CameraFrame::ArmorType> type;  ///< 当前跟踪装甲尺寸；LOST 时为空。
  double dt_s{0.0};  ///< 本帧预测使用的时间间隔，单位为秒。
  /** @brief x,vx,y,vy,z,vz,yaw,vyaw,r,dr,dz 顺序的 EKF 状态。 */
  std::array<double, 11> state_vector{};
  std::array<double, 11> covariance_diagonal{};  ///< 与状态向量同顺序的协方差对角线。
  std::vector<ArmorAssociation> associations;    ///< 当前帧候选的槽位关联诊断。
  std::vector<double> innovation;  ///< 按匹配观测拼接的方位、俯仰、距离和 yaw 残差。
  std::optional<double> nis;  ///< 当前量测更新的归一化创新平方；无更新时为空。
  double armor_roll_rad{0.0};               ///< 当前标签对应的装甲安装滚转角。
  std::vector<PredictionHorizon> horizons;  ///< 配置中全部时域的预测结果。
  geometry::Vector3 velocity_world{geometry::Vector3::Zero()};  ///< 车辆中心世界系速度。
  std::string reset_reason;  ///< 最近一次重置原因；重新初始化成功后清空。
};

/**
 * @brief 从不可变预测快照按匀速模型外推任意未来时域。
 * @throws std::invalid_argument seconds 非有限或为负数。
 */
[[nodiscard]] PredictionHorizon ExtrapolatePrediction(const ArmorPredictionResult& prediction,
                                                      double seconds);

}  // namespace mv::modules
