#pragma once

#include "modules/armor_pnp/armor_pnp_types.hpp"

#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <span>

namespace mv::modules::detail {

/** @brief 生成单个来源统计快照前保存的各类原始样本。 */
struct PnpMetricSamples {
  std::vector<double> reprojection;  ///< 重投影 RMSE 样本。
  std::vector<double> corner;        ///< 平均角点真值误差样本。
  std::vector<double> position;      ///< 三维位置误差样本。
  std::vector<double> depth;         ///< 深度绝对误差样本。
  std::vector<double> rotation;      ///< 姿态角距离误差样本。
  std::vector<double> jitter;        ///< 连续帧位置误差变化量样本。
};

/** @brief 汇总精修、求解、真值误差、连续帧稳定性和分组百分位指标。 */
class PnpMetrics final {
 public:
  /** @brief 累计单个检测的角点精修状态与耗时。 */
  void RecordRefinement(const CornerRefinementResult& refinement);
  /** @brief 累计匹配真值下的原始和正式输入角点误差。 */
  void RecordMatchedCorners(const CornerRefinementResult& refinement,
                            const std::array<cv::Point2f, 4>& final_corners,
                            const std::array<cv::Point2f, 4>& truth_corners);
  /** @brief 累计正式检测链的求解成功或拒绝原因。 */
  void RecordDetectionSolve(const ArmorPnpAttempt& attempt);
  /** @brief 写入连续帧抖动/候选切换并将有效估计加入全局和分组样本。 */
  void RecordAttempts(std::span<ArmorPnpAttempt> attempts, std::uint64_t sequence);
  /** @brief 每 100 帧原子更新一次全部摘要，并将最近快照复制到帧结果。 */
  void PopulateSnapshot(std::uint64_t sequence, ArmorPnpFrameResult& result);

 private:
  PnpMetricSamples truth_samples_;      ///< 真值投影基准链累计样本。
  PnpMetricSamples detection_samples_;  ///< 正式检测链累计样本。
  std::map<std::string, PnpMetricSamples> distance_samples_;  ///< 按真值距离分组样本。
  std::map<std::string, PnpMetricSamples> angle_samples_;     ///< 按真值观察角分组样本。
  std::map<std::string, PnpMetricSamples> size_samples_;  ///< 按真值装甲尺寸分组样本。
  std::map<std::pair<PnpInputSource, std::uint64_t>, geometry::Vector3>
      previous_error_;  ///< 来源和真值 ID 到上一连续帧位置误差。
  std::map<std::pair<PnpInputSource, std::uint64_t>, std::size_t>
      previous_candidate_;  ///< 来源和真值 ID 到上一 IPPE 候选。
  std::map<std::pair<PnpInputSource, std::uint64_t>, std::uint64_t>
      previous_sequence_;                        ///< 来源和真值 ID 到上一观测帧序号。
  PnpSolveSummary solve_summary_;                ///< 尚未快照的正式求解累计状态。
  CornerRefinementSummary refinement_summary_;   ///< 尚未快照的精修累计状态。
  PnpSolveSummary solve_snapshot_;               ///< 最近一次原子更新的求解状态。
  CornerRefinementSummary refinement_snapshot_;  ///< 最近一次原子更新的精修状态。
  std::vector<double> refinement_elapsed_samples_;  ///< 精修耗时累计样本。
  std::vector<double> raw_corner_error_samples_;    ///< 原始网络角点真值误差样本。
  std::vector<double> final_corner_error_samples_;  ///< 正式输入角点真值误差样本。
  PnpSourceSummary truth_summary_;                  ///< 最近的真值基准链统计快照。
  PnpSourceSummary detection_summary_;              ///< 最近的正式检测链统计快照。
  std::map<std::string, PnpSourceSummary> distance_summaries_;  ///< 最近的距离分组快照。
  std::map<std::string, PnpSourceSummary> angle_summaries_;  ///< 最近的观察角分组快照。
  std::map<std::string, PnpSourceSummary> size_summaries_;   ///< 最近的尺寸分组快照。
  std::uint64_t summary_sequence_{0};  ///< 最近一次原子快照对应的帧序号。
  bool summary_initialized_{false};    ///< 是否至少生成过一次统计快照。
};

}  // namespace mv::modules::detail
