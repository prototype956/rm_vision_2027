#include "modules/armor_predictor/detail/armor_association.hpp"

#include "modules/armor_predictor/detail/four_armor_model.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include <numbers>

namespace mv::modules::detail {
namespace {

bool Finite(double value) noexcept {
  return std::isfinite(value);
}

}  // namespace

std::vector<int> AssociateArmors(std::span<const Observation> observations,
                                 const StateVector& state, const ArmorPredictorConfig& config,
                                 std::vector<ArmorAssociation>& diagnostics) {
  std::vector<int> best(observations.size(), -1), current(observations.size(), -1);
  int best_count = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  std::array<bool, 4> used{};
  std::vector<std::array<double, 4>> costs(observations.size());
  // 代价在两个门限下分别归一化，位置和 yaw 因而以无量纲平方误差共同参与排序。
  for (std::size_t row = 0; row < observations.size(); ++row) {
    costs[row].fill(std::numeric_limits<double>::infinity());
    for (int slot = 0; slot < 4; ++slot) {
      const auto PREDICTED_POSITION = WorldArmorPosition(state, slot);
      const double position_error =
          (observations[row].world_t_armor.translation - PREDICTED_POSITION).norm();
      const double predicted_yaw = WrapAngle(state[6] + slot * std::numbers::pi / 2.0);
      const double yaw_error = std::abs(WrapAngle(observations[row].yaw - predicted_yaw));
      if (position_error <= config.association_max_position_m &&
          yaw_error <= config.association_max_yaw_rad) {
        costs[row][slot] = std::pow(position_error / config.association_max_position_m, 2) +
                           std::pow(yaw_error / config.association_max_yaw_rad, 2);
      }
    }
  }
  std::function<void(std::size_t, int, double)> search = [&](std::size_t row, int count,
                                                             double cost) {
    if (row == observations.size()) {
      if (count > best_count || (count == best_count && cost < best_cost)) {
        best_count = count;
        best_cost = cost;
        best = current;
      }
      return;
    }
    current[row] = -1;
    search(row + 1, count, cost);
    for (int slot = 0; slot < 4; ++slot) {
      if (used[slot] || !Finite(costs[row][slot]))
        continue;
      used[slot] = true;
      current[row] = slot;
      search(row + 1, count + 1, cost + costs[row][slot]);
      used[slot] = false;
    }
    current[row] = -1;
  };
  // 单帧候选数很小，穷举可明确保证“匹配数最大，其次总代价最小”的全局最优解。
  search(0, 0, 0.0);

  diagnostics.clear();
  diagnostics.reserve(observations.size());
  for (std::size_t row = 0; row < observations.size(); ++row) {
    ArmorAssociation diagnostic;
    diagnostic.input_index = observations[row].input_index;
    diagnostic.slot = best[row];
    diagnostic.observed_position_world = observations[row].world_t_armor.translation;
    if (best[row] < 0) {
      diagnostic.rejection_reason = "association_gate";
    } else {
      const auto PREDICTED_POSITION = WorldArmorPosition(state, best[row]);
      diagnostic.predicted_position_world = PREDICTED_POSITION;
      diagnostic.position_error_m =
          (observations[row].world_t_armor.translation - PREDICTED_POSITION).norm();
      diagnostic.yaw_error_rad =
          WrapAngle(observations[row].yaw -
                    WrapAngle(state[6] + static_cast<double>(best[row]) * std::numbers::pi / 2.0));
    }
    diagnostics.push_back(std::move(diagnostic));
  }
  return best;
}

}  // namespace mv::modules::detail
