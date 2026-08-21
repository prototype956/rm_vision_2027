#include "modules/armor_predictor/armor_predictor.hpp"

#include "modules/armor_predictor/detail/armor_motion_model.hpp"
#include "modules/armor_predictor/detail/error_state_ekf.hpp"
#include "modules/armor_predictor/detail/image_observation.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <bit>
#include <optional>

namespace mv::modules {
namespace {

constexpr double HALF_PI = 1.57079632679489661923;

struct DetectionObservation {
  std::size_t input_index{0};
  ArmorLabel label{ArmorLabel::SENTRY};
  hal::CameraFrame::ArmorType type{hal::CameraFrame::ArmorType::SMALL};
  std::array<cv::Point2f, 4> corners{};
  cv::Point2f center{};
  const ArmorPoseEstimate* pnp{nullptr};
};

struct PairCost {
  double center_error_px{0.0};
  double edge_angle_error_rad{0.0};
  double perimeter_ratio_error{0.0};
  double total{std::numeric_limits<double>::infinity()};
};

struct ProjectedLightbar {
  int slot{0};
  bool left{true};
  cv::Point2f top{};
  cv::Point2f bottom{};
  cv::Point2f center{};
  double length_px{0.0};
  double angle_rad{0.0};
};

struct LightbarPairCost {
  double center_error_px{0.0};
  double endpoint_distance_ratio{std::numeric_limits<double>::infinity()};
  double angle_error_rad{std::numeric_limits<double>::infinity()};
  double log_length_error{std::numeric_limits<double>::infinity()};
  double total{std::numeric_limits<double>::infinity()};
  bool passes_gate{false};
};

double WrapAngle(double angle) noexcept {
  return std::remainder(angle, 2.0 * std::acos(-1.0));
}

double WrapFourArmorYaw(double angle) noexcept {
  return std::remainder(angle, HALF_PI);
}

int LabelPriority(ArmorLabel label, const ArmorPredictorConfig& config) noexcept {
  const auto INDEX = static_cast<std::size_t>(label);
  return INDEX < config.label_priorities.size() ? config.label_priorities[INDEX]
                                                : std::numeric_limits<int>::max();
}

bool SupportedLabel(ArmorLabel label) noexcept {
  return label == ArmorLabel::SENTRY || label == ArmorLabel::ONE || label == ArmorLabel::TWO ||
         label == ArmorLabel::THREE || label == ArmorLabel::FOUR;
}

double ArmorTiltForLabel(ArmorLabel label, const ArmorPredictorConfig& config) noexcept {
  if (label == ArmorLabel::ONE)
    return config.hero_armor_tilt_rad;
  if (label == ArmorLabel::SENTRY)
    return 0.0;
  return config.vehicle_armor_tilt_rad;
}

double InitialRadiusForLabel(ArmorLabel label, const ArmorPredictorConfig& config) noexcept {
  return label == ArmorLabel::ONE ? config.hero_initial_radius_m : config.vehicle_initial_radius_m;
}

bool ValidTransform(const geometry::RigidTransform& transform) noexcept {
  return transform.translation.allFinite() && transform.rotation.coeffs().allFinite() &&
         transform.rotation.norm() > 1.0e-8;
}

bool ValidGeometry(const hal::CameraFrame::FrameGeometry& geometry) noexcept {
  const auto& calibration = geometry.calibration;
  if (calibration.width == 0 || calibration.height == 0 || !std::isfinite(calibration.fx) ||
      !std::isfinite(calibration.fy) || !std::isfinite(calibration.cx) ||
      !std::isfinite(calibration.cy) || calibration.fx <= 0.0 || calibration.fy <= 0.0 ||
      !ValidTransform(geometry.world_t_gimbal) ||
      !ValidTransform(geometry.gimbal_t_camera_optical)) {
    return false;
  }
  return std::all_of(calibration.distortion.begin(), calibration.distortion.end(),
                     [](double value) { return std::isfinite(value); });
}

bool FiniteCorners(const std::array<cv::Point2f, 4>& corners) noexcept {
  return std::all_of(corners.begin(), corners.end(), [](const cv::Point2f& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
  });
}

cv::Point2f Center(const std::array<cv::Point2f, 4>& corners) noexcept {
  cv::Point2f result{};
  for (const auto& corner : corners)
    result += corner;
  return result * 0.25F;
}

double Perimeter(const std::array<cv::Point2f, 4>& corners) noexcept {
  double result = 0.0;
  for (int index = 0; index < 4; ++index)
    result += cv::norm(corners[(index + 1) % 4] - corners[index]);
  return result;
}

PairCost CalculateCost(const std::array<cv::Point2f, 4>& observed,
                       const std::array<cv::Point2f, 4>& predicted,
                       const ArmorPredictorConfig& config) {
  PairCost result;
  result.center_error_px = cv::norm(Center(observed) - Center(predicted));
  for (int index = 0; index < 4; ++index) {
    const auto OBSERVED_EDGE = observed[(index + 1) % 4] - observed[index];
    const auto PREDICTED_EDGE = predicted[(index + 1) % 4] - predicted[index];
    result.edge_angle_error_rad +=
        std::abs(WrapAngle(std::atan2(OBSERVED_EDGE.y, OBSERVED_EDGE.x) -
                           std::atan2(PREDICTED_EDGE.y, PREDICTED_EDGE.x)));
  }
  result.edge_angle_error_rad *= 0.25;
  const double OBSERVED_PERIMETER = Perimeter(observed);
  const double PREDICTED_PERIMETER = Perimeter(predicted);
  if (OBSERVED_PERIMETER <= 1.0e-6 || PREDICTED_PERIMETER <= 1.0e-6)
    return result;
  result.perimeter_ratio_error = std::abs(std::log(OBSERVED_PERIMETER / PREDICTED_PERIMETER));
  result.total = config.association_center_weight * result.center_error_px +
                 config.association_edge_angle_weight * result.edge_angle_error_rad +
                 config.association_perimeter_ratio_weight * result.perimeter_ratio_error;
  return result;
}

std::array<cv::Point2f, 4> ProjectCorners(const detail::NominalState& state, int slot, double tilt,
                                          hal::CameraFrame::ArmorType type,
                                          const hal::CameraFrame::FrameGeometry& geometry) {
  const auto PROJECTED = detail::ProjectArmorCorners(
      detail::CastState<double>(state), {.slot = slot, .tilt_rad = tilt}, type, geometry);
  std::array<cv::Point2f, 4> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = cv::Point2f(static_cast<float>(PROJECTED[index].x()),
                                static_cast<float>(PROJECTED[index].y()));
  }
  return result;
}

std::vector<int> VisibleSlots(const detail::NominalState& state, double tilt,
                              const hal::CameraFrame::FrameGeometry& geometry, int limit) {
  const auto WORLD_T_CAMERA =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  const auto CAMERA_T_WORLD = geometry::Inverse(WORLD_T_CAMERA);
  std::vector<std::pair<double, int>> ranked;
  for (int slot = 0; slot < 4; ++slot) {
    const auto CAMERA_T_ARMOR = geometry::Compose(
        CAMERA_T_WORLD, detail::WorldArmorPose(state, {.slot = slot, .tilt_rad = tilt}));
    const geometry::Vector3 NORMAL =
        geometry::TransformVector(CAMERA_T_ARMOR, geometry::Vector3::UnitZ());
    const double DISTANCE = CAMERA_T_ARMOR.translation.norm();
    if (DISTANCE <= 1.0e-8 || CAMERA_T_ARMOR.translation.z() <= 1.0e-6)
      continue;
    const double FACING = NORMAL.dot(-CAMERA_T_ARMOR.translation / DISTANCE);
    if (FACING > 0.0)
      ranked.emplace_back(FACING, slot);
  }
  std::sort(ranked.begin(), ranked.end(), std::greater<>());
  std::vector<int> result;
  for (int index = 0; index < std::min(limit, static_cast<int>(ranked.size())); ++index)
    result.push_back(ranked[index].second);
  return result;
}

std::vector<int> Associate(const std::vector<DetectionObservation>& observations,
                           const detail::NominalState& state, double tilt,
                           const hal::CameraFrame::FrameGeometry& geometry,
                           const ArmorPredictorConfig& config, double gate,
                           std::vector<ArmorAssociation>& diagnostics) {
  const auto SLOTS = VisibleSlots(state, tilt, geometry, config.visible_slot_count);
  std::vector<std::array<cv::Point2f, 4>> projected;
  projected.reserve(SLOTS.size());
  for (int slot : SLOTS)
    projected.push_back(ProjectCorners(state, slot, tilt, observations.front().type, geometry));

  std::vector<std::vector<PairCost>> costs(observations.size(),
                                           std::vector<PairCost>(SLOTS.size()));
  for (std::size_t row = 0; row < observations.size(); ++row) {
    for (std::size_t column = 0; column < SLOTS.size(); ++column)
      costs[row][column] = CalculateCost(observations[row].corners, projected[column], config);
  }

  std::vector<int> best(observations.size(), -1), current(observations.size(), -1);
  std::vector<bool> used(SLOTS.size(), false);
  int best_count = -1;
  double best_cost = std::numeric_limits<double>::infinity();
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
    for (std::size_t column = 0; column < SLOTS.size(); ++column) {
      if (used[column] || !std::isfinite(costs[row][column].total) ||
          costs[row][column].total > gate) {
        continue;
      }
      used[column] = true;
      current[row] = static_cast<int>(column);
      search(row + 1, count + 1, cost + costs[row][column].total);
      used[column] = false;
    }
  };
  search(0, 0, 0.0);

  diagnostics.clear();
  diagnostics.reserve(observations.size());
  for (std::size_t row = 0; row < observations.size(); ++row) {
    ArmorAssociation diagnostic;
    diagnostic.input_index = observations[row].input_index;
    diagnostic.observed_corners = observations[row].corners;
    diagnostic.gate = gate;
    std::optional<std::size_t> nearest_column;
    for (std::size_t column = 0; column < SLOTS.size(); ++column) {
      if (!std::isfinite(costs[row][column].total))
        continue;
      if (!nearest_column || costs[row][column].total < costs[row][*nearest_column].total) {
        nearest_column = column;
      }
    }
    if (nearest_column) {
      const auto& nearest_cost = costs[row][*nearest_column];
      diagnostic.candidate_slot = SLOTS[*nearest_column];
      diagnostic.center_error_px = nearest_cost.center_error_px;
      diagnostic.edge_angle_error_rad = nearest_cost.edge_angle_error_rad;
      diagnostic.perimeter_ratio_error = nearest_cost.perimeter_ratio_error;
      diagnostic.total_cost = nearest_cost.total;
      diagnostic.predicted_corners = projected[*nearest_column];
    }
    if (best[row] < 0) {
      if (SLOTS.empty()) {
        diagnostic.rejection_reason = "no_visible_slot";
      } else if (!nearest_column || costs[row][*nearest_column].total > gate) {
        diagnostic.rejection_reason = "pixel_association_gate";
      } else {
        diagnostic.rejection_reason = "global_assignment_conflict";
      }
    } else {
      const std::size_t COLUMN = static_cast<std::size_t>(best[row]);
      const auto& cost = costs[row][COLUMN];
      diagnostic.slot = SLOTS[COLUMN];
      diagnostic.candidate_slot = SLOTS[COLUMN];
      diagnostic.center_error_px = cost.center_error_px;
      diagnostic.edge_angle_error_rad = cost.edge_angle_error_rad;
      diagnostic.perimeter_ratio_error = cost.perimeter_ratio_error;
      diagnostic.total_cost = cost.total;
      diagnostic.predicted_corners = projected[COLUMN];
      diagnostic.rejection_reason.clear();
      best[row] = SLOTS[COLUMN];
    }
    diagnostics.push_back(std::move(diagnostic));
  }
  return best;
}

ProjectedLightbar MakeProjectedLightbar(const std::array<cv::Point2f, 4>& corners, int slot,
                                        bool left) {
  const int TOP = left ? 0 : 1;
  const int BOTTOM = left ? 3 : 2;
  const cv::Point2f DELTA = corners[BOTTOM] - corners[TOP];
  return {.slot = slot,
          .left = left,
          .top = corners[TOP],
          .bottom = corners[BOTTOM],
          .center = (corners[TOP] + corners[BOTTOM]) * 0.5F,
          .length_px = cv::norm(DELTA),
          .angle_rad = std::atan2(static_cast<double>(DELTA.x), static_cast<double>(DELTA.y))};
}

double LineAngleError(double left, double right) noexcept {
  return std::abs(std::remainder(left - right, std::acos(-1.0)));
}

bool DuplicateOfArmor(const LightbarDetection& lightbar, const ProjectedLightbar& armor_light,
                      const ArmorPredictorConfig& config) {
  if (lightbar.length_px <= 1.0e-6 || armor_light.length_px <= 1.0e-6)
    return false;
  const double LENGTH = std::max(lightbar.length_px, armor_light.length_px);
  return cv::norm(lightbar.center - armor_light.center) <=
             config.light_dedup_center_length_ratio * LENGTH &&
         LineAngleError(lightbar.angle_rad, armor_light.angle_rad) <=
             config.light_dedup_angle_gate_rad &&
         std::abs(std::log(lightbar.length_px / armor_light.length_px)) <=
             config.light_dedup_log_length_gate;
}

LightbarPairCost CalculateLightbarCost(const LightbarDetection& observed,
                                       const ProjectedLightbar& predicted,
                                       const ArmorPredictorConfig& config) {
  LightbarPairCost result;
  if (observed.length_px <= 1.0e-6 || predicted.length_px <= 1.0e-6)
    return result;
  result.center_error_px = cv::norm(observed.center - predicted.center);
  result.endpoint_distance_ratio =
      (cv::norm(observed.top - predicted.top) + cv::norm(observed.bottom - predicted.bottom)) /
      predicted.length_px;
  result.angle_error_rad = LineAngleError(observed.angle_rad, predicted.angle_rad);
  result.log_length_error = std::abs(std::log(observed.length_px / predicted.length_px));
  result.total = config.light_match_position_weight * result.endpoint_distance_ratio +
                 config.light_match_angle_weight * result.angle_error_rad +
                 config.light_match_length_weight * result.log_length_error;
  result.passes_gate =
      result.log_length_error <= config.light_match_log_length_gate &&
      result.angle_error_rad <= config.light_match_angle_gate_rad &&
      result.endpoint_distance_ratio <= config.light_match_endpoint_distance_length_ratio;
  return result;
}

std::vector<int> AssociateLightbars(std::span<const LightbarDetection> detections,
                                    const std::vector<DetectionObservation>& armor_observations,
                                    const std::vector<int>& armor_slots,
                                    const detail::NominalState& state, double tilt,
                                    hal::CameraFrame::ArmorType type,
                                    const hal::CameraFrame::FrameGeometry& geometry,
                                    const ArmorPredictorConfig& config,
                                    std::vector<LightbarAssociation>& diagnostics) {
  diagnostics.clear();
  diagnostics.resize(detections.size());
  std::array<bool, 8> occupied{};
  std::vector<ProjectedLightbar> measured_armor_lights;
  for (std::size_t index = 0; index < armor_slots.size(); ++index) {
    if (armor_slots[index] < 0)
      continue;
    for (bool left : {true, false}) {
      occupied[static_cast<std::size_t>(armor_slots[index]) * 2U + (left ? 0U : 1U)] = true;
      measured_armor_lights.push_back(
          MakeProjectedLightbar(armor_observations[index].corners, armor_slots[index], left));
    }
  }

  const auto VISIBLE_SLOTS = VisibleSlots(state, tilt, geometry, config.visible_slot_count);
  std::vector<ProjectedLightbar> predicted;
  for (int slot : VISIBLE_SLOTS) {
    const auto CORNERS = ProjectCorners(state, slot, tilt, type, geometry);
    for (bool left : {true, false}) {
      if (!occupied[static_cast<std::size_t>(slot) * 2U + (left ? 0U : 1U)])
        predicted.push_back(MakeProjectedLightbar(CORNERS, slot, left));
    }
  }

  std::vector<bool> duplicate(detections.size(), false);
  for (std::size_t row = 0; row < detections.size(); ++row) {
    auto& diagnostic = diagnostics[row];
    diagnostic.input_index = detections[row].input_index;
    diagnostic.observed_top = detections[row].top;
    diagnostic.observed_bottom = detections[row].bottom;
    for (const auto& armor_light : measured_armor_lights) {
      if (!DuplicateOfArmor(detections[row], armor_light, config))
        continue;
      duplicate[row] = true;
      diagnostic.duplicate_full_armor = true;
      diagnostic.candidate_slot = armor_light.slot;
      diagnostic.candidate_left = armor_light.left;
      diagnostic.predicted_top = armor_light.top;
      diagnostic.predicted_bottom = armor_light.bottom;
      diagnostic.rejection_reason = "duplicate_full_armor";
      break;
    }
  }

  std::vector<std::vector<LightbarPairCost>> costs(detections.size(),
                                                   std::vector<LightbarPairCost>(predicted.size()));
  for (std::size_t row = 0; row < detections.size(); ++row) {
    if (duplicate[row])
      continue;
    for (std::size_t column = 0; column < predicted.size(); ++column)
      costs[row][column] = CalculateLightbarCost(detections[row], predicted[column], config);
  }

  struct AssignmentState {
    double cost{std::numeric_limits<double>::infinity()};
    std::vector<int> assignment;
  };
  const std::size_t MASK_COUNT = std::size_t{1} << predicted.size();
  std::vector<AssignmentState> states(MASK_COUNT);
  states[0].cost = 0.0;
  states[0].assignment.assign(detections.size(), -1);
  for (std::size_t row = 0; row < detections.size(); ++row) {
    auto next = states;
    if (duplicate[row]) {
      states = std::move(next);
      continue;
    }
    for (std::size_t mask = 0; mask < MASK_COUNT; ++mask) {
      if (!std::isfinite(states[mask].cost))
        continue;
      for (std::size_t column = 0; column < predicted.size(); ++column) {
        const std::size_t BIT = std::size_t{1} << column;
        if ((mask & BIT) != 0 || !costs[row][column].passes_gate)
          continue;
        const std::size_t NEW_MASK = mask | BIT;
        const double NEW_COST = states[mask].cost + costs[row][column].total;
        if (NEW_COST >= next[NEW_MASK].cost)
          continue;
        next[NEW_MASK] = states[mask];
        next[NEW_MASK].cost = NEW_COST;
        next[NEW_MASK].assignment[row] = static_cast<int>(column);
      }
    }
    states = std::move(next);
  }
  std::size_t best_mask = 0;
  for (std::size_t mask = 1; mask < MASK_COUNT; ++mask) {
    if (!std::isfinite(states[mask].cost))
      continue;
    const int COUNT = std::popcount(mask);
    const int BEST_COUNT = std::popcount(best_mask);
    if (COUNT > BEST_COUNT || (COUNT == BEST_COUNT && states[mask].cost < states[best_mask].cost))
      best_mask = mask;
  }
  std::vector<int> result(detections.size(), -1);
  if (!states[best_mask].assignment.empty())
    result = states[best_mask].assignment;

  for (std::size_t row = 0; row < detections.size(); ++row) {
    auto& diagnostic = diagnostics[row];
    if (duplicate[row])
      continue;
    std::optional<std::size_t> nearest;
    for (std::size_t column = 0; column < predicted.size(); ++column) {
      if (!nearest || costs[row][column].total < costs[row][*nearest].total)
        nearest = column;
    }
    if (nearest) {
      const auto& cost = costs[row][*nearest];
      const auto& candidate = predicted[*nearest];
      diagnostic.candidate_slot = candidate.slot;
      diagnostic.candidate_left = candidate.left;
      diagnostic.center_error_px = cost.center_error_px;
      diagnostic.endpoint_distance_ratio = cost.endpoint_distance_ratio;
      diagnostic.angle_error_rad = cost.angle_error_rad;
      diagnostic.log_length_error = cost.log_length_error;
      diagnostic.total_cost = cost.total;
      diagnostic.predicted_top = candidate.top;
      diagnostic.predicted_bottom = candidate.bottom;
    }
    if (result[row] < 0) {
      if (predicted.empty()) {
        diagnostic.rejection_reason = "no_visible_light";
      } else if (!nearest || !costs[row][*nearest].passes_gate) {
        diagnostic.rejection_reason = "light_association_gate";
      } else {
        diagnostic.rejection_reason = "global_assignment_conflict";
      }
      continue;
    }
    const std::size_t COLUMN = static_cast<std::size_t>(result[row]);
    const auto& cost = costs[row][COLUMN];
    const auto& match = predicted[COLUMN];
    diagnostic.slot = match.slot;
    diagnostic.left = match.left;
    diagnostic.candidate_slot = match.slot;
    diagnostic.candidate_left = match.left;
    diagnostic.center_error_px = cost.center_error_px;
    diagnostic.endpoint_distance_ratio = cost.endpoint_distance_ratio;
    diagnostic.angle_error_rad = cost.angle_error_rad;
    diagnostic.log_length_error = cost.log_length_error;
    diagnostic.total_cost = cost.total;
    diagnostic.predicted_top = match.top;
    diagnostic.predicted_bottom = match.bottom;
    diagnostic.rejection_reason.clear();
  }
  return result;
}

struct LightOnlyPairSelection {
  std::vector<bool> usable;
  int pair_count{0};
};

/** @brief 仅保留同一槽位同时拥有左右身份的独立灯条，避免单线观测驱动完整状态。 */
LightOnlyPairSelection SelectLightOnlyPairs(const std::vector<int>& matches,
                                            std::vector<LightbarAssociation>& diagnostics) {
  std::array<unsigned int, 4> identity_masks{};
  for (std::size_t index = 0; index < matches.size(); ++index) {
    if (matches[index] < 0 || diagnostics[index].slot < 0 || diagnostics[index].slot >= 4)
      continue;
    identity_masks[static_cast<std::size_t>(diagnostics[index].slot)] |=
        diagnostics[index].left ? 1U : 2U;
  }

  LightOnlyPairSelection result;
  result.usable.resize(matches.size(), false);
  for (const unsigned int MASK : identity_masks) {
    if (MASK == 3U)
      ++result.pair_count;
  }
  for (std::size_t index = 0; index < matches.size(); ++index) {
    if (matches[index] < 0 || diagnostics[index].slot < 0 || diagnostics[index].slot >= 4)
      continue;
    const bool PAIRED = identity_masks[static_cast<std::size_t>(diagnostics[index].slot)] == 3U;
    result.usable[index] = PAIRED;
    if (!PAIRED)
      diagnostics[index].rejection_reason = "insufficient_light_only_geometry";
  }
  return result;
}

double ObservedDepthDifference(const ArmorPoseEstimate& estimate) {
  const double WIDTH = estimate.type == hal::CameraFrame::ArmorType::LARGE ? 0.225 : 0.135;
  const auto LEFT =
      geometry::TransformPoint(estimate.camera_t_armor, geometry::Vector3(-0.5 * WIDTH, 0.0, 0.0));
  const auto RIGHT =
      geometry::TransformPoint(estimate.camera_t_armor, geometry::Vector3(0.5 * WIDTH, 0.0, 0.0));
  return LEFT.z() - RIGHT.z();
}

detail::NominalState InitialState(const DetectionObservation& observation,
                                  const hal::CameraFrame::FrameGeometry& geometry,
                                  const ArmorPredictorConfig& config) {
  const double RADIUS = InitialRadiusForLabel(observation.label, config);
  detail::NominalState state;
  state.log_radius_1 = std::log(RADIUS);
  state.log_radius_2 = std::log(RADIUS);
  const double TILT = ArmorTiltForLabel(observation.label, config);
  const auto CAR_T_ARMOR =
      detail::CarTArmor(detail::CastState<double>(state), {.slot = 0, .tilt_rad = TILT});
  const auto WORLD_T_CAMERA =
      geometry::Compose(geometry.world_t_gimbal, geometry.gimbal_t_camera_optical);
  const auto WORLD_T_ARMOR = geometry::Compose(WORLD_T_CAMERA, observation.pnp->camera_t_armor);
  state.world_q_car = (WORLD_T_ARMOR.rotation * CAR_T_ARMOR.rotation.conjugate()).normalized();
  state.position_world = WORLD_T_ARMOR.translation - state.world_q_car * CAR_T_ARMOR.translation;
  return state;
}

double YawVariance(const detail::NominalState& state, const detail::StateMatrix& covariance) {
  constexpr double EPSILON = 1.0e-6;
  Eigen::RowVector3d jacobian;
  for (int axis = 0; axis < 3; ++axis) {
    detail::ErrorVector plus_error = detail::ErrorVector::Zero();
    detail::ErrorVector minus_error = detail::ErrorVector::Zero();
    plus_error[detail::state_index::ROT_X + axis] = EPSILON;
    minus_error[detail::state_index::ROT_X + axis] = -EPSILON;
    auto plus = state;
    auto minus = state;
    detail::Inject(plus_error, plus);
    detail::Inject(minus_error, minus);
    jacobian[axis] =
        WrapAngle(detail::HeadingYaw(plus) - detail::HeadingYaw(minus)) / (2.0 * EPSILON);
  }
  return std::max(
      0.0,
      (jacobian * covariance.block<3, 3>(detail::state_index::ROT_X, detail::state_index::ROT_X) *
       jacobian.transpose())(0, 0));
}

enum class ManeuverPhase { IDLE, PENDING, ACTIVE };

const char* ManeuverPhaseName(ManeuverPhase phase) noexcept {
  switch (phase) {
    case ManeuverPhase::IDLE:
      return "idle";
    case ManeuverPhase::PENDING:
      return "pending";
    case ManeuverPhase::ACTIVE:
      return "active";
  }
  return "idle";
}

}  // namespace

struct ArmorPredictor::Impl {
  explicit Impl(ArmorPredictorConfig value) : config(value) {}

  void Reset(std::string reason);
  [[nodiscard]] bool ObserveManeuverEvidence(std::string trigger, double cost);
  void ClearManeuver() noexcept;
  void Initialize(const DetectionObservation& observation,
                  const hal::CameraFrame::FrameGeometry& geometry);
  [[nodiscard]] std::vector<DetectionObservation> ExtractObservations(
      std::span<const ArmorDetection> detections,
      std::span<const CornerRefinementResult> refinements,
      const ArmorPnpFrameResult& pnp_result) const;
  [[nodiscard]] ArmorPredictionResult Snapshot(const hal::CameraFrame& frame, double dt) const;
  [[nodiscard]] ArmorPredictionResult ProcessFrame(
      const hal::CameraFrame& frame, std::span<const ArmorDetection> detections,
      std::span<const CornerRefinementResult> refinements, const ArmorPnpFrameResult& pnp_result,
      const LightbarDetectionResult& lightbar_result);

  ArmorPredictorConfig config;
  detail::ArmorEsekf filter;
  TrackerState tracker_state{TrackerState::LOST};
  std::optional<ArmorLabel> label;
  std::optional<hal::CameraFrame::ArmorType> type;
  int detect_count{0};
  int temp_lost_count{0};
  bool has_timestamp{false};
  bool timestamp_uses_capture{false};
  std::uint64_t last_capture_timestamp_ns{0};
  std::chrono::steady_clock::time_point last_receive_time{};
  std::string last_reset_reason;
  std::uint64_t reset_count{0};
  ManeuverPhase maneuver_phase{ManeuverPhase::IDLE};
  int maneuver_evidence_frames{0};
  double maneuver_evidence_cost{0.0};
  double maneuver_confirmation_remaining_s{0.0};
  double maneuver_remaining_s{0.0};
  std::string maneuver_trigger;
  double frame_yaw_process_variance_used{0.0};
  std::optional<double> frame_trial_yaw_velocity_update_rad_s;
};

const char* TrackerStateName(TrackerState state) noexcept {
  switch (state) {
    case TrackerState::LOST:
      return "lost";
    case TrackerState::DETECTING:
      return "detecting";
    case TrackerState::TRACKING:
      return "tracking";
    case TrackerState::TEMP_LOST:
      return "temp_lost";
  }
  return "unknown";
}

void ArmorPredictor::Impl::Reset(std::string reason) {
  tracker_state = TrackerState::LOST;
  label.reset();
  type.reset();
  filter.Reset();
  detect_count = 0;
  temp_lost_count = 0;
  ClearManeuver();
  last_reset_reason = std::move(reason);
  ++reset_count;
}

bool ArmorPredictor::Impl::ObserveManeuverEvidence(std::string trigger, double cost) {
  if (maneuver_phase == ManeuverPhase::ACTIVE)
    return false;
  if (maneuver_phase == ManeuverPhase::IDLE) {
    maneuver_phase = ManeuverPhase::PENDING;
    maneuver_evidence_frames = 1;
    maneuver_evidence_cost = cost;
    maneuver_confirmation_remaining_s = config.maneuver_confirmation_window_s;
    maneuver_trigger = std::move(trigger);
    return false;
  }
  ++maneuver_evidence_frames;
  maneuver_evidence_cost = std::max(maneuver_evidence_cost, cost);
  maneuver_trigger = std::move(trigger);
  if (maneuver_evidence_frames < config.maneuver_confirmation_frames)
    return false;
  maneuver_phase = ManeuverPhase::ACTIVE;
  maneuver_confirmation_remaining_s = 0.0;
  maneuver_remaining_s = config.maneuver_hold_s;
  return true;
}

void ArmorPredictor::Impl::ClearManeuver() noexcept {
  maneuver_phase = ManeuverPhase::IDLE;
  maneuver_evidence_frames = 0;
  maneuver_evidence_cost = 0.0;
  maneuver_confirmation_remaining_s = 0.0;
  maneuver_remaining_s = 0.0;
  maneuver_trigger.clear();
}

void ArmorPredictor::Impl::Initialize(const DetectionObservation& observation,
                                      const hal::CameraFrame::FrameGeometry& geometry) {
  filter.Initialize(InitialState(observation, geometry, config), config);
  if (!filter.Initialized() || filter.Diverged(config)) {
    Reset("initial_state_invalid");
    return;
  }
  label = observation.label;
  type = observation.type;
  tracker_state = TrackerState::DETECTING;
  detect_count = 1;
  temp_lost_count = 0;
  ClearManeuver();
  last_reset_reason.clear();
}

std::vector<DetectionObservation> ArmorPredictor::Impl::ExtractObservations(
    std::span<const ArmorDetection> detections, std::span<const CornerRefinementResult> refinements,
    const ArmorPnpFrameResult& pnp_result) const {
  std::vector<DetectionObservation> result;
  result.reserve(detections.size());
  for (std::size_t index = 0; index < detections.size(); ++index) {
    if (!SupportedLabel(detections[index].label))
      continue;
    const auto& corners =
        refinements[index].success ? refinements[index].refined_corners : detections[index].corners;
    if (!FiniteCorners(corners))
      continue;
    const ArmorPoseEstimate* estimate = nullptr;
    for (const auto& attempt : pnp_result.attempts) {
      if (attempt.source == PnpInputSource::DETECTION && attempt.input_index == index &&
          attempt.estimate) {
        estimate = &*attempt.estimate;
        break;
      }
    }
    result.push_back({.input_index = index,
                      .label = detections[index].label,
                      .type = ArmorTypeForLabel(detections[index].label),
                      .corners = corners,
                      .center = Center(corners),
                      .pnp = estimate});
  }
  return result;
}

ArmorPredictionResult ArmorPredictor::Impl::Snapshot(const hal::CameraFrame& frame,
                                                     double dt) const {
  ArmorPredictionResult result;
  result.sequence = frame.sequence;
  result.source_capture_timestamp_ns = frame.capture_timestamp_ns;
  result.source_receive_steady_time = frame.receive_steady_time;
  result.state = tracker_state;
  result.label = label;
  result.type = type;
  result.dt_s = dt;
  result.reset_reason = last_reset_reason;
  result.reset_count = reset_count;
  result.maneuver_active = maneuver_phase == ManeuverPhase::ACTIVE;
  result.maneuver_phase = ManeuverPhaseName(maneuver_phase);
  result.maneuver_trigger = maneuver_trigger;
  result.maneuver_evidence_frames = maneuver_evidence_frames;
  result.maneuver_evidence_cost = maneuver_evidence_cost;
  result.maneuver_confirmation_remaining_s = maneuver_confirmation_remaining_s;
  result.maneuver_remaining_s = maneuver_remaining_s;
  result.yaw_process_variance_used = frame_yaw_process_variance_used;
  result.trial_yaw_velocity_update_rad_s = frame_trial_yaw_velocity_update_rad_s;
  if (tracker_state == TrackerState::LOST)
    return result;

  const auto& state = filter.State();
  const auto& covariance = filter.Covariance();
  result.state_vector = detail::DiagnosticState(state);
  for (int index = 0; index < detail::K_STATE_SIZE; ++index)
    result.covariance_diagonal[index] = covariance(index, index);
  result.center_world = state.position_world;
  result.velocity_world = state.velocity_world;
  result.orientation_world = state.world_q_car;
  result.yaw_velocity_rad_s = state.yaw_velocity_rad_s;
  result.radii_m = {std::exp(state.log_radius_1), std::exp(state.log_radius_2)};
  result.height_offset_m = state.height_offset_m;
  result.armor_tilt_rad = label ? ArmorTiltForLabel(*label, config) : 0.0;
  const std::array<int, 3> CENTER_INDICES{detail::state_index::CX, detail::state_index::CY,
                                          detail::state_index::CZ};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column)
      result.center_covariance_world(row, column) =
          covariance(CENTER_INDICES[row], CENTER_INDICES[column]);
  }
  result.yaw_variance_rad2 = YawVariance(state, covariance);

  result.horizons.reserve(config.prediction_horizons_s.size());
  for (double seconds : config.prediction_horizons_s) {
    const auto FUTURE = detail::PredictState(state, seconds);
    PredictionHorizon horizon;
    horizon.seconds = seconds;
    horizon.center_world = FUTURE.position_world;
    horizon.orientation_world = FUTURE.world_q_car;
    horizon.yaw = detail::HeadingYaw(FUTURE);
    for (int slot = 0; slot < 4; ++slot) {
      horizon.armors[slot] = {.slot = slot,
                              .world_t_armor = detail::WorldArmorPose(
                                  FUTURE, {.slot = slot, .tilt_rad = result.armor_tilt_rad})};
    }
    result.horizons.push_back(std::move(horizon));
  }

  if (frame.geometry && label) {
    const auto BEST = std::min_element(
        frame.geometry->targets.begin(), frame.geometry->targets.end(),
        [&](const auto& left, const auto& right) {
          const double LEFT_PENALTY =
              left.armor_label == static_cast<std::uint8_t>(*label) ? 0.0 : 1.0e6;
          const double RIGHT_PENALTY =
              right.armor_label == static_cast<std::uint8_t>(*label) ? 0.0 : 1.0e6;
          return LEFT_PENALTY + (left.position_world - state.position_world).squaredNorm() <
                 RIGHT_PENALTY + (right.position_world - state.position_world).squaredNorm();
        });
    if (BEST != frame.geometry->targets.end() &&
        BEST->armor_label == static_cast<std::uint8_t>(*label)) {
      result.truth_center_error_m = (state.position_world - BEST->position_world).norm();
      result.truth_yaw_error_rad = WrapAngle(detail::HeadingYaw(state) - BEST->yaw);
      result.truth_yaw_equivalent_error_rad = WrapFourArmorYaw(*result.truth_yaw_error_rad);
      result.truth_yaw_velocity_error_rad_s = state.yaw_velocity_rad_s - BEST->yaw_velocity;
    }
  }
  return result;
}

ArmorPredictionResult ArmorPredictor::Impl::ProcessFrame(
    const hal::CameraFrame& frame, std::span<const ArmorDetection> detections,
    std::span<const CornerRefinementResult> refinements, const ArmorPnpFrameResult& pnp_result,
    const LightbarDetectionResult& lightbar_result) {
  const auto START = std::chrono::steady_clock::now();
  frame_yaw_process_variance_used = 0.0;
  frame_trial_yaw_velocity_update_rad_s.reset();
  double dt = 0.0;
  const bool USE_CAPTURE = frame.capture_timestamp_ns.has_value();
  if (has_timestamp) {
    if (USE_CAPTURE != timestamp_uses_capture) {
      Reset("timestamp_source_changed");
    } else if (USE_CAPTURE) {
      if (*frame.capture_timestamp_ns <= last_capture_timestamp_ns) {
        Reset("non_monotonic_timestamp");
      } else {
        dt = static_cast<double>(*frame.capture_timestamp_ns - last_capture_timestamp_ns) * 1.0e-9;
      }
    } else {
      dt = std::chrono::duration<double>(frame.receive_steady_time - last_receive_time).count();
      if (dt <= 0.0)
        Reset("non_monotonic_timestamp");
    }
    if (dt > config.max_dt_s)
      Reset("large_dt");
  }
  has_timestamp = true;
  timestamp_uses_capture = USE_CAPTURE;
  if (USE_CAPTURE)
    last_capture_timestamp_ns = *frame.capture_timestamp_ns;
  last_receive_time = frame.receive_steady_time;
  if (dt > 0.0 && maneuver_phase == ManeuverPhase::PENDING) {
    maneuver_confirmation_remaining_s = std::max(0.0, maneuver_confirmation_remaining_s - dt);
    if (maneuver_confirmation_remaining_s == 0.0)
      ClearManeuver();
  } else if (dt > 0.0 && maneuver_phase == ManeuverPhase::ACTIVE) {
    maneuver_remaining_s = std::max(0.0, maneuver_remaining_s - dt);
    if (maneuver_remaining_s == 0.0)
      ClearManeuver();
  }

  const auto FINISH = [&](ArmorPredictionResult result) {
    result.estimation_elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - START).count();
    return result;
  };
  if (detections.size() != refinements.size()) {
    Reset("detection_refinement_count_mismatch");
    return FINISH(Snapshot(frame, dt));
  }
  if (!frame.geometry) {
    Reset("missing_frame_geometry");
    return FINISH(Snapshot(frame, dt));
  }
  if (!ValidGeometry(*frame.geometry)) {
    Reset("invalid_frame_geometry");
    return FINISH(Snapshot(frame, dt));
  }
  const bool MANEUVER_ACTIVE_AT_PREDICT = maneuver_phase == ManeuverPhase::ACTIVE;
  if (tracker_state != TrackerState::LOST && dt > 0.0) {
    const double YAW_VARIANCE = MANEUVER_ACTIVE_AT_PREDICT
                                    ? config.maneuver_active_yaw_acceleration_variance
                                    : config.yaw_acceleration_variance;
    frame_yaw_process_variance_used = YAW_VARIANCE;
    if (!filter.Predict(dt, config, YAW_VARIANCE)) {
      Reset("esekf_prediction_failed");
      return FINISH(Snapshot(frame, dt));
    }
  }

  const auto OBSERVATIONS = ExtractObservations(detections, refinements, pnp_result);
  if (tracker_state == TrackerState::LOST) {
    std::vector<const DetectionObservation*> candidates;
    for (const auto& observation : OBSERVATIONS) {
      if (observation.pnp != nullptr)
        candidates.push_back(&observation);
    }
    if (candidates.empty()) {
      auto result = Snapshot(frame, dt);
      result.detected_lightbar_count = static_cast<int>(lightbar_result.detections.size());
      for (const auto& lightbar : lightbar_result.detections) {
        result.lightbar_associations.push_back({.input_index = lightbar.input_index,
                                                .observed_top = lightbar.top,
                                                .observed_bottom = lightbar.bottom,
                                                .rejection_reason = "tracker_unavailable"});
      }
      result.rejected_lightbar_count = result.detected_lightbar_count;
      return FINISH(std::move(result));
    }
    const auto& calibration = frame.geometry->calibration;
    const auto BEST = *std::min_element(
        candidates.begin(), candidates.end(), [&](const auto* left, const auto* right) {
          const int LEFT_PRIORITY = LabelPriority(left->label, config);
          const int RIGHT_PRIORITY = LabelPriority(right->label, config);
          if (LEFT_PRIORITY != RIGHT_PRIORITY)
            return LEFT_PRIORITY < RIGHT_PRIORITY;
          return std::hypot(left->center.x - calibration.cx, left->center.y - calibration.cy) <
                 std::hypot(right->center.x - calibration.cx, right->center.y - calibration.cy);
        });
    Initialize(*BEST, *frame.geometry);
    auto result = Snapshot(frame, dt);
    result.detected_lightbar_count = static_cast<int>(lightbar_result.detections.size());
    for (const auto& lightbar : lightbar_result.detections) {
      result.lightbar_associations.push_back({.input_index = lightbar.input_index,
                                              .observed_top = lightbar.top,
                                              .observed_bottom = lightbar.bottom,
                                              .rejection_reason = "tracker_unavailable"});
    }
    result.rejected_lightbar_count = result.detected_lightbar_count;
    if (tracker_state != TrackerState::LOST) {
      result.associations.push_back({.input_index = BEST->input_index,
                                     .slot = 0,
                                     .candidate_slot = 0,
                                     .accepted = true,
                                     .observed_corners = BEST->corners,
                                     .predicted_corners = BEST->corners,
                                     .rejection_reason = {}});
      result.accepted_association_count = 1;
    }
    return FINISH(std::move(result));
  }

  std::vector<DetectionObservation> candidates;
  for (const auto& observation : OBSERVATIONS) {
    if (observation.label == *label && observation.type == *type)
      candidates.push_back(observation);
  }
  ArmorPredictionResult diagnostic = Snapshot(frame, dt);
  std::vector<int> slots;
  double association_gate_used = 0.0;
  bool used_recovery_gate = false;
  if (!candidates.empty()) {
    const double GATE = tracker_state == TrackerState::DETECTING
                            ? config.association_initial_gate
                            : (maneuver_remaining_s > 0.0 ? config.maneuver_recovery_gate
                                                          : config.association_gate);
    association_gate_used = GATE;
    slots = Associate(candidates, filter.State(), ArmorTiltForLabel(*label, config),
                      *frame.geometry, config, GATE, diagnostic.associations);
    const bool HAS_NORMAL_MATCH =
        std::any_of(slots.begin(), slots.end(), [](int slot) { return slot >= 0; });
    if (!HAS_NORMAL_MATCH && tracker_state != TrackerState::DETECTING &&
        GATE < config.maneuver_recovery_gate) {
      association_gate_used = config.maneuver_recovery_gate;
      slots = Associate(candidates, filter.State(), ArmorTiltForLabel(*label, config),
                        *frame.geometry, config, association_gate_used, diagnostic.associations);
      used_recovery_gate =
          std::any_of(slots.begin(), slots.end(), [](int slot) { return slot >= 0; });
    }
  }
  const int ARMOR_MATCH_COUNT = static_cast<int>(
      std::count_if(slots.begin(), slots.end(), [](int slot) { return slot >= 0; }));
  const double TILT = ArmorTiltForLabel(*label, config);
  const auto LIGHT_MATCHES =
      AssociateLightbars(lightbar_result.detections, candidates, slots, filter.State(), TILT, *type,
                         *frame.geometry, config, diagnostic.lightbar_associations);
  const int LIGHT_MATCH_COUNT = static_cast<int>(std::count_if(
      LIGHT_MATCHES.begin(), LIGHT_MATCHES.end(), [](int column) { return column >= 0; }));
  std::vector<bool> usable_light_matches(LIGHT_MATCHES.size(), true);
  if (ARMOR_MATCH_COUNT == 0) {
    auto selection = SelectLightOnlyPairs(LIGHT_MATCHES, diagnostic.lightbar_associations);
    usable_light_matches = std::move(selection.usable);
    diagnostic.light_only_pair_count = selection.pair_count;
    if (LIGHT_MATCH_COUNT > 0 && selection.pair_count == 0) {
      diagnostic.light_only_update_blocked = true;
      diagnostic.light_only_rejection_reason = "insufficient_light_only_geometry";
    }
  }
  const auto LIGHT_IS_USABLE = [&](std::size_t index) {
    return LIGHT_MATCHES[index] >= 0 && usable_light_matches[index];
  };
  int usable_light_count = 0;
  for (std::size_t index = 0; index < LIGHT_MATCHES.size(); ++index) {
    if (LIGHT_IS_USABLE(index))
      ++usable_light_count;
  }
  diagnostic.detected_lightbar_count = static_cast<int>(lightbar_result.detections.size());
  diagnostic.deduplicated_lightbar_count = static_cast<int>(std::count_if(
      diagnostic.lightbar_associations.begin(), diagnostic.lightbar_associations.end(),
      [](const LightbarAssociation& association) { return association.duplicate_full_armor; }));
  diagnostic.matched_lightbar_count = LIGHT_MATCH_COUNT;
  bool found = false;
  double maximum_cost = 0.0;
  for (const auto& association : diagnostic.associations) {
    if (association.slot >= 0)
      maximum_cost = std::max(maximum_cost, association.total_cost);
  }
  if (ARMOR_MATCH_COUNT > 0 || usable_light_count > 0) {
    std::vector<detail::ImageObservation> armor_observations;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (slots[index] < 0)
        continue;
      const auto UVL = detail::MakeUvlObservations(candidates[index].corners, *frame.geometry, TILT,
                                                   candidates[index].type, slots[index]);
      armor_observations.emplace_back(UVL[0]);
      armor_observations.emplace_back(UVL[1]);
      if (ARMOR_MATCH_COUNT == 1 && candidates[index].pnp != nullptr) {
        armor_observations.emplace_back(detail::DepthDifferenceObservation{
            .value_m = ObservedDepthDifference(*candidates[index].pnp),
            .slot = slots[index],
            .armor_tilt_rad = TILT,
            .type = candidates[index].type,
            .geometry = &*frame.geometry});
      }
    }
    std::vector<detail::ImageObservation> combined_observations = armor_observations;
    for (std::size_t index = 0; index < LIGHT_MATCHES.size(); ++index) {
      if (!LIGHT_IS_USABLE(index))
        continue;
      const auto& association = diagnostic.lightbar_associations[index];
      combined_observations.emplace_back(detail::MakeStandaloneUvlObservation(
          lightbar_result.detections[index].top, lightbar_result.detections[index].bottom,
          *frame.geometry, TILT, *type, association.slot, association.left));
    }

    const auto RUN_UPDATE = [&](const std::vector<detail::ImageObservation>& observations,
                                bool trigger_noise, detail::ArmorEsekf& candidate,
                                detail::EsekfUpdateDiagnostic& update) {
      candidate = filter;
      if (trigger_noise) {
        const double EXTRA_VARIANCE =
            config.maneuver_trigger_yaw_acceleration_variance - config.yaw_acceleration_variance;
        if (dt <= 0.0 || !candidate.AddYawAccelerationNoise(dt, EXTRA_VARIANCE))
          return false;
      }
      return candidate.Update(observations, config, update) && update.nis &&
             update.residual_dimension > 0;
    };
    const auto NIS_PER_DOF = [](const detail::EsekfUpdateDiagnostic& update) {
      return *update.nis / static_cast<double>(update.residual_dimension);
    };

    detail::ArmorEsekf armor_candidate = filter;
    detail::EsekfUpdateDiagnostic armor_update;
    const bool ARMOR_UPDATE_VALID =
        ARMOR_MATCH_COUNT > 0 &&
        RUN_UPDATE(armor_observations, false, armor_candidate, armor_update);
    detail::ArmorEsekf combined_candidate = filter;
    detail::EsekfUpdateDiagnostic combined_update;
    bool combined_update_valid =
        RUN_UPDATE(combined_observations, false, combined_candidate, combined_update);
    bool maneuver_confirmed = false;
    if (combined_update_valid) {
      const double STEADY_NIS_PER_DOF = NIS_PER_DOF(combined_update);
      const bool ASSOCIATION_COST_EVIDENCE =
          maximum_cost >= config.maneuver_association_cost_trigger;
      const bool NIS_EVIDENCE = STEADY_NIS_PER_DOF > config.maneuver_nis_per_dof_gate;
      const bool HAS_MANEUVER_EVIDENCE =
          tracker_state != TrackerState::DETECTING &&
          (used_recovery_gate || ASSOCIATION_COST_EVIDENCE || NIS_EVIDENCE);
      std::string evidence_reason;
      if (used_recovery_gate) {
        evidence_reason = "recovery_gate";
      } else if (ASSOCIATION_COST_EVIDENCE) {
        evidence_reason = "association_cost";
      } else if (NIS_EVIDENCE) {
        evidence_reason = "prior_nis";
      }

      if (maneuver_phase == ManeuverPhase::PENDING) {
        if (HAS_MANEUVER_EVIDENCE) {
          maneuver_confirmed = ObserveManeuverEvidence(evidence_reason, maximum_cost);
        } else {
          ClearManeuver();
        }
      } else if (maneuver_phase == ManeuverPhase::IDLE && HAS_MANEUVER_EVIDENCE) {
        maneuver_confirmed = ObserveManeuverEvidence(evidence_reason, maximum_cost);
      }
      if (maneuver_confirmed) {
        combined_update = {};
        combined_update_valid =
            RUN_UPDATE(combined_observations, true, combined_candidate, combined_update);
      }
    } else if (maneuver_phase == ManeuverPhase::PENDING) {
      ClearManeuver();
    }

    const bool COMBINED_PASSES_NIS =
        combined_update_valid && NIS_PER_DOF(combined_update) <= config.maneuver_nis_per_dof_gate;
    const bool ARMOR_PASSES_NIS =
        ARMOR_UPDATE_VALID && NIS_PER_DOF(armor_update) <= config.maneuver_nis_per_dof_gate;
    const detail::EsekfUpdateDiagnostic* selected_update = nullptr;
    if (COMBINED_PASSES_NIS) {
      filter = std::move(combined_candidate);
      selected_update = &combined_update;
      found = true;
      diagnostic.light_fusion_used = LIGHT_MATCH_COUNT > 0;
      diagnostic.light_only_update = usable_light_count > 0 && ARMOR_MATCH_COUNT == 0;
      for (auto& association : diagnostic.associations)
        association.accepted = association.slot >= 0;
      for (std::size_t index = 0; index < diagnostic.lightbar_associations.size(); ++index)
        diagnostic.lightbar_associations[index].accepted = LIGHT_IS_USABLE(index);
      if (maneuver_confirmed)
        frame_yaw_process_variance_used = config.maneuver_trigger_yaw_acceleration_variance;
    } else if (LIGHT_MATCH_COUNT > 0 && ARMOR_PASSES_NIS) {
      filter = std::move(armor_candidate);
      selected_update = &armor_update;
      found = true;
      diagnostic.armor_fallback_used = true;
      for (auto& association : diagnostic.associations)
        association.accepted = association.slot >= 0;
      for (std::size_t index = 0; index < diagnostic.lightbar_associations.size(); ++index) {
        auto& association = diagnostic.lightbar_associations[index];
        if (LIGHT_IS_USABLE(index))
          association.rejection_reason = "combined_nis_gate";
      }
    } else {
      if (combined_update_valid)
        selected_update = &combined_update;
      for (auto& association : diagnostic.associations) {
        if (association.slot >= 0)
          association.rejection_reason = "nis_gate";
      }
      for (std::size_t index = 0; index < diagnostic.lightbar_associations.size(); ++index) {
        auto& association = diagnostic.lightbar_associations[index];
        if (LIGHT_IS_USABLE(index))
          association.rejection_reason = combined_update_valid ? "nis_gate" : "esekf_update_failed";
      }
      if (ARMOR_MATCH_COUNT == 0 && usable_light_count > 0) {
        diagnostic.light_only_update_blocked = true;
        diagnostic.light_only_rejection_reason =
            combined_update_valid ? "nis_gate" : "esekf_update_failed";
      }
    }
    if (selected_update != nullptr) {
      diagnostic.innovation = selected_update->innovation;
      diagnostic.nis = selected_update->nis;
      diagnostic.nis_per_dof = NIS_PER_DOF(*selected_update);
      diagnostic.esekf_iterations = selected_update->iteration_count;
      frame_trial_yaw_velocity_update_rad_s = selected_update->yaw_velocity_update_rad_s;
    }
    if (!found && ARMOR_MATCH_COUNT > 0 && !combined_update_valid &&
        (LIGHT_MATCH_COUNT == 0 || !ARMOR_UPDATE_VALID)) {
      Reset(maneuver_confirmed ? "esekf_maneuver_update_failed" : "esekf_update_failed");
    }
  } else if (maneuver_phase == ManeuverPhase::PENDING) {
    ClearManeuver();
  }

  if (tracker_state == TrackerState::DETECTING) {
    if (found && ARMOR_MATCH_COUNT > 0) {
      if (++detect_count >= config.min_detect_count)
        tracker_state = TrackerState::TRACKING;
    } else if (found && LIGHT_MATCH_COUNT > 0) {
      // 初始化确认必须持续看到完整装甲；独立灯条仅维持当前候选状态。
    } else if (tracker_state != TrackerState::LOST) {
      Reset("detecting_missed");
    }
  } else if (tracker_state == TrackerState::TRACKING) {
    if (!found) {
      tracker_state = TrackerState::TEMP_LOST;
      temp_lost_count = 1;
    }
  } else if (tracker_state == TrackerState::TEMP_LOST) {
    if (found) {
      tracker_state = TrackerState::TRACKING;
      temp_lost_count = 0;
    } else if (++temp_lost_count > config.max_temp_lost_count) {
      Reset("temporary_loss_timeout");
    }
  }
  if (tracker_state != TrackerState::LOST && filter.Diverged(config))
    Reset("state_diverged");

  auto result = Snapshot(frame, dt);
  result.associations = std::move(diagnostic.associations);
  result.lightbar_associations = std::move(diagnostic.lightbar_associations);
  result.innovation = std::move(diagnostic.innovation);
  result.nis = diagnostic.nis;
  result.nis_per_dof = diagnostic.nis_per_dof;
  result.esekf_iterations = diagnostic.esekf_iterations;
  result.association_gate_used = association_gate_used;
  result.accepted_association_count = static_cast<int>(
      std::count_if(result.associations.begin(), result.associations.end(),
                    [](const ArmorAssociation& association) { return association.accepted; }));
  result.rejected_association_count =
      static_cast<int>(result.associations.size()) - result.accepted_association_count;
  result.detected_lightbar_count = diagnostic.detected_lightbar_count;
  result.deduplicated_lightbar_count = diagnostic.deduplicated_lightbar_count;
  result.matched_lightbar_count = diagnostic.matched_lightbar_count;
  result.light_only_pair_count = diagnostic.light_only_pair_count;
  result.accepted_lightbar_count = static_cast<int>(
      std::count_if(result.lightbar_associations.begin(), result.lightbar_associations.end(),
                    [](const LightbarAssociation& association) { return association.accepted; }));
  result.rejected_lightbar_count =
      std::max(0, result.detected_lightbar_count - result.deduplicated_lightbar_count -
                      result.accepted_lightbar_count);
  result.light_only_update = diagnostic.light_only_update;
  result.light_only_update_blocked = diagnostic.light_only_update_blocked;
  result.light_only_rejection_reason = std::move(diagnostic.light_only_rejection_reason);
  result.light_fusion_used = diagnostic.light_fusion_used;
  result.armor_fallback_used = diagnostic.armor_fallback_used;
  return FINISH(std::move(result));
}

ArmorPredictor::ArmorPredictor(ArmorPredictorConfig config)
    : impl_(std::make_unique<Impl>(config)) {}

ArmorPredictor::~ArmorPredictor() = default;

ArmorPredictor::ArmorPredictor(const ArmorPredictor& other)
    : impl_(other.impl_ ? std::make_unique<Impl>(*other.impl_) : nullptr) {}

ArmorPredictor& ArmorPredictor::operator=(const ArmorPredictor& other) {
  if (this != &other)
    impl_ = other.impl_ ? std::make_unique<Impl>(*other.impl_) : nullptr;
  return *this;
}

ArmorPredictor::ArmorPredictor(ArmorPredictor&& other) noexcept = default;
ArmorPredictor& ArmorPredictor::operator=(ArmorPredictor&& other) noexcept = default;

ArmorPredictionResult ArmorPredictor::ProcessFrame(
    const hal::CameraFrame& frame, std::span<const ArmorDetection> detections,
    std::span<const CornerRefinementResult> refinements, const ArmorPnpFrameResult& pnp_result,
    const LightbarDetectionResult& lightbar_result) {
  return impl_->ProcessFrame(frame, detections, refinements, pnp_result, lightbar_result);
}

PredictionHorizon ExtrapolatePrediction(const ArmorPredictionResult& prediction, double seconds) {
  if (!std::isfinite(seconds) || seconds < 0.0)
    throw std::invalid_argument("prediction horizon must be finite and nonnegative");
  detail::NominalState state;
  state.position_world = prediction.center_world;
  state.velocity_world = prediction.velocity_world;
  state.world_q_car = prediction.orientation_world;
  state.yaw_velocity_rad_s = prediction.yaw_velocity_rad_s;
  state.log_radius_1 = std::log(prediction.radii_m[0]);
  state.log_radius_2 = std::log(prediction.radii_m[1]);
  state.height_offset_m = prediction.height_offset_m;
  const auto FUTURE = detail::PredictState(state, seconds);
  PredictionHorizon horizon;
  horizon.seconds = seconds;
  horizon.center_world = FUTURE.position_world;
  horizon.orientation_world = FUTURE.world_q_car;
  horizon.yaw = detail::HeadingYaw(FUTURE);
  for (int slot = 0; slot < 4; ++slot) {
    horizon.armors[slot] = {.slot = slot,
                            .world_t_armor = detail::WorldArmorPose(
                                FUTURE, {.slot = slot, .tilt_rad = prediction.armor_tilt_rad})};
  }
  return horizon;
}

}  // namespace mv::modules
