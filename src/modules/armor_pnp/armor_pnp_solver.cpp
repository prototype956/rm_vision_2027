#include "modules/armor_pnp/armor_pnp_solver.hpp"

#include "modules/armor_pnp/detail/ippe_solver.hpp"

#include <utility>

namespace mv::modules {

ArmorPnpSolver::ArmorPnpSolver(ArmorPnpConfig config) : config_(config) {}

ArmorPnpSolveResult ArmorPnpSolver::Solve(std::span<const cv::Point2f, 4> image_corners,
                                          geometry::ArmorType type,
                                          const frame::CameraModel& camera_model,
                                          std::size_t input_index, std::uint8_t label) const {
  return detail::SolveIppe(config_, image_corners, type, camera_model, input_index, label);
}

}  // namespace mv::modules
