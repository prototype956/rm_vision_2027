#pragma once

#include "hal/camera/i_camera.hpp"
#include "modules/armor_pnp/armor_pnp.hpp"

#include <cstdint>
#include <string>

#include <foxglove/schemas.hpp>

namespace mv::tool::foxglove::pnp {

[[nodiscard]] ::foxglove::schemas::SceneUpdate EncodeEstimates(
    const modules::ArmorPnpFrameResult& result, const hal::CameraFrame::FrameGeometry& geometry,
    const ::foxglove::schemas::Timestamp& timestamp);

[[nodiscard]] ::foxglove::schemas::ImageAnnotations EncodeAnnotations(
    const modules::ArmorPnpFrameResult& result, const ::foxglove::schemas::Timestamp& timestamp);

[[nodiscard]] std::string EncodeStats(const modules::ArmorPnpFrameResult& result,
                                      std::uint64_t sequence,
                                      const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::pnp
