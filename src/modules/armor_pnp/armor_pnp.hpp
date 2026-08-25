#pragma once

#include "frame/frame_types.hpp"
#include "modules/armor_pnp/armor_pnp_config.hpp"
#include "modules/armor_pnp/armor_pnp_types.hpp"

#include <memory>

#include <span>

namespace mv::modules {

/** @brief 对正式检测角点运行 IPPE，并维护真值无关的求解健康统计。 */
class ArmorPnp final {
 public:
  explicit ArmorPnp(ArmorPnpConfig config);
  ~ArmorPnp();

  ArmorPnp(const ArmorPnp& other);
  ArmorPnp& operator=(const ArmorPnp& other);
  ArmorPnp(ArmorPnp&& other) noexcept;
  ArmorPnp& operator=(ArmorPnp&& other) noexcept;

  /** @brief 对当前帧全部正式检测运行一次 PnP，并更新累计健康快照。 */
  [[nodiscard]] ArmorPnpFrameResult ProcessFrame(
      std::uint64_t sequence, const frame::CameraModel& camera_model,
      std::span<const ArmorDetection> detections,
      std::span<const CornerRefinementResult> refinements);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::modules
