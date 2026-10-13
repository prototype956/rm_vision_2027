#pragma once

namespace mv::modules {
/** @brief 参考时域采样约定；将周期和点数绑定传递，避免混淆单位。 */
struct GimbalReferenceTiming {
  double dt_s{0.01};
  int horizon_steps{50};
};
}  // namespace mv::modules
