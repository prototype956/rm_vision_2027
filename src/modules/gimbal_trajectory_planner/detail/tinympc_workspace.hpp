#pragma once

#include "tinympc/types.hpp"

namespace mv::modules::detail {

/** @brief 将 TinyMPC 工作区的时域变量左移一个采样点，以复用上一周期求解结果。 */
void ShiftTinyMpcWarmStart(TinySolver& solver);

/** @brief 使用当前参考轨迹和初始状态重建 TinyMPC 热启动工作区。 */
void RebaseTinyMpcWarmStart(TinySolver& solver, const tinyVector& x0);

/** @brief 刷新参考轨迹对应的线性代价后执行一次 TinyMPC 求解。 */
int SolveTinyMpcWithFreshReference(TinySolver& solver);

}  // namespace mv::modules::detail
