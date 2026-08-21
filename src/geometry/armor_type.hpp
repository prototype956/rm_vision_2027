#pragma once

#include <cstdint>

namespace mv::geometry {

/** @brief 装甲板物理尺寸类别，用于选择对应的三维物点模型。 */
enum class ArmorType : std::uint8_t { SMALL = 0, LARGE = 1 };

}  // namespace mv::geometry
