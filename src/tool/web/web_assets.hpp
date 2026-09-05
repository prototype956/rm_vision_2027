#pragma once

#include <string_view>

namespace mv::tool::web {

/** @brief 返回不依赖外部资源的 Web 调试单页应用。 */
[[nodiscard]] std::string_view IndexHtml() noexcept;

}  // namespace mv::tool::web
