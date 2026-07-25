# ============================================================================
# 编译器选项配置
# ============================================================================

# C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译器标志
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wno-deprecated)
endif()

# 构建类型特定标志
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# 编译宏定义
# 注意：不使用 DEBUG/RELEASE 这类通用宏名，避免与第三方库（如 foxglove schemas.hpp 中
# Log::LogLevel::DEBUG 枚举项）发生宏替换冲突，改用项目专属前缀 MV_DEBUG/MV_RELEASE
add_compile_definitions(
    $<$<CONFIG:Debug>:MV_DEBUG>
    $<$<CONFIG:Release>:MV_RELEASE>
    SPDLOG_FMT_EXTERNAL  # 解决 spdlog 和 fmt 冲突
)

message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
