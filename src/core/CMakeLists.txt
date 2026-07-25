# ============================================================================
# mv-core — 基础设施库（ConfigManager + Logger）
# ============================================================================
# 作为静态库，其他所有模块通过 target_link_libraries(... mv-core) 使用

add_library(mv-core STATIC
    logger.cpp
    # config.hpp 是 header-only，不需要 .cpp
)

target_link_libraries(mv-core
    PUBLIC
        yaml-cpp        # ConfigManager 使用（接口暴露到外部，故用 PUBLIC）
        spdlog::spdlog  # Logger 使用
        fmt::fmt
)

target_include_directories(mv-core
    PUBLIC
        # 构建时：让所有 include "src/core/xxx.hpp" 的目标找到此目录
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

# C++20 及以上，shared_mutex 需要线程库
target_link_libraries(mv-core PUBLIC Threads::Threads)

# 供诊断使用：打印此库包含的源文件
message(STATUS "    + mv-core (ConfigManager, Logger)")
