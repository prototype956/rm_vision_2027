# ============================================================================
# EX_MiracleVision - BJFU RoboMaster 2026 Vision System
# ============================================================================
# 重构日期: 2026-02-11
# 依赖管理: 系统包 + 仓库内 MindVision SDK
# 平台支持: Linux only
# ============================================================================

cmake_minimum_required(VERSION 3.15)

project(MiracleVision
    VERSION 2024.2.20
    LANGUAGES CXX
    DESCRIPTION "BJFU RoboMaster 2026 Vision System"
)

# ============================================================================
# 项目选项
# ============================================================================

# 构建选项
option(BUILD_MAIN "Build main MiracleVision executable" ON)

# 第三方库选项
option(USE_MINDVISION_SDK "Enable MindVision Camera SDK (required for real camera)" ON)
option(USE_OPENVINO "Enable OpenVINO armor detector" ON)

if(BUILD_MAIN AND NOT USE_OPENVINO)
    message(FATAL_ERROR
        "BUILD_MAIN requires USE_OPENVINO=ON. "
        "Use -DBUILD_MAIN=OFF -DUSE_OPENVINO=OFF for camera-only builds.")
endif()

# ============================================================================
# 开发工具配置
# ============================================================================

# 导出 compile_commands.json 供 clangd 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# 平台检查
# ============================================================================

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "Only Linux platform is supported!")
endif()

message(STATUS "")
message(STATUS "============================================================")
message(STATUS "  EX_MiracleVision Configuration")
message(STATUS "============================================================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "System: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")

# ============================================================================
# CMake 模块路径
# ============================================================================

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ============================================================================
# 包含 CMake 配置模块
# ============================================================================

message(STATUS "Loading CMake modules...")

# 编译器选项
include(CompilerOptions)

# 系统包依赖查找
include(Dependencies)

# 第三方库配置
include(ThirdParty)

message(STATUS "")

# ============================================================================
# 配置路径定义
# ============================================================================

set(CONFIG_FILE_PATH "${CMAKE_SOURCE_DIR}/src/config"
    CACHE PATH "Configuration files directory")

message(STATUS "Path configuration:")
message(STATUS "  Config path: ${CONFIG_FILE_PATH}")
message(STATUS "  Project root: ${CMAKE_SOURCE_DIR}")
message(STATUS "")

# ============================================================================
# 添加子目录
# ============================================================================

message(STATUS "Configuring project modules...")

# 核心模块
add_subdirectory(src)

# 独立硬件测试程序（默认构建）
add_subdirectory(test)

message(STATUS "")

# ============================================================================
# 输出目录配置
# ============================================================================

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ============================================================================
# 构建总结
# ============================================================================

message(STATUS "============================================================")
message(STATUS "  Configuration Summary")
message(STATUS "============================================================")
message(STATUS "Build options:")
message(STATUS "  BUILD_MAIN:         ${BUILD_MAIN}")
message(STATUS "")
message(STATUS "Third-party libraries:")
message(STATUS "  MindVision SDK:     ${USE_MINDVISION_SDK}")
message(STATUS "  OpenVINO:           ${USE_OPENVINO}")
message(STATUS "")
message(STATUS "Output directories:")
message(STATUS "  Executables:        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "  Libraries:          ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "============================================================")
message(STATUS "")
message(STATUS "Configuration completed successfully!")
message(STATUS "Run 'cmake --build build' to compile the project.")
message(STATUS "")
