# ============================================================================
# MindVision Camera SDK 配置
# 迈德威视工业相机 SDK
# ============================================================================

message(STATUS "    Configuring MindVision SDK...")

# 创建 IMPORTED 库（GLOBAL 使其可跨目录可见，供 src/hal/camera 检测 TARGET MVSDK）
add_library(MVSDK SHARED IMPORTED GLOBAL)

# 检测系统架构
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCHITECTURE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "      System architecture: ${ARCHITECTURE}")

# 根据架构选择对应的库
set(MVSDK_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/linux")
set(MVSDK_LIB_DIR "${MVSDK_BASE_DIR}/lib")

if(ARCHITECTURE STREQUAL "x86_64")
    set(MVSDK_LIBRARY "${MVSDK_LIB_DIR}/x64/libMVSDK.so")
elseif(ARCHITECTURE STREQUAL "x86")
    set(MVSDK_LIBRARY "${MVSDK_LIB_DIR}/x86/libMVSDK.so")
elseif(ARCHITECTURE STREQUAL "aarch64")
    set(MVSDK_LIBRARY "${MVSDK_LIB_DIR}/arm64/libMVSDK.so")
elseif(ARCHITECTURE STREQUAL "armv7" OR ARCHITECTURE STREQUAL "armv7l")
    set(MVSDK_LIBRARY "${MVSDK_LIB_DIR}/arm/libMVSDK.so")
else()
    message(FATAL_ERROR "Unsupported architecture for MindVision SDK: ${ARCHITECTURE}")
endif()

# 检查库文件是否存在
if(NOT EXISTS ${MVSDK_LIBRARY})
    message(FATAL_ERROR "MindVision SDK library not found: ${MVSDK_LIBRARY}")
endif()

# 设置库属性
set_target_properties(MVSDK PROPERTIES
    IMPORTED_LOCATION ${MVSDK_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES "${MVSDK_BASE_DIR}/include"
)

message(STATUS "      ✓ MindVision SDK library: ${MVSDK_LIBRARY}")
message(STATUS "      ✓ MindVision SDK include: ${MVSDK_BASE_DIR}/include")
