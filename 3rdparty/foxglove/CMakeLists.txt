# ============================================================================
# Foxglove WebSocket SDK 配置
# 用于实时数据可视化和调试
# ============================================================================

message(STATUS "    Configuring Foxglove SDK...")

# 创建 INTERFACE 库（header-only + 预编译库）
add_library(foxglove_sdk INTERFACE)

# 设置头文件路径
target_include_directories(foxglove_sdk INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# 设置库文件路径
set(FOXGLOVE_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")

# 检查库文件是否存在
if(NOT EXISTS "${FOXGLOVE_LIB_PATH}/libfoxglove.so")
    message(WARNING "Foxglove library not found: ${FOXGLOVE_LIB_PATH}/libfoxglove.so")
    message(WARNING "Foxglove SDK may not work properly")
endif()

# 添加库搜索路径
link_directories(${FOXGLOVE_LIB_PATH})

# 链接库（使用 -lfoxglove 方式）
target_link_libraries(foxglove_sdk INTERFACE foxglove)

# 设置 RPATH，确保运行时能找到库
# BUILD_RPATH: 构建时使用
# INSTALL_RPATH: 安装后使用
set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH};${FOXGLOVE_LIB_PATH}" PARENT_SCOPE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${FOXGLOVE_LIB_PATH}" PARENT_SCOPE)

message(STATUS "      ✓ Foxglove SDK include: ${CMAKE_CURRENT_SOURCE_DIR}/include")
message(STATUS "      ✓ Foxglove SDK library: ${FOXGLOVE_LIB_PATH}")
