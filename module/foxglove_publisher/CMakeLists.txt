# ============================================================================
# Foxglove Publisher - WebSocket 数据发布模块
# ============================================================================

message(STATUS "    Configuring foxglove_publisher...")

# 获取 Foxglove C++ 源文件
file(GLOB_RECURSE FOXGLOVE_CPP_SOURCES "${CMAKE_SOURCE_DIR}/3rdparty/foxglove/src/*.cpp")

# 定义 foxglove_publisher 库
add_library(mv-foxglove-publisher SHARED
    foxglove_publisher.cpp
    ${FOXGLOVE_CPP_SOURCES}
)

# 包含路径
target_include_directories(mv-foxglove-publisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/module>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/devices>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/utils>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/base>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/3rdparty/foxglove/include>
)

# 链接依赖
target_link_libraries(mv-foxglove-publisher
    PUBLIC
    mv-base64
    opencv_core
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    fmt::fmt
    spdlog::spdlog
)

message(STATUS "      ✓ Foxglove publisher configured")
