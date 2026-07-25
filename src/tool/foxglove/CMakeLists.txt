# ============================================================================
# src/tool/foxglove/ — Foxglove 可视化调试工具库 mv-tool-foxglove
# ============================================================================
# 仅供调试 / 测试程序使用，Release 主程序不强制链接。
# PImpl 模式：Foxglove SDK 头文件完全隔离在 .cpp 内，
# 上游目标 include foxglove_sink.hpp 时不引入 foxglove SDK 编译代价。

# foxglove C++ 包装层源文件（将 C API libfoxglove.so 包装为 foxglove:: C++ 类型）
# mv-foxglove-publisher（旧模块）也用同一套文件，两者独立编译无冲突
file(GLOB_RECURSE FOXGLOVE_CPP_WRAPPERS "${CMAKE_SOURCE_DIR}/3rdparty/foxglove/src/*.cpp")

add_library(mv-tool-foxglove STATIC
    foxglove_sink.cpp
    detail/serial_visualizer.cpp
    detail/traditional_vision_debug_publisher.cpp
    detail/hud_status_publisher.cpp
    detail/image_publisher.cpp
    detail/detection_publisher.cpp
    detail/pnp_visualizer.cpp
    detail/tf_publisher.cpp
    detail/thread_monitor.cpp
    detail/gimbal_publisher.cpp
    detail/tracking_visualizer.cpp
    ${FOXGLOVE_CPP_WRAPPERS}   # Foxglove C++ 包装器（channel / server / schemas 等）
)

target_include_directories(mv-tool-foxglove
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src   # 使 #include "tool/foxglove/..." 生效
    PRIVATE
        ${CMAKE_SOURCE_DIR}/3rdparty/foxglove/include  # SDK 头文件（仅供 .cpp 编译）
)

target_link_libraries(mv-tool-foxglove
    PUBLIC
        mv-interfaces             # mv::Detection / GimbalControl
        ${OpenCV_LIBS}
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
    PRIVATE
        foxglove_sdk   # 提供 -lfoxglove（C API .so）+ 搜索路径
        spdlog::spdlog
)

message(STATUS "      + mv-tool-foxglove (foxglove sink / image / detection / pnp / tf / thread monitor / gimbal / tracking)")
