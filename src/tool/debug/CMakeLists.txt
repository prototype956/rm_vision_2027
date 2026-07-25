# ============================================================================
# src/tool/debug/ — 调试工具库 mv-tool-debug
# ============================================================================
# 仅供 test / 离线工具程序使用，不参与 Release 主程序构建。
# Pimpl 隔离了 OpenCV / yaml-cpp，上游目标在 include 本库头文件时
# 不引入这两个大型依赖的编译代价。

add_library(mv-tool-debug STATIC
    param_tuner.cpp
    view_renderer.cpp
    debug_session.cpp
    light_vis_painter.cpp
    predict_param_manager.cpp
    armor_param_manager.cpp
    shooter_param_manager.cpp
    pnp_param_manager.cpp
)

target_include_directories(mv-tool-debug
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src   # 使 #include "tool/debug/..." 生效
)

target_link_libraries(mv-tool-debug
    PUBLIC
        mv-core
        mv-mod-armor-detector    # DebugData / Params 类型来源
        ${OpenCV_LIBS}
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
    PRIVATE
        yaml-cpp
)

message(STATUS "      + mv-tool-debug (debug session / param tuner / view renderer / predict param manager)")
