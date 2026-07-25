# ============================================================================
# mv-interfaces — 跨层共享数据类型与业务接口（纯头文件库）
# ============================================================================
#
# 为什么是 INTERFACE 库？
#   所有接口和数据类型都在头文件中定义，没有需要单独编译的 .cpp。
#   INTERFACE 库只传播 include 路径和链接依赖，不产生 .a/.so 目标文件，
#   编译速度更快，不引入多余的链接符号。

add_library(mv-interfaces INTERFACE)

target_include_directories(mv-interfaces
    INTERFACE
        # 让 #include "interfaces/types.hpp" 或 #include "types.hpp" 均可用
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mv-interfaces
    INTERFACE
        mv-core               # Logger + ConfigManager（接口的注释中使用到日志模式）
        opencv_core           # cv::Mat, cv::Point2f, cv::Rect2f
        yaml-cpp              # YAML::Node
        Eigen3::Eigen         # Eigen::Vector3d
)

message(STATUS "    [interfaces] mv-interfaces (INTERFACE, header-only)")
