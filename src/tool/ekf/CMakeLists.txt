# ============================================================================
# mv-tool-ekf — 扩展卡尔曼滤波器工具库（Stage 8-A）
# ============================================================================
#
# 独立为一个静态库的原因：
#   EkfPredictor 和未来可能的 BuffPredictor 均依赖此 EKF 核心，
#   抽出为独立库后只编译一次，Link 时各自引入，避免目标文件重复。

add_library(mv-tool-ekf STATIC
    extended_kalman_filter.cpp
)

target_include_directories(mv-tool-ekf
    PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
)

target_link_libraries(mv-tool-ekf
    PUBLIC
        Eigen3::Eigen
)

message(STATUS "    [tool] mv-tool-ekf (ExtendedKalmanFilter)")
