# ============================================================================
# mv-hal — 硬件抽象层（Hardware Abstraction Layer）
# ============================================================================
#
# 相机、串口姿态与云台 HAL。

message(STATUS "  Configuring HAL modules...")

add_subdirectory(serial)
add_subdirectory(imu)
add_subdirectory(camera)
add_subdirectory(gimbal)
