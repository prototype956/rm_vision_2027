# ============================================================================
# mv-hal — 硬件抽象层（Hardware Abstraction Layer）
# ============================================================================
#
# 分拆为两个独立静态库：
#   mv-hal-camera  — 相机 HAL（ICamera + 具体实现）
#   mv-hal-serial  — 串口 HAL（ISerial + 具体实现）
#
# 为什么分开而不合并？
#   - 不需要相机的模块（仅串口）不必链接 OpenCV；
#   - 有 MindVision SDK 的目标和没有的目标可以独立控制；
#   - CI 机器上可以只编译 serial，不需要任何相机 SDK。
#
# 外部使用方式（示例）：
#   target_link_libraries(my_target PRIVATE mv-hal-camera mv-hal-serial)

message(STATUS "  Configuring HAL modules...")

add_subdirectory(camera)
add_subdirectory(serial)
