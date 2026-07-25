# ============================================================================
# mv-hal — 硬件抽象层（Hardware Abstraction Layer）
# ============================================================================
#
# 当前只构建相机 HAL。serial/ 仅保留为空目录，等待重新设计。

message(STATUS "  Configuring HAL modules...")

add_subdirectory(camera)
