# 实机算法与传输的运行时组合；HAL 无需依赖 modules。
add_library(mv-runtime-gimbal-control STATIC
    ${PROJECT_SOURCE_DIR}/src/runtime/serial_gimbal_control_adapter.cpp
    ${PROJECT_SOURCE_DIR}/src/runtime/gimbal_tuning_reload.cpp)
target_include_directories(mv-runtime-gimbal-control PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-runtime-gimbal-control
    PUBLIC mv-modules-gimbal-control mv-hal-gimbal-serial PRIVATE mv-core Threads::Threads)
