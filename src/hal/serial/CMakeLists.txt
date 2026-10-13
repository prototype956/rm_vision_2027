# 串口设备、线协议、会话和时间同步；不依赖相机、IMU 数学类型或帧数据。
add_library(mv-hal-serial STATIC controller_link.cpp)
target_include_directories(mv-hal-serial PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-hal-serial PRIVATE mv-core Threads::Threads)

# 与基础链路分开构建，避免 mv-hal-imu 与串口库相互依赖。
add_library(mv-hal-gimbal-serial STATIC serial_gimbal_command_sink.cpp)
target_include_directories(mv-hal-gimbal-serial PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-hal-gimbal-serial PUBLIC mv-hal-gimbal mv-hal-imu
    PRIVATE mv-core yaml-cpp)
