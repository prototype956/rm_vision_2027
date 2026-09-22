# 串口设备、线协议、会话和时间同步；不依赖相机、IMU 数学类型或帧数据。
add_library(mv-hal-serial STATIC controller_link.cpp)
target_include_directories(mv-hal-serial PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-hal-serial PRIVATE Threads::Threads)
