add_library(mv-hal-imu STATIC serial_imu.cpp)
target_include_directories(mv-hal-imu PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-hal-imu PUBLIC mv-core mv-frame-core PRIVATE mv-hal-serial)
