add_executable(mv-imu-axis-capture imu_axis_capture_main.cpp)
target_link_libraries(mv-imu-axis-capture PRIVATE mv-hal-serial Threads::Threads)
set_target_properties(mv-imu-axis-capture PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
