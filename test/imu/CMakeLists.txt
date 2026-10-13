add_executable(mv-imu-axis-capture imu_axis_capture_main.cpp)
target_link_libraries(mv-imu-axis-capture PRIVATE mv-real-geometry-config mv-hal-serial Threads::Threads)
set_target_properties(mv-imu-axis-capture PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(mv-imu-drift imu_drift_main.cpp)
target_link_libraries(mv-imu-drift PRIVATE mv-real-geometry-config mv-hal-serial Threads::Threads)
set_target_properties(mv-imu-drift PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

foreach(imu_target mv-imu-axis-capture mv-imu-drift)
    target_compile_definitions(${imu_target} PRIVATE PROJECT_ROOT_PATH="${CMAKE_SOURCE_DIR}")
endforeach()
