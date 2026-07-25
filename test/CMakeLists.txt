add_executable(mv-camera-test
    camera_test_main.cpp
    camera_test_application.cpp
)

target_include_directories(mv-camera-test
    PRIVATE
        "${CMAKE_SOURCE_DIR}"
)

target_link_libraries(mv-camera-test
    PRIVATE
        mv-core
        mv-hal-camera
        opencv_highgui
        opencv_imgproc
        opencv_imgcodecs
)

target_compile_definitions(mv-camera-test PRIVATE
    CONFIG_FILE_PATH="${CONFIG_FILE_PATH}"
)

set_target_properties(mv-camera-test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "    + mv-camera-test (full camera diagnostics)")
