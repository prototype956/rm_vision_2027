# MindVision 实机长时验收：相机抓帧、同步检测、预览和资源统计。
add_executable(mv-armor-detector-test
    armor_detector_test_main.cpp
    armor_detector_test_application.cpp
)

target_include_directories(mv-armor-detector-test
    PRIVATE
        "${CMAKE_SOURCE_DIR}"
)

target_link_libraries(mv-armor-detector-test
    PRIVATE
        mv-core
        mv-hal-camera
        mv-modules-armor-detector
        mv-tool-armor-overlay
        mv-tool-debug
        opencv_imgcodecs
        opencv_imgproc
)

target_compile_definitions(mv-armor-detector-test PRIVATE
    CONFIG_FILE_PATH="${CONFIG_FILE_PATH}"
    PROJECT_ROOT_PATH="${CMAKE_SOURCE_DIR}"
)

set_target_properties(mv-armor-detector-test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# 离线视频入口：无需相机 SDK 运行时设备，用于回放、可视化和性能验收。
add_executable(mv-armor-detector-video-test
    armor_detector_video_test_main.cpp
)

target_link_libraries(mv-armor-detector-video-test
    PRIVATE
        mv-core
        mv-modules-armor-detector
        mv-tool-armor-overlay
        mv-tool-debug
        opencv_imgcodecs
        opencv_imgproc
        opencv_videoio
)

target_compile_definitions(mv-armor-detector-video-test PRIVATE
    CONFIG_FILE_PATH="${CONFIG_FILE_PATH}"
    PROJECT_ROOT_PATH="${CMAKE_SOURCE_DIR}"
)

set_target_properties(mv-armor-detector-video-test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "    + mv-armor-detector-test (MindVision + GPU endurance)")
message(STATUS "    + mv-armor-detector-video-test (offline video + benchmark)")
