# OpenVINO 0526 装甲检测静态库：公开 API、严格配置解析和模型后处理。
add_library(mv-modules-armor-detector STATIC
    armor_detector.cpp
    armor_detector_config.cpp
    armor_detector_postprocess.cpp
)

target_include_directories(mv-modules-armor-detector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-armor-detector
    PUBLIC
        mv-core
        opencv_core
        yaml-cpp
    PRIVATE
        openvino::runtime
        opencv_imgproc
)

message(STATUS "    + mv-modules-armor-detector (OpenVINO YOLO 0526)")
