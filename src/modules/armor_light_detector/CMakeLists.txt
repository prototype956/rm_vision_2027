add_library(mv-modules-armor-light-detector STATIC
    armor_light_detector.cpp
    armor_light_detector_config.cpp
)

target_include_directories(mv-modules-armor-light-detector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-armor-light-detector
    PUBLIC
        mv-core
        mv-modules-armor-corner-refiner
        mv-modules-armor-detector
        opencv_core
        yaml-cpp
    PRIVATE
        opencv_imgproc
)

message(STATUS "    + mv-modules-armor-light-detector (full-frame OpenCV lightbars)")
