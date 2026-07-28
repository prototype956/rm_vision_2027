add_library(mv-tool-debug STATIC
    debug_window.cpp
)

target_include_directories(mv-tool-debug
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-debug
    PUBLIC
        opencv_core
    PRIVATE
        opencv_highgui
)

message(STATUS "    + mv-tool-debug (reusable OpenCV debug window)")

if(USE_OPENVINO)
    add_library(mv-tool-armor-overlay STATIC
        armor_detection_overlay.cpp
    )

    target_include_directories(mv-tool-armor-overlay
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:include>
    )

    target_link_libraries(mv-tool-armor-overlay
        PUBLIC
            mv-modules-armor-detector
            opencv_core
        PRIVATE
            fmt::fmt
            opencv_imgproc
    )

    message(STATUS "    + mv-tool-armor-overlay (armor detection visualization)")
endif()
