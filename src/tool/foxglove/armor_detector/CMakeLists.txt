add_library(mv-tool-foxglove-armor-detector STATIC
    armor_channel_set.cpp
    armor_debug_component.cpp
    armor_message_encoder.cpp
    armor_publisher_metrics.cpp
    latest_frame_queue.cpp
)

target_include_directories(mv-tool-foxglove-armor-detector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-armor-detector
    PUBLIC
        mv-tool-foxglove-runtime
    PRIVATE
        mv-core
        mv-hal-camera
        mv-modules-armor-detector
        fmt::fmt
        opencv_imgcodecs
)
