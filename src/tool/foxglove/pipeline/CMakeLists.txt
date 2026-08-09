add_library(mv-tool-foxglove-pipeline STATIC
    latest_frame_queue.cpp
    vision_channel_set.cpp
    vision_debug_pipeline.cpp
    vision_message_encoder.cpp
    vision_publisher_metrics.cpp
)

target_include_directories(mv-tool-foxglove-pipeline
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-pipeline
    PUBLIC
        mv-tool-foxglove-runtime
    PRIVATE
        mv-core
        mv-hal-camera
        mv-modules-armor-detector
        mv-tool-foxglove-image
        mv-tool-foxglove-armor-detector
        mv-tool-foxglove-spatial
        mv-tool-foxglove-simulation
        fmt::fmt
)

message(STATUS "    + mv-tool-foxglove-pipeline (atomic vision debug batches)")
