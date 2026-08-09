add_subdirectory(runtime)
add_subdirectory(image)
add_subdirectory(armor_detector)
add_subdirectory(spatial)
add_subdirectory(simulation)
add_subdirectory(pnp)
add_subdirectory(pipeline)

add_library(mv-tool-foxglove STATIC
    vision_debug_publisher.cpp
    foxglove_config.cpp
)

target_include_directories(mv-tool-foxglove
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove
    PUBLIC
        mv-core
        mv-hal-camera
        mv-modules-armor-detector
        mv-modules-armor-pnp
    PRIVATE
        mv-tool-foxglove-pipeline
        mv-tool-foxglove-runtime
)

message(STATUS "    + mv-tool-foxglove (WebSocket + MCAP vision diagnostics)")
