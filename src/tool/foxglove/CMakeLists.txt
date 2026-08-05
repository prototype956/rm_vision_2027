add_subdirectory(runtime)
add_subdirectory(armor_detector)

add_library(mv-tool-foxglove STATIC
    armor_debug_publisher.cpp
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
    PRIVATE
        mv-tool-foxglove-armor-detector
        mv-tool-foxglove-runtime
)

message(STATUS "    + mv-tool-foxglove (WebSocket + MCAP armor diagnostics)")
