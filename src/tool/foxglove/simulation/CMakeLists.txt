add_library(mv-tool-foxglove-simulation STATIC
    simulation_message_encoder.cpp
)

target_include_directories(mv-tool-foxglove-simulation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-simulation
    PUBLIC
        foxglove_sdk
        mv-hal-camera
        mv-geometry
    PRIVATE
        fmt::fmt
)

message(STATUS "    + mv-tool-foxglove-simulation (ground truth + projections)")
