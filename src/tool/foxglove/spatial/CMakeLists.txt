add_library(mv-tool-foxglove-spatial STATIC
    spatial_message_encoder.cpp
)

target_include_directories(mv-tool-foxglove-spatial
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-spatial PUBLIC foxglove_sdk mv-hal-camera mv-geometry)

message(STATUS "    + mv-tool-foxglove-spatial (TF + calibration + frustum)")
