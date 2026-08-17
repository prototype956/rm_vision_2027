add_library(mv-tool-foxglove-control STATIC
    control_debug_publisher.cpp
)

target_include_directories(mv-tool-foxglove-control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-control
    PUBLIC mv-tool-foxglove-runtime mv-modules-fire-control
    PRIVATE mv-core fmt::fmt
)

message(STATUS "    + mv-tool-foxglove-control (bounded 100 Hz control diagnostics)")
