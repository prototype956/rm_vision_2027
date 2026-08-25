add_library(mv-frame-core INTERFACE)

target_include_directories(mv-frame-core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-frame-core INTERFACE mv-geometry opencv_core)

add_library(mv-frame-data INTERFACE)
target_link_libraries(mv-frame-data INTERFACE
    mv-frame-core
    mv-hal-gimbal
    mv-simulation-data
)

message(STATUS "    + mv-frame-core (formal captured frame + spatial context)")
message(STATUS "    + mv-frame-data (synchronized integration frame packet)")
