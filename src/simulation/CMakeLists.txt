add_library(mv-simulation-data INTERFACE)

target_include_directories(mv-simulation-data INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-simulation-data INTERFACE mv-geometry)

message(STATUS "    + mv-simulation-data (simulation truth data contracts)")
