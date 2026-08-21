add_library(mv-frame-data INTERFACE)

target_include_directories(mv-frame-data INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-frame-data INTERFACE mv-geometry mv-hal-gimbal opencv_core)

message(STATUS "    + mv-frame-data (captured frame + synchronized optional context)")
