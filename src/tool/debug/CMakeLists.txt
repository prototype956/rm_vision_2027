add_library(mv-tool-debug STATIC
    debug_window.cpp
)

target_include_directories(mv-tool-debug
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-debug
    PUBLIC
        opencv_core
    PRIVATE
        opencv_highgui
)

message(STATUS "    + mv-tool-debug (reusable OpenCV debug window)")
