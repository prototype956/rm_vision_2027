add_library(mv-tool-web STATIC
    latest_preview.cpp
    web_assets.cpp
    web_debug_config.cpp
    web_debug_server.cpp
)

target_include_directories(mv-tool-web
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-web
    PUBLIC
        mv-core
        mv-runtime
    PRIVATE
        Boost::system
        nlohmann_json::nlohmann_json
        mv-tool-armor-overlay
        opencv_core
        opencv_imgcodecs
        opencv_imgproc
        Threads::Threads
        yaml-cpp
)

message(STATUS "    + mv-tool-web (authenticated HTTP tuning + JPEG preview)")
