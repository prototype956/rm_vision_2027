add_executable(mv-web-debug-test
    web_debug_test_main.cpp
)

target_link_libraries(mv-web-debug-test
    PRIVATE
        Boost::system
        mv-tool-web
        nlohmann_json::nlohmann_json
        opencv_core
        opencv_imgcodecs
        Threads::Threads
)

target_compile_definitions(mv-web-debug-test PRIVATE
    PROJECT_ROOT_PATH="${CMAKE_SOURCE_DIR}"
)

set_target_properties(mv-web-debug-test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "    + mv-web-debug-test (loopback HTTP + tuning transaction acceptance)")
