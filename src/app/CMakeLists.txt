add_executable(mv-vision-main
    main.cpp
    camera_preview.cpp
)

target_link_libraries(mv-vision-main
    PRIVATE
        mv-core
        mv-hal-camera
        opencv_highgui
)

target_compile_definitions(mv-vision-main PRIVATE
    CONFIG_FILE_PATH="${CONFIG_FILE_PATH}"
)

set_target_properties(mv-vision-main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "    + mv-vision-main (fixed MindVision preview)")
