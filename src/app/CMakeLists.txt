add_executable(mv-vision-main
    main.cpp
)

target_link_libraries(mv-vision-main
    PRIVATE
        mv-core
        mv-hal-camera
        mv-modules-armor-detector
        mv-modules-armor-pnp
        mv-tool-armor-overlay
        mv-tool-debug
        mv-tool-foxglove
        opencv_imgproc
)

target_compile_definitions(mv-vision-main PRIVATE
    CONFIG_FILE_PATH="${CONFIG_FILE_PATH}"
    PROJECT_ROOT_PATH="${CMAKE_SOURCE_DIR}"
)

set_target_properties(mv-vision-main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "    + mv-vision-main (configured camera + YOLO 0526)")
