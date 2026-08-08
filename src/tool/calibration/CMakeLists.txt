add_library(mv-tool-calibration STATIC
    calibration.cpp
)

target_include_directories(mv-tool-calibration
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-calibration
    PUBLIC
        mv-core
        mv-hal-camera
        opencv_core
        yaml-cpp
    PRIVATE
        opencv_calib3d
        opencv_imgproc
)

add_executable(mv-camera-calibration
    calibration_main.cpp
    calibration_application.cpp
)

target_link_libraries(mv-camera-calibration
    PRIVATE
        mv-core
        mv-hal-camera
        mv-tool-calibration
        mv-tool-debug
        opencv_calib3d
        opencv_imgproc
        opencv_imgcodecs
)

target_compile_definitions(mv-camera-calibration PRIVATE
    CONFIG_FILE_PATH="${CONFIG_FILE_PATH}"
    PROJECT_ROOT_PATH="${CMAKE_SOURCE_DIR}"
)

set_target_properties(mv-camera-calibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "    + mv-tool-calibration + mv-camera-calibration")
