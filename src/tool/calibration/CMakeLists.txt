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

add_executable(mv-extrinsic-calibration
    extrinsic_calibration_main.cpp
    extrinsic_capture_application.cpp
    extrinsic_solve_application.cpp
    extrinsic_visualize_application.cpp
)

target_link_libraries(mv-extrinsic-calibration
    PRIVATE
        mv-core
        mv-hal-camera
        mv-tool-debug
        opencv_calib3d
        opencv_imgcodecs
        opencv_imgproc
)

if(USE_ARMOR_DETECTOR)
    target_link_libraries(mv-extrinsic-calibration
        PRIVATE mv-tool-foxglove mv-tool-foxglove-runtime foxglove_sdk
    )
    target_compile_definitions(mv-extrinsic-calibration PRIVATE MV_EXTRINSIC_HAS_FOXGLOVE=1)
endif()

target_compile_definitions(mv-extrinsic-calibration PRIVATE
    CONFIG_FILE_PATH="${CONFIG_FILE_PATH}"
    PROJECT_ROOT_PATH="${CMAKE_SOURCE_DIR}"
)

set_target_properties(mv-extrinsic-calibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "    + mv-extrinsic-calibration")
