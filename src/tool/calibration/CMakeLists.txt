# ============================================================================
# src/tool/calibration/ - Calibration tool executables (scaffold v1)
# ============================================================================

add_library(mv-tool-calibration-common STATIC
    calibration_io.cpp
)

target_include_directories(mv-tool-calibration-common
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mv-tool-calibration-common
    PUBLIC
        fmt::fmt
        yaml-cpp
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

add_executable(mv-calib-capture
    capture_calib_data.cpp
)

target_link_libraries(mv-calib-capture
    PRIVATE
        mv-tool-calibration-common
    mv-hal-serial
)

add_executable(mv-calib-camera
    calibrate_camera.cpp
)

target_link_libraries(mv-calib-camera
    PRIVATE
        mv-tool-calibration-common
)

add_executable(mv-calib-rwhandeye
    calibrate_robotworld_handeye.cpp
)

target_link_libraries(mv-calib-rwhandeye
    PRIVATE
        mv-tool-calibration-common
)

add_executable(mv-calib-validate
    validate_calibration.cpp
)

target_link_libraries(mv-calib-validate
    PRIVATE
        mv-tool-calibration-common
)

message(STATUS "      + mv-calib-capture (capture calibration frames scaffold)")
message(STATUS "      + mv-calib-camera (intrinsics: chessboard/circles)")
message(STATUS "      + mv-calib-rwhandeye (RobotWorld-HandEye main flow)")
message(STATUS "      + mv-calib-validate (det/orthogonality/reprojection checks)")
