add_library(mv-modules-fire-control STATIC
    fire_control.cpp
    fire_control_config.cpp
    gimbal_feedback_estimator.cpp
)

target_include_directories(mv-modules-fire-control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-fire-control
    PUBLIC mv-core mv-hal-camera mv-hal-gimbal mv-modules-armor-predictor
           mv-modules-gimbal-trajectory-planner Eigen3::Eigen
    PRIVATE yaml-cpp
)

message(STATUS "    + mv-modules-fire-control (ballistics, slot selection and fire gating)")
