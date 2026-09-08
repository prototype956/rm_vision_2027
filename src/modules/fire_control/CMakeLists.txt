add_library(mv-modules-fire-control STATIC
    control_session.cpp
    policy_observation.cpp
    rule_policy.cpp
    fire_control.cpp
    fire_control_config.cpp
    gimbal_feedback_estimator.cpp
)

target_include_directories(mv-modules-fire-control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-fire-control
    PUBLIC mv-core mv-hal-gimbal mv-prediction-model
           mv-modules-gimbal-trajectory-planner Eigen3::Eigen
    PRIVATE yaml-cpp
)

message(STATUS "    + mv-modules-fire-control (ballistics, slot selection and fire gating)")

set_property(TARGET mv-modules-fire-control PROPERTY POSITION_INDEPENDENT_CODE ON)
