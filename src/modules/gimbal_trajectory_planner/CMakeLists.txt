add_library(mv-modules-gimbal-trajectory-planner STATIC
    detail/tinympc_workspace.cpp
    gimbal_trajectory_planner.cpp
    gimbal_trajectory_planner_config.cpp
)

target_include_directories(mv-modules-gimbal-trajectory-planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-gimbal-trajectory-planner
    PUBLIC mv-core mv-hal-gimbal Eigen3::Eigen
    PRIVATE mv-3rdparty-tinympc mv-modules-armor-predictor yaml-cpp
)

message(STATUS "    + mv-modules-gimbal-trajectory-planner (TinyMPC state feedback)")
