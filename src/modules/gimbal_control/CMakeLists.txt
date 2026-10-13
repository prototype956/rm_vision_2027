add_library(mv-modules-gimbal-types STATIC gimbal_trajectory_types.cpp)
target_include_directories(mv-modules-gimbal-types PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-modules-gimbal-types PUBLIC mv-hal-gimbal Eigen3::Eigen)

add_library(mv-modules-gimbal-pid STATIC pid/pid_rate_controller.cpp)
target_include_directories(mv-modules-gimbal-pid PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-modules-gimbal-pid
    PUBLIC mv-hal-gimbal Eigen3::Eigen yaml-cpp PRIVATE mv-core)

add_library(mv-modules-gimbal-mpc STATIC
    mpc/detail/tinympc_workspace.cpp
    mpc/mpc_trajectory_planner.cpp
    mpc/mpc_trajectory_planner_config.cpp
    mpc/mpc_rate_follower.cpp)
target_include_directories(mv-modules-gimbal-mpc PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-modules-gimbal-mpc PUBLIC mv-modules-gimbal-types Eigen3::Eigen yaml-cpp
    PRIVATE mv-core mv-3rdparty-tinympc)

add_library(mv-modules-gimbal-control STATIC
    gimbal_control_config.cpp gimbal_reference_planner.cpp)
target_include_directories(mv-modules-gimbal-control PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mv-modules-gimbal-control
    PUBLIC mv-modules-gimbal-pid mv-modules-gimbal-mpc PRIVATE mv-core)

set_target_properties(mv-modules-gimbal-types mv-modules-gimbal-pid mv-modules-gimbal-mpc
    mv-modules-gimbal-control PROPERTIES POSITION_INDEPENDENT_CODE ON)
