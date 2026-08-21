add_library(mv-modules-armor-predictor STATIC
    armor_predictor.cpp
    armor_predictor_config.cpp
    detail/armor_motion_model.cpp
    detail/error_state_ekf.cpp
    detail/image_observation.cpp
)

target_include_directories(mv-modules-armor-predictor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-armor-predictor
    PUBLIC mv-core mv-hal-camera mv-modules-armor-detector mv-modules-armor-light-detector
           mv-modules-armor-pnp Eigen3::Eigen
    PRIVATE yaml-cpp Ceres::ceres
)

message(STATUS "    + mv-modules-armor-predictor (13-state image ESEKF prediction)")
