add_library(mv-modules-armor-predictor STATIC
    armor_predictor.cpp
    armor_predictor_config.cpp
    detail/armor_association.cpp
    detail/armor_ekf.cpp
    detail/four_armor_model.cpp
)

target_include_directories(mv-modules-armor-predictor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-armor-predictor
    PUBLIC mv-core mv-hal-camera mv-modules-armor-detector mv-modules-armor-pnp Eigen3::Eigen
    PRIVATE yaml-cpp
)

message(STATUS "    + mv-modules-armor-predictor (four-armor EKF prediction)")
