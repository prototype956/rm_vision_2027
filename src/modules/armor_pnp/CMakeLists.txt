add_library(mv-modules-armor-pnp STATIC
    armor_pnp.cpp
    armor_pnp_config.cpp
    detail/ippe_solver.cpp
    detail/pnp_metrics.cpp
    detail/truth_evaluator.cpp
)

target_include_directories(mv-modules-armor-pnp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-armor-pnp
    PUBLIC mv-core mv-hal-camera mv-modules-armor-detector mv-modules-armor-corner-refiner
    PRIVATE opencv_calib3d opencv_imgproc yaml-cpp
)

message(STATUS "    + mv-modules-armor-pnp (IPPE armor pose estimation)")
