add_library(mv-runtime STATIC
    control_runtime.cpp
    control_runtime_loop.cpp
    runtime_error_policy.cpp
    runtime_supervisor.cpp
    vision_pipeline.cpp
    vision_tuning.cpp
    vision_runtime.cpp
    real_frame_geometry.cpp
    real_geometry_config.cpp
)

target_include_directories(mv-runtime
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(mv-runtime
    PUBLIC
        mv-hal-imu
        mv-hal-camera
        mv-hal-gimbal
        mv-frame-core
        mv-modules-armor-corner-refiner
        mv-modules-armor-detector
        mv-modules-armor-light-detector
        mv-modules-armor-pnp
        mv-modules-armor-predictor
        mv-modules-fire-control
        mv-modules-gimbal-trajectory-planner
    PRIVATE
        mv-core
        mv-tool-armor-overlay
        mv-tool-debug
        mv-tool-simulation-evaluation
        fmt::fmt
        opencv_imgproc
)

message(STATUS "    + mv-runtime (vision pipeline + fixed-rate control)")
