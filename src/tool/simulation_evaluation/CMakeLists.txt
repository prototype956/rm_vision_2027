add_library(mv-tool-simulation-evaluation STATIC
    simulation_evaluation_config.cpp
    simulation_evaluator.cpp
)

target_include_directories(mv-tool-simulation-evaluation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-simulation-evaluation
    PUBLIC
        mv-frame-core
        mv-simulation-data
        mv-modules-armor-corner-refiner
        mv-modules-armor-detector
        mv-modules-armor-pnp
        mv-modules-armor-predictor
    PRIVATE
        mv-core
        opencv_imgproc
        yaml-cpp
)

message(STATUS "    + mv-tool-simulation-evaluation (truth matching + accuracy metrics)")
