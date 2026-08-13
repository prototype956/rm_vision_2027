add_library(mv-tool-foxglove-prediction STATIC prediction_message_encoder.cpp)

target_include_directories(mv-tool-foxglove-prediction PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-prediction
    PUBLIC foxglove_sdk mv-hal-camera mv-modules-armor-predictor
    PRIVATE fmt::fmt
)

message(STATUS "    + mv-tool-foxglove-prediction (EKF scene + state + truth overlay)")
