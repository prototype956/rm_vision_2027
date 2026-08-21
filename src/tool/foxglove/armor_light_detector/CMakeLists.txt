add_library(mv-tool-foxglove-armor-light-detector STATIC
    lightbar_message_encoder.cpp
)

target_include_directories(mv-tool-foxglove-armor-light-detector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-armor-light-detector
    PUBLIC
        foxglove_sdk
        mv-modules-armor-light-detector
        mv-modules-armor-predictor
    PRIVATE
        fmt::fmt
)

message(STATUS "    + mv-tool-foxglove-armor-light-detector (lightbar annotations + stats)")
