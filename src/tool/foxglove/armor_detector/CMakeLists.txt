add_library(mv-tool-foxglove-armor-detector STATIC
    armor_message_encoder.cpp
)

target_include_directories(mv-tool-foxglove-armor-detector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-armor-detector
    PUBLIC
        foxglove_sdk
        mv-modules-armor-detector
    PRIVATE
        fmt::fmt
)

message(STATUS "    + mv-tool-foxglove-armor-detector (annotations + detector stats)")
