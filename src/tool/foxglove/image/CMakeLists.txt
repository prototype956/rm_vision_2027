add_library(mv-tool-foxglove-image STATIC
    image_message_encoder.cpp
)

target_include_directories(mv-tool-foxglove-image
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-image
    PUBLIC
        foxglove_sdk
        opencv_core
    PRIVATE
        opencv_imgcodecs
)

message(STATUS "    + mv-tool-foxglove-image (JPEG encoding)")
