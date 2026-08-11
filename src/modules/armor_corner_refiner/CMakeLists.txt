add_library(mv-modules-armor-corner-refiner STATIC
    armor_corner_refiner.cpp
)

target_include_directories(mv-modules-armor-corner-refiner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-armor-corner-refiner
    PUBLIC mv-core mv-hal-camera mv-modules-armor-detector
    PRIVATE opencv_core opencv_imgproc yaml-cpp
)

message(STATUS "    + mv-modules-armor-corner-refiner (shadow light-bar endpoint strategies)")
