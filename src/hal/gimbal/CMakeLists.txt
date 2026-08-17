add_library(mv-hal-gimbal INTERFACE)

target_include_directories(mv-hal-gimbal INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

add_subdirectory(talos)
