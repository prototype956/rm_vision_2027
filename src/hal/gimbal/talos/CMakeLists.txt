add_library(mv-hal-gimbal-talos STATIC
    talos_gimbal_command_sink.cpp
)

target_include_directories(mv-hal-gimbal-talos PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-hal-gimbal-talos PUBLIC mv-hal-gimbal PRIVATE mv-core yaml-cpp)

message(STATUS "    + mv-hal-gimbal-talos (Talos v5 command triple buffer)")
