add_library(mv-tool-foxglove-pnp STATIC pnp_message_encoder.cpp)
target_include_directories(mv-tool-foxglove-pnp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(mv-tool-foxglove-pnp PUBLIC foxglove_sdk mv-hal-camera mv-modules-armor-pnp PRIVATE fmt::fmt)
message(STATUS "    + mv-tool-foxglove-pnp (PnP scene + annotations + metrics)")
