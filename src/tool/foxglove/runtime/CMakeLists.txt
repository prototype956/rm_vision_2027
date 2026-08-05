add_library(mv-tool-foxglove-runtime STATIC
    foxglove_session.cpp
    subscription_registry.cpp
)

target_include_directories(mv-tool-foxglove-runtime
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-tool-foxglove-runtime
    PUBLIC
        foxglove_sdk
    PRIVATE
        mv-core
)
