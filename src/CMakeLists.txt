# ============================================================================
# Active source modules
# ============================================================================

message(STATUS "  Configuring source modules...")

add_subdirectory(core)
add_subdirectory(geometry)
add_subdirectory(frame)
add_subdirectory(tool)

if(USE_OPENVINO)
    add_subdirectory(modules)
endif()

add_subdirectory(hal)

if(BUILD_MAIN)
    add_subdirectory(runtime)
    add_subdirectory(app)
endif()
