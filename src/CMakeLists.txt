# ============================================================================
# Active source modules
# ============================================================================

message(STATUS "  Configuring source modules...")

add_subdirectory(core)
add_subdirectory(tool)

if(USE_OPENVINO)
    add_subdirectory(modules)
endif()

if(BUILD_MAIN)
    add_subdirectory(app)
endif()

add_subdirectory(hal)
