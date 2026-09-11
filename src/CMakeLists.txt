# ============================================================================
# Active source modules
# ============================================================================

message(STATUS "  Configuring source modules...")

add_subdirectory(core)
add_subdirectory(geometry)
add_subdirectory(simulation)
add_subdirectory(frame)
add_subdirectory(tool)

if(USE_ARMOR_DETECTOR)
    add_subdirectory(modules)
endif()

add_subdirectory(hal)

if(BUILD_MAIN)
    add_subdirectory(runtime)
    add_subdirectory(app)
endif()
