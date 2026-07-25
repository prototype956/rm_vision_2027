# ============================================================================
# Active source modules
# ============================================================================

message(STATUS "  Configuring source modules...")

add_subdirectory(core)

if(BUILD_MAIN)
    add_subdirectory(app)
endif()

add_subdirectory(hal)
