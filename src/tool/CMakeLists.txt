# ============================================================================
# Reusable development and visualization tools
# ============================================================================

add_subdirectory(debug)
add_subdirectory(calibration)

if(USE_ARMOR_DETECTOR)
    add_subdirectory(simulation_evaluation)
    add_subdirectory(foxglove)
    add_subdirectory(web)
endif()
