# ============================================================================
# Reusable development and visualization tools
# ============================================================================

add_subdirectory(debug)
add_subdirectory(calibration)

if(USE_OPENVINO)
    add_subdirectory(foxglove)
endif()
