# ============================================================================
# Reusable development and visualization tools
# ============================================================================

add_subdirectory(debug)

if(USE_OPENVINO)
    add_subdirectory(foxglove)
endif()
