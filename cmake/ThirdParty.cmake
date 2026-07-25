# ============================================================================
# Repository-provided third-party dependencies
# ============================================================================

message(STATUS "Configuring third-party dependencies...")

# ----------------------------------------------------------------------------
# MindVision SDK - 迈德威视相机 SDK
# ----------------------------------------------------------------------------
if(USE_MINDVISION_SDK)
    message(STATUS "  Configuring MindVision SDK...")

    if(EXISTS "${CMAKE_SOURCE_DIR}/3rdparty/mindvision")
        add_subdirectory(3rdparty/mindvision)
        message(STATUS "  ✓ MindVision SDK enabled")
    else()
        message(WARNING "  ⚠ MindVision SDK directory not found, disabling...")
        set(USE_MINDVISION_SDK OFF CACHE BOOL "MindVision SDK disabled" FORCE)
    endif()
else()
    message(STATUS "  ⊗ MindVision SDK disabled")
endif()

message(STATUS "Third-party configuration completed!")
