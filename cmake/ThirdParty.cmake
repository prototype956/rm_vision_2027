# ============================================================================
# Repository-provided third-party dependencies
# ============================================================================

message(STATUS "Configuring third-party dependencies...")

if(USE_OPENVINO)
    add_subdirectory(3rdparty/tinympc)
    message(STATUS "  ✓ TinyMPC enabled")
endif()

# ----------------------------------------------------------------------------
# Foxglove SDK - WebSocket 与 MCAP 调试输出
# ----------------------------------------------------------------------------
if(USE_OPENVINO AND EXISTS "${CMAKE_SOURCE_DIR}/3rdparty/foxglove/CMakeLists.txt")
    add_subdirectory(3rdparty/foxglove)
    message(STATUS "  ✓ Foxglove SDK enabled")
elseif(USE_OPENVINO)
    message(FATAL_ERROR "Foxglove SDK directory not found: ${CMAKE_SOURCE_DIR}/3rdparty/foxglove")
else()
    message(STATUS "  ⊗ Foxglove SDK disabled with OpenVINO")
endif()

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
