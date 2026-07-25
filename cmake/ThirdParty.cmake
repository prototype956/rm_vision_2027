# ============================================================================
# 第三方库配置 - 非 vcpkg 管理的依赖
# ============================================================================

message(STATUS "Configuring third-party dependencies...")

# ----------------------------------------------------------------------------
# 选项定义
# ----------------------------------------------------------------------------
option(USE_MINDVISION_SDK "Enable MindVision Camera SDK (for real camera)" ON)
option(USE_FOXGLOVE_SDK "Enable Foxglove WebSocket Publisher (for debugging)" ON)
option(USE_OPENVINO "Enable OpenVINO inference (traditional vision without ONNX)" OFF)
option(USE_ONNXRUNTIME "Enable ONNX Runtime inference (traditional vision without OpenVINO)" OFF)

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
    message(STATUS "  ⊗ MindVision SDK disabled (video debugging mode)")
endif()

# ----------------------------------------------------------------------------
# Foxglove SDK - WebSocket 可视化调试
# ----------------------------------------------------------------------------
if(USE_FOXGLOVE_SDK)
    message(STATUS "  Configuring Foxglove SDK...")

    if(EXISTS "${CMAKE_SOURCE_DIR}/3rdparty/foxglove")
        add_subdirectory(3rdparty/foxglove)
        message(STATUS "  ✓ Foxglove SDK enabled")
    else()
        message(WARNING "  ⚠ Foxglove SDK directory not found, disabling...")
        set(USE_FOXGLOVE_SDK OFF CACHE BOOL "Foxglove SDK disabled" FORCE)
    endif()
else()
    message(STATUS "  ⊗ Foxglove SDK disabled (no web debugging)")
endif()

# ----------------------------------------------------------------------------
# OpenVINO - Intel 推理引擎 (可选)
# ----------------------------------------------------------------------------
if(USE_OPENVINO)
    message(STATUS "  Configuring OpenVINO...")

    # 允许用户自定义路径
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake"
        CACHE PATH "OpenVINO installation directory")

    find_package(OpenVINO QUIET COMPONENTS Runtime)

    if(OpenVINO_FOUND)
        message(STATUS "  ✓ OpenVINO found: ${OpenVINO_VERSION}")
        message(STATUS "    OpenVINO path: ${OpenVINO_DIR}")
        add_compile_definitions(ENABLE_OPENVINO)
    else()
        message(WARNING "  ⚠ OpenVINO not found at ${OpenVINO_DIR}")
        message(WARNING "    Please set OpenVINO_DIR or disable USE_OPENVINO")
        set(USE_OPENVINO OFF CACHE BOOL "OpenVINO disabled" FORCE)
    endif()
else()
    message(STATUS "  ⊗ OpenVINO disabled")
endif()

# ----------------------------------------------------------------------------
# ONNX Runtime - 推理引擎 (可选)
# ----------------------------------------------------------------------------
if(USE_ONNXRUNTIME)
    message(STATUS "  Configuring ONNX Runtime...")

    # 允许用户自定义路径
    set(ONNXRUNTIME_ROOT_PATH "/usr/local"
        CACHE PATH "ONNX Runtime installation root")

    set(ONNXRUNTIME_INCLUDE_DIRS
        "${ONNXRUNTIME_ROOT_PATH}/include/onnxruntime"
        "${ONNXRUNTIME_ROOT_PATH}/include"
    )
    set(ONNXRUNTIME_LIB "${ONNXRUNTIME_ROOT_PATH}/lib/libonnxruntime.so")

    if(EXISTS ${ONNXRUNTIME_LIB})
        message(STATUS "  ✓ ONNX Runtime found: ${ONNXRUNTIME_LIB}")
        add_compile_definitions(ENABLE_ONNXRUNTIME)
    else()
        message(WARNING "  ⚠ ONNX Runtime library not found at ${ONNXRUNTIME_LIB}")
        message(WARNING "    Please set ONNXRUNTIME_ROOT_PATH or disable USE_ONNXRUNTIME")
        set(USE_ONNXRUNTIME OFF CACHE BOOL "ONNX Runtime disabled" FORCE)
    endif()
else()
    message(STATUS "  ⊗ ONNX Runtime disabled")
endif()

# ----------------------------------------------------------------------------
# 冲突检查
# ----------------------------------------------------------------------------
if(USE_OPENVINO AND USE_ONNXRUNTIME)
    message(WARNING "Both OpenVINO and ONNX Runtime are enabled!")
    message(WARNING "This may cause conflicts. Consider using only one.")
endif()

# ----------------------------------------------------------------------------
# TinyMPC - 嵌入式 MPC 求解器（用于 MpcVoter 开火决策）
# ----------------------------------------------------------------------------
# 使用场景：MpcVoter 调用 TinyMPC 对 yaw/pitch 做 100 步（1s）在线轨迹规划，
#   判断 HALF_HORIZON（0.5s）处能否对准目标，从而决策是否允许开火。
# 来源：从 sp_vision_25/tasks/auto_aim/planner/tinympc/ 复制并适配。
if(EXISTS "${CMAKE_SOURCE_DIR}/3rdparty/tinympc")
    add_subdirectory(3rdparty/tinympc)
    message(STATUS "  ✓ TinyMPC enabled (mv-3rdparty-tinympc)")
else()
    message(WARNING "  ⚠ TinyMPC directory not found at 3rdparty/tinympc, MpcVoter will not compile")
endif()

message(STATUS "Third-party configuration completed!")
