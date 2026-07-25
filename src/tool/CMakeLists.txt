# ============================================================================
# src/tool/ — 工具模块集合
# ============================================================================
# 各功能子目录独立构建，以后可在此处添加更多工具模块：
#   debug/   ── 离线调试工具库（DebugSession / ParamTuner / ViewRenderer）
#   record/  ── 未来：视频录制工具（预留）
#   replay/  ── 未来：回放分析工具（预留）

add_subdirectory(debug)
add_subdirectory(ekf)
add_subdirectory(calibration)
if(USE_FOXGLOVE_SDK)
    add_subdirectory(foxglove)
else()
    message(STATUS "      ⊗ mv-tool-foxglove skipped (USE_FOXGLOVE_SDK=OFF)")
endif()
