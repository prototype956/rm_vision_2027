# ============================================================================
# src/ — 重构后的核心模块目录
# ============================================================================
# 各阶段子目录按需添加，未完成的阶段注释掉

message(STATUS "  Configuring refactored src modules...")

# 阶段一：基础设施
add_subdirectory(core)

# 主程序
if(BUILD_MAIN)
    add_subdirectory(app)
endif()

# 阶段二：硬件抽象层（HAL）
add_subdirectory(hal)

# 阶段三：接口层 + 工厂系统
add_subdirectory(interfaces)
add_subdirectory(factory)
add_subdirectory(modules)  # Stage 6：具体算法模块（装甲板检测器、解算器、预测器等）

# 阶段四：线程 Pipeline
add_subdirectory(pipeline)

# 阶段五：状态机
add_subdirectory(fsm)

# 调试工具库（ParamTuner / ViewRenderer / DebugSession）
add_subdirectory(tool)
