# ============================================================================
# mv-fsm — 视觉系统状态机（Stage 5）
# ============================================================================
#
# 包含：
#   state_machine.hpp — 通用 FSM 模板（header-only，纯标准库）
#   vision_fsm.hpp    — VisionFSM 类声明 + SystemState 枚举 + SystemContext
#   vision_fsm.cpp    — VisionFSM 实现 + 状态处理器实现
#
# 依赖说明：
#   - mv-pipeline  → VisionPipeline（FSM 持有其所有权）
#   - mv-core      → Logger（MV_LOG_xxx 宏）

add_library(mv-fsm STATIC
    vision_fsm.cpp
)

target_include_directories(mv-fsm
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-fsm
    PUBLIC
        mv-pipeline        # VisionPipeline（传入 VisionFSM 构造函数）
        mv-core            # Logger
)

target_compile_features(mv-fsm PUBLIC cxx_std_20)
