# ============================================================================
# src/app/ — 应用入口目录
# ============================================================================
# 注意：mv-smoke-test 已迁移到 src/test/CMakeLists.txt

# ── Stage 6：主程序可执行文件 ───────────────────────────────────────────────
add_executable(mv-vision-main main.cpp)

target_link_libraries(mv-vision-main
    PRIVATE
        mv-core
        mv-fsm
        mv-pipeline
        mv-hal-camera
        mv-hal-serial
        mv-mod-armor-detector
        mv-mod-pnp-solver
        mv-mod-simple-predictor
        mv-mod-simple-voter
        mv-mod-rm-shooter
)

target_compile_definitions(mv-vision-main PRIVATE
    CONFIG_FILE_PATH="${CONFIG_FILE_PATH}"
)

message(STATUS "    + mv-vision-main (stage-6 main entry)")
