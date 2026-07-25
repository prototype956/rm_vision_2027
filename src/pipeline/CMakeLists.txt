# ============================================================================
# mv-pipeline — 视觉流水线（Stage 4）
# ============================================================================
#
# 包含：
#   channel.hpp       — 带 Shutdown 的线程安全通道（header-only）
#   node.hpp          — PipelineNode 抽象基类（header-only）
#   packet.hpp        — 节点间数据包类型（header-only）
#   pipeline.hpp      — VisionPipeline 编排器（header-only，含 Builder）
#   capture_node.cpp  — 采集节点实现
#   detect_node.cpp   — 检测+解算节点实现
#   predict_node.cpp  — 预测+投票节点实现
#   serial_node.cpp   — 串口收发节点实现
#
# 依赖说明：
#   - mv-interfaces  → types.hpp、IDetector/ISolver/IPredictor/IVoter/IShooter
#   - mv-hal-camera  → ICamera
#   - mv-hal-serial  → ISerial
#   - mv-core        → Logger（MV_LOG_xxx 宏）
#   - Threads        → std::thread、std::mutex 等（系统线程库）

add_library(mv-pipeline STATIC
    capture_node.cpp
    detect_node.cpp
    predict_node.cpp
    serial_node.cpp
)

target_include_directories(mv-pipeline
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-pipeline
    PUBLIC
        mv-interfaces      # types.hpp + 所有业务接口（IDetector、ISolver 等）
        mv-hal-camera      # ICamera（CaptureNode 持有）
        mv-hal-serial      # ISerial（SerialNode 持有）
        mv-core            # Logger
    PRIVATE
        Threads::Threads   # pthread，std::thread 运行时
)

message(STATUS "    [pipeline] mv-pipeline (CaptureNode + DetectNode + PredictNode + SerialNode)")
