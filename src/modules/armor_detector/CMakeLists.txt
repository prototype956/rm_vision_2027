# 双后端 0526 装甲检测静态库：公开 API、严格配置解析和模型后处理。
add_library(mv-modules-armor-detector STATIC
    armor_detector.cpp
    armor_inference_backend.cpp
    armor_detector_config.cpp
    armor_detector_postprocess.cpp
)

target_include_directories(mv-modules-armor-detector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-modules-armor-detector
    PUBLIC
        mv-core
        opencv_core
        yaml-cpp
    PRIVATE
        opencv_imgproc
)

target_compile_definitions(mv-modules-armor-detector PRIVATE
    MV_USE_OPENVINO=$<BOOL:${USE_OPENVINO}>
    MV_USE_TENSORRT=$<BOOL:${USE_TENSORRT}>
)
if(USE_OPENVINO)
    target_sources(mv-modules-armor-detector PRIVATE openvino_backend.cpp)
    target_link_libraries(mv-modules-armor-detector PRIVATE openvino::runtime)
endif()
if(USE_TENSORRT)
    target_sources(mv-modules-armor-detector PRIVATE tensorrt_backend.cpp)
    target_link_libraries(mv-modules-armor-detector PRIVATE TensorRT::nvinfer TensorRT::nvonnxparser CUDA::cudart)
endif()
message(STATUS "    + mv-modules-armor-detector (OpenVINO=${USE_OPENVINO}, TensorRT=${USE_TENSORRT})")
