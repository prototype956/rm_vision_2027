# ============================================================================
# mv-hal-camera — 相机硬件抽象层
# ============================================================================
#
# 包含：
#   ICamera         — 纯虚接口（header-only）
#   OpenCvCamera    — opencv/ 下的 USB/文件/RTSP 相机（始终编译）
#   TalosCamera     — talos/ 下的 Daedalus 共享内存相机（始终编译）
#   MindVisionCamera — 工业相机（USE_MINDVISION_SDK=ON 时编译完整版，
#                       否则编译为"始终返回 false"的桩）
#
# MindVision 的桩始终保留相同的类和链接接口，使项目可以在未启用 SDK
# 的环境中完成编译；此时 Open() 会明确返回失败。

add_library(mv-hal-camera STATIC
    camera_factory.cpp
    opencv/opencv_camera.cpp
    talos/talos_camera.cpp
    talos/talos_config.cpp
    talos/talos_device.cpp
    mindvision/mindvision_camera.cpp
    mindvision/mindvision_config.cpp
    mindvision/mindvision_device.cpp
)

target_include_directories(mv-hal-camera
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-hal-camera
    PUBLIC
        mv-core               # 依赖 Logger 和 ConfigLoader
        mv-geometry           # CameraFrame 公开三维几何类型
        opencv_core           # cv::Mat 类型（接口中使用，故 PUBLIC）
        yaml-cpp
    PRIVATE
        opencv_videoio        # cv::VideoCapture（仅 .cpp 使用，故 PRIVATE）
        opencv_imgproc        # Talos RGB8 兼容转换（仅 .cpp 使用）
)

# MindVision SDK 条件编译
if(USE_MINDVISION_SDK AND TARGET MVSDK)
    target_compile_definitions(mv-hal-camera PRIVATE MV_HAS_MVSDK)
    target_link_libraries(mv-hal-camera PRIVATE MVSDK)
    message(STATUS "    + mv-hal-camera (OpenCvCamera + MindVisionCamera [SDK])")
else()
    message(STATUS "    + mv-hal-camera (OpenCvCamera + MindVisionCamera [stub])")
endif()
