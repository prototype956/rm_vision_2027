# ============================================================================
# 依赖查找配置 - 使用系统包
# ============================================================================

message(STATUS "Finding dependencies from system packages...")

# ----------------------------------------------------------------------------
# 线程库
# ----------------------------------------------------------------------------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
message(STATUS "  ✓ Threads found")

# ----------------------------------------------------------------------------
# 系统包依赖 (通过 apt 安装)
# ----------------------------------------------------------------------------

# fmt - 格式化库
find_package(fmt REQUIRED)
message(STATUS "  ✓ fmt found: ${fmt_VERSION}")

# spdlog - 日志库  
find_package(spdlog REQUIRED)
message(STATUS "  ✓ spdlog found: ${spdlog_VERSION}")

# yaml-cpp - YAML 解析
find_package(yaml-cpp REQUIRED)
message(STATUS "  ✓ yaml-cpp found")

# Eigen - 内部三维几何与固定尺寸线性代数
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
message(STATUS "  ✓ Eigen found: ${Eigen3_VERSION}")

# OpenCV - 当前应用、HAL 与测试所需组件
find_package(OpenCV REQUIRED COMPONENTS
    core
    calib3d
    imgproc
    imgcodecs
    highgui
    videoio
)
message(STATUS "  ✓ OpenCV found: ${OpenCV_VERSION}")
message(STATUS "    OpenCV modules: ${OpenCV_LIBS}")

if(USE_OPENVINO)
    find_package(OpenVINO 2024.0 REQUIRED COMPONENTS Runtime)
    message(STATUS "  ✓ OpenVINO Runtime found: ${OpenVINO_VERSION}")
    find_package(Ceres 2.0 REQUIRED)
    message(STATUS "  ✓ Ceres found: ${Ceres_VERSION}")
else()
    message(STATUS "  ⊗ OpenVINO disabled")
endif()

# ----------------------------------------------------------------------------
# 检查关键 OpenCV 模块
# ----------------------------------------------------------------------------
set(REQUIRED_OPENCV_COMPONENTS 
    core calib3d imgproc imgcodecs highgui videoio
)

foreach(component ${REQUIRED_OPENCV_COMPONENTS})
    if(NOT TARGET opencv_${component})
        message(WARNING "OpenCV component '${component}' not found!")
    endif()
endforeach()

message(STATUS "All required dependencies found successfully!")
