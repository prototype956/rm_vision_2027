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

# OpenCV - 当前应用、HAL 与测试所需组件
find_package(OpenCV REQUIRED COMPONENTS
    core
    imgproc
    imgcodecs
    highgui
    videoio
)
message(STATUS "  ✓ OpenCV found: ${OpenCV_VERSION}")
message(STATUS "    OpenCV modules: ${OpenCV_LIBS}")

# ----------------------------------------------------------------------------
# 检查关键 OpenCV 模块
# ----------------------------------------------------------------------------
set(REQUIRED_OPENCV_COMPONENTS 
    core imgproc imgcodecs highgui videoio
)

foreach(component ${REQUIRED_OPENCV_COMPONENTS})
    if(NOT TARGET opencv_${component})
        message(WARNING "OpenCV component '${component}' not found!")
    endif()
endforeach()

message(STATUS "All required dependencies found successfully!")
