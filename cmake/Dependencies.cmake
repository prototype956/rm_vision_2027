# ============================================================================
# 依赖查找配置 - 使用系统包 (不再使用 vcpkg)
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

# nlohmann_json - JSON 解析
find_package(nlohmann_json 3.10 REQUIRED)
message(STATUS "  ✓ nlohmann_json found")

# Eigen3 - 线性代数
find_package(Eigen3 REQUIRED)
message(STATUS "  ✓ Eigen3 found: ${Eigen3_VERSION}")

# TBB - 并行计算
find_package(TBB REQUIRED)
message(STATUS "  ✓ TBB found")

# OpenCV - 计算机视觉
find_package(OpenCV REQUIRED)
message(STATUS "  ✓ OpenCV found: ${OpenCV_VERSION}")
message(STATUS "    OpenCV modules: ${OpenCV_LIBS}")

# ----------------------------------------------------------------------------
# 检查关键 OpenCV 模块
# ----------------------------------------------------------------------------
set(REQUIRED_OPENCV_COMPONENTS 
    core imgproc imgcodecs highgui videoio calib3d dnn
)

foreach(component ${REQUIRED_OPENCV_COMPONENTS})
    if(NOT TARGET opencv_${component})
        message(WARNING "OpenCV component '${component}' not found!")
    endif()
endforeach()

message(STATUS "All vcpkg dependencies found successfully!")
