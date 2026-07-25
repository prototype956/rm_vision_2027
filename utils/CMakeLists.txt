# ============================================================================
# Utils 工具库
# ============================================================================

message(STATUS "  Configuring utils libraries...")

# ----------------------------------------------------------------------------
# mv-img-tools - 图像处理工具
# ----------------------------------------------------------------------------
add_library(mv-img-tools SHARED img_tools.cpp)

target_link_libraries(mv-img-tools
    PUBLIC
    opencv_core
    opencv_imgproc
    fmt::fmt
)

target_include_directories(mv-img-tools PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/utils>
)

# ----------------------------------------------------------------------------
# mv-plotter - 数据绘图工具
# ----------------------------------------------------------------------------
add_library(mv-plotter SHARED plotter.cpp)

target_link_libraries(mv-plotter
    PUBLIC
    nlohmann_json::nlohmann_json
    fmt::fmt
)

target_include_directories(mv-plotter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/utils>
)

# ----------------------------------------------------------------------------
# mv-logger - 日志工具 (OBJECT 库)
# ----------------------------------------------------------------------------
add_library(mv-logger OBJECT logger.cpp)

target_link_libraries(mv-logger PUBLIC
    spdlog::spdlog
    fmt::fmt
)

target_include_directories(mv-logger PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# OBJECT 库需要设置 POSITION_INDEPENDENT_CODE 以便链接到共享库
set_target_properties(mv-logger PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ----------------------------------------------------------------------------
# mv-math-tools - 数学工具 (OBJECT 库)
# ----------------------------------------------------------------------------
add_library(mv-math-tools OBJECT math_tools.cpp)

target_link_libraries(mv-math-tools PUBLIC
    opencv_core
    Eigen3::Eigen
)

target_include_directories(mv-math-tools PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# OBJECT 库需要设置 POSITION_INDEPENDENT_CODE 以便链接到共享库
set_target_properties(mv-math-tools PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ----------------------------------------------------------------------------
# mv-base64 - Base64 编解码 (OBJECT 库)
# ----------------------------------------------------------------------------
add_library(mv-base64 OBJECT base64.cpp)

target_include_directories(mv-base64 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# OBJECT 库需要设置 POSITION_INDEPENDENT_CODE 以便链接到共享库
set_target_properties(mv-base64 PROPERTIES POSITION_INDEPENDENT_CODE ON)

message(STATUS "    ✓ Utils libraries configured")
