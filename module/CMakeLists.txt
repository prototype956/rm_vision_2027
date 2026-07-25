# ============================================================================
# Module 算法模块库
# ============================================================================

message(STATUS "  Configuring algorithm modules...")

# ----------------------------------------------------------------------------
# angle_solve - 角度解算模块
# ----------------------------------------------------------------------------
add_library(mv-basic-pnp SHARED angle_solve/basic_pnp.cpp)
target_link_libraries(mv-basic-pnp
    PUBLIC
    opencv_core
    opencv_calib3d
    opencv_imgproc
    opencv_highgui
    fmt::fmt
)
target_include_directories(mv-basic-pnp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/angle_solve>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

add_library(mv-angle-solve SHARED angle_solve/angle_solve.cpp)
target_link_libraries(mv-angle-solve
    PUBLIC
    mv-basic-pnp
    opencv_core
    fmt::fmt
)
target_include_directories(mv-angle-solve PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/angle_solve>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

# ----------------------------------------------------------------------------
# armor - 装甲板检测模块
# ----------------------------------------------------------------------------
add_library(mv-basic-armor SHARED armor/basic_armor.cpp)
target_link_libraries(mv-basic-armor
    PUBLIC
    opencv_core
    opencv_imgproc
    opencv_highgui
    fmt::fmt
)
target_include_directories(mv-basic-armor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/armor>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

add_library(mv-fan-armor SHARED armor/fan_armor.cpp)
target_link_libraries(mv-fan-armor
    PUBLIC
    opencv_core
    opencv_imgproc
    fmt::fmt
)
target_include_directories(mv-fan-armor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/armor>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

if(USE_ONNXRUNTIME)
    add_library(mv-dnn-armor SHARED armor/DNN_armor.cpp)
    target_link_libraries(mv-dnn-armor
        PUBLIC
        opencv_core
        opencv_imgproc
        opencv_dnn
        fmt::fmt
    )
    target_include_directories(mv-dnn-armor PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/armor>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    )
    target_compile_definitions(mv-dnn-armor PUBLIC USE_ONNXRUNTIME)
    if(ONNXRUNTIME_LIB)
        target_include_directories(mv-dnn-armor PRIVATE ${ONNXRUNTIME_INCLUDE_DIRS})
        target_link_libraries(mv-dnn-armor PRIVATE ${ONNXRUNTIME_LIB})
    endif()
    message(STATUS "    ✓ mv-dnn-armor enabled (ONNX Runtime)")
else()
    message(STATUS "    ⊗ mv-dnn-armor disabled (USE_ONNXRUNTIME=OFF)")
endif()

# ----------------------------------------------------------------------------
# buff - 能量机关模块
# ----------------------------------------------------------------------------
add_library(mv-new-buff SHARED buff/new_buff.cpp)
target_link_libraries(mv-new-buff
    PUBLIC
    opencv_core
    opencv_imgproc
    fmt::fmt
)
target_include_directories(mv-new-buff PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/buff>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

add_library(mv-basic-buff SHARED buff/basic_buff.cpp)
target_link_libraries(mv-basic-buff
    PRIVATE
    mv-fan-armor
    mv-new-buff
    PUBLIC
    opencv_core
    opencv_imgproc
    fmt::fmt
)
target_include_directories(mv-basic-buff PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/buff>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

# ----------------------------------------------------------------------------
# filter - 滤波器模块
# ----------------------------------------------------------------------------
add_library(mv-basic-kalman SHARED filter/basic_kalman.cpp)
target_link_libraries(mv-basic-kalman
    PUBLIC
    opencv_core
    fmt::fmt
)
target_include_directories(mv-basic-kalman PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/filter>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

# ----------------------------------------------------------------------------
# ml - 机器学习模块
# ----------------------------------------------------------------------------
add_library(mv-onnx-inferring SHARED ml/onnx_inferring.cpp)
target_link_libraries(mv-onnx-inferring
    PUBLIC
    opencv_core
    opencv_dnn
    fmt::fmt
)
target_include_directories(mv-onnx-inferring PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ml>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

# 如果启用了 ONNX Runtime
if(USE_ONNXRUNTIME AND ONNXRUNTIME_LIB)
    target_include_directories(mv-onnx-inferring PRIVATE ${ONNXRUNTIME_INCLUDE_DIRS})
    target_link_libraries(mv-onnx-inferring PRIVATE ${ONNXRUNTIME_LIB})
endif()

# ----------------------------------------------------------------------------
# roi - ROI 处理模块
# ----------------------------------------------------------------------------
add_library(mv-basic-roi SHARED roi/basic_roi.cpp)
target_link_libraries(mv-basic-roi
    PUBLIC
    opencv_core
    opencv_imgproc
    fmt::fmt
)
target_include_directories(mv-basic-roi PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/roi>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

# ----------------------------------------------------------------------------
# camera - 相机标定模块
# ----------------------------------------------------------------------------
add_library(mv-camera-calibration SHARED camera/camera_calibration.cpp)
target_link_libraries(mv-camera-calibration
    PUBLIC
    opencv_core
    opencv_calib3d
    opencv_imgproc
    opencv_imgcodecs
    fmt::fmt
)
target_include_directories(mv-camera-calibration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/camera>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

# ----------------------------------------------------------------------------
# predictor - 预测模块
# ----------------------------------------------------------------------------
add_library(mv-predictor SHARED
    predictor/tracker.cpp
    predictor/solver.cpp
    predictor/extended_kalman_filter.cpp
    predictor/armor.cpp
    predictor/target.cpp
    ${CMAKE_SOURCE_DIR}/utils/logger.cpp
    ${CMAKE_SOURCE_DIR}/utils/math_tools.cpp
)

target_link_libraries(mv-predictor
    PUBLIC
    opencv_core
    Eigen3::Eigen
    fmt::fmt
    yaml-cpp
)

target_include_directories(mv-predictor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/predictor>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/utils>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

# ----------------------------------------------------------------------------
# foxglove_publisher - Foxglove 发布器
# ----------------------------------------------------------------------------
if(USE_FOXGLOVE_SDK)
    add_subdirectory(foxglove_publisher)
endif()

message(STATUS "    ✓ Algorithm modules configured")
