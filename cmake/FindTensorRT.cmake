# TensorRT C++ SDK: supports distribution multiarch layouts and NVIDIA tar archives.
# Use -DTensorRT_ROOT=/path/to/sdk for a local, non-system installation.
find_path(TensorRT_INCLUDE_DIR NvInfer.h
    HINTS ${TensorRT_ROOT} ENV TensorRT_ROOT
    PATH_SUFFIXES include include/${CMAKE_LIBRARY_ARCHITECTURE})
find_path(TensorRT_ONNX_INCLUDE_DIR NvOnnxParser.h
    HINTS ${TensorRT_ROOT} ENV TensorRT_ROOT
    PATH_SUFFIXES include include/${CMAKE_LIBRARY_ARCHITECTURE})
find_library(TensorRT_LIBRARY NAMES nvinfer
    HINTS ${TensorRT_ROOT} ENV TensorRT_ROOT
    PATH_SUFFIXES lib lib64 lib/${CMAKE_LIBRARY_ARCHITECTURE})
find_library(TensorRT_ONNX_LIBRARY NAMES nvonnxparser
    HINTS ${TensorRT_ROOT} ENV TensorRT_ROOT
    PATH_SUFFIXES lib lib64 lib/${CMAKE_LIBRARY_ARCHITECTURE})
if(TensorRT_INCLUDE_DIR AND EXISTS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h")
    file(READ "${TensorRT_INCLUDE_DIR}/NvInferVersion.h" _trt_version_header)
    foreach(_part MAJOR MINOR PATCH)
        string(REGEX MATCH "#define[ \t]+NV_TENSORRT_${_part}[ \t]+([A-Za-z0-9_]+)" _match "${_trt_version_header}")
        set(_value "${CMAKE_MATCH_1}")
        # NVIDIA packages may alias these to TRT_*_ENTERPRISE instead of numeric literals.
        if(NOT _value MATCHES "^[0-9]+$")
            string(REGEX MATCH "#define[ \t]+${_value}[ \t]+([0-9]+)" _match "${_trt_version_header}")
            set(_value "${CMAKE_MATCH_1}")
        endif()
        set(_trt_${_part} "${_value}")
    endforeach()
    set(TensorRT_VERSION "${_trt_MAJOR}.${_trt_MINOR}.${_trt_PATCH}")
endif()
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TensorRT
    REQUIRED_VARS TensorRT_INCLUDE_DIR TensorRT_ONNX_INCLUDE_DIR TensorRT_LIBRARY TensorRT_ONNX_LIBRARY
    VERSION_VAR TensorRT_VERSION)
if(TensorRT_FOUND)
    if(NOT TARGET TensorRT::nvinfer)
        add_library(TensorRT::nvinfer UNKNOWN IMPORTED)
        set_target_properties(TensorRT::nvinfer PROPERTIES
            IMPORTED_LOCATION "${TensorRT_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIR}")
    endif()
    if(NOT TARGET TensorRT::nvonnxparser)
        add_library(TensorRT::nvonnxparser UNKNOWN IMPORTED)
        set_target_properties(TensorRT::nvonnxparser PROPERTIES
            IMPORTED_LOCATION "${TensorRT_ONNX_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_ONNX_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES TensorRT::nvinfer)
    endif()
endif()
mark_as_advanced(TensorRT_INCLUDE_DIR TensorRT_ONNX_INCLUDE_DIR TensorRT_LIBRARY TensorRT_ONNX_LIBRARY)
