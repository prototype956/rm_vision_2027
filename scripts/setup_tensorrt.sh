#!/usr/bin/env bash
# 使用 source 加载此文件，即可使用工作区内的 TensorRT SDK，无需 sudo。
# 可通过 TensorRT_ROOT 指定其他 SDK 根目录，该目录应包含 include/ 和 lib/。

RM_TENSORRT_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export TensorRT_ROOT="${TensorRT_ROOT:-${RM_TENSORRT_SCRIPT_DIR}/../.deps/tensorrt/usr}"
RM_TENSORRT_ARCH="$(uname -m)-linux-gnu"
RM_TENSORRT_LIB_DIR=""
for RM_TENSORRT_CANDIDATE in "${TensorRT_ROOT}/lib/${RM_TENSORRT_ARCH}" "${TensorRT_ROOT}/lib" "${TensorRT_ROOT}/lib64"; do
  if [[ -f "${RM_TENSORRT_CANDIDATE}/libnvinfer.so" && -f "${RM_TENSORRT_CANDIDATE}/libnvonnxparser.so" ]]; then
    RM_TENSORRT_LIB_DIR="${RM_TENSORRT_CANDIDATE}"
    break
  fi
done
if [[ -z "${RM_TENSORRT_LIB_DIR}" ]]; then
  echo "[TensorRT] SDK libraries not found under ${TensorRT_ROOT}; install the matching runtime and ONNX parser first." >&2
  return 1 2>/dev/null || exit 1
fi

export LD_LIBRARY_PATH="${RM_TENSORRT_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
# 可选：加载安装在同一工作区中的通用 C++ 依赖，如 JSON、Ceres。
RM_VISION_DEPS_ROOT="${RM_TENSORRT_SCRIPT_DIR}/../.deps/vision/usr"
if [[ -d "${RM_VISION_DEPS_ROOT}" ]]; then
  export CMAKE_PREFIX_PATH="${RM_VISION_DEPS_ROOT}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${RM_VISION_DEPS_ROOT}/lib/${RM_TENSORRT_ARCH}:${LD_LIBRARY_PATH}"
fi
echo "[TensorRT] SDK: ${TensorRT_ROOT}"
echo "[TensorRT] Configure with -DUSE_OPENVINO=OFF -DUSE_TENSORRT=ON -DTensorRT_ROOT=\"\${TensorRT_ROOT}\""
unset RM_TENSORRT_SCRIPT_DIR RM_TENSORRT_ARCH RM_TENSORRT_LIB_DIR RM_TENSORRT_CANDIDATE RM_VISION_DEPS_ROOT
