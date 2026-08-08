#!/usr/bin/env bash

set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)

if [ "$#" -gt 1 ]; then
  echo "Usage: $0 [build-directory]" >&2
  exit 2
fi

BUILD_DIR=${1:-"${PROJECT_ROOT}/build-camera"}
case "${BUILD_DIR}" in
  /*) ;;
  *) BUILD_DIR="${PROJECT_ROOT}/${BUILD_DIR}" ;;
esac

CALIBRATION_BIN="${BUILD_DIR}/bin/mv-camera-calibration"
if [ ! -x "${CALIBRATION_BIN}" ]; then
  echo "Camera calibration executable not found: ${CALIBRATION_BIN}" >&2
  echo "Build it with:" >&2
  echo "  cmake -S \"${PROJECT_ROOT}\" -B \"${BUILD_DIR}\" -DCMAKE_BUILD_TYPE=Release -DBUILD_MAIN=OFF -DUSE_OPENVINO=OFF" >&2
  echo "  cmake --build \"${BUILD_DIR}\" --parallel 4 --target mv-camera-calibration" >&2
  exit 1
fi

exec "${CALIBRATION_BIN}"
