#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
# 临时诊断工具仅依赖 C++20 和现有串口模块，不要求相机、OpenVINO 或完整视觉构建。
mkdir -p build-imu-axis/bin
"${CXX:-c++}" -std=c++20 -O2 -Wall -Wextra -Wpedantic -pthread -Isrc \
  test/imu/imu_axis_capture_main.cpp src/hal/serial/controller_link.cpp \
  -o build-imu-axis/bin/mv-imu-axis-capture
exec build-imu-axis/bin/mv-imu-axis-capture "$@"
