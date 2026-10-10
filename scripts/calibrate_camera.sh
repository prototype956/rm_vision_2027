#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)
ARTIFACTS_ROOT="${PROJECT_ROOT}/artifacts"
SELECTED_PATH=""

print_help() {
  cat <<EOF
Usage:
  $0
  $0 [build-directory] [capture | solve <capture-directory>]
  $0 [build-directory] simulation

Without arguments, start the interactive calibration menu:
  1  camera intrinsic capture
  2  simulator extrinsic capture
  3  camera intrinsic solve
  4  simulator extrinsic solve
  5  Foxglove extrinsic visualization
  6  IMU axis capture

The explicit command form is kept for automation. The simulation mode starts
the Talos simulator and launches mv-extrinsic-calibration. Set SIMULATOR_ROOT
when the simulator is not in ../bevy_robomaster_simulator relative to this
repository.

Build directory overrides:
  BUILD_DIR=/path/to/build          camera and explicit command selection
  VISION_BUILD_DIR=/path/to/build   simulator/extrinsic menu selection
EOF
}

absolute_path() {
  case "$1" in
    /*) printf '%s\n' "$1" ;;
    *) printf '%s\n' "${PROJECT_ROOT}/$1" ;;
  esac
}

resolve_build_dir() {
  local kind="$1"
  local binary="$2"
  local candidate
  local -a candidates=()

  if [[ -n "${BUILD_DIR:-}" ]]; then
    candidates+=("${BUILD_DIR}")
  fi
  if [[ "$kind" == "extrinsic" && -n "${VISION_BUILD_DIR:-}" ]]; then
    candidates+=("${VISION_BUILD_DIR}")
  fi
  if [[ "$kind" == "camera" ]]; then
    candidates+=("${PROJECT_ROOT}/build-camera" "${PROJECT_ROOT}/build-openvino"
                 "${PROJECT_ROOT}/build-tensorrt")
  else
    candidates+=("${PROJECT_ROOT}/build-openvino" "${PROJECT_ROOT}/build-tensorrt"
                 "${PROJECT_ROOT}/build-camera")
  fi

  for candidate in "${candidates[@]}"; do
    candidate=$(absolute_path "$candidate")
    if [[ -x "${candidate}/bin/${binary}" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

pause_menu() {
  local ignored
  if [[ -t 0 ]]; then
    read -r -p "按回车返回主菜单..." ignored || true
  fi
}

run_tool_from_menu() {
  local description="$1"
  shift
  printf '\n[%s]\n' "$description"
  set +e
  "$@"
  local status=$?
  set -e
  if [[ "$status" -eq 0 ]]; then
    echo "功能已完成。"
  else
    echo "功能退出，状态码: ${status}" >&2
  fi
  pause_menu
}

camera_binary_or_report() {
  local build_dir
  if ! build_dir=$(resolve_build_dir camera mv-camera-calibration); then
    echo "未找到 mv-camera-calibration。请先构建 build-camera 或 build-openvino。" >&2
    echo "  cmake --build build-camera --parallel 4 --target mv-camera-calibration" >&2
    return 1
  fi
  printf '%s/bin/mv-camera-calibration\n' "$build_dir"
}

extrinsic_binary_or_report() {
  local build_dir
  if ! build_dir=$(resolve_build_dir extrinsic mv-extrinsic-calibration); then
    echo "未找到 mv-extrinsic-calibration。请先构建 build-openvino 或 build-tensorrt。" >&2
    echo "  cmake --build build-openvino --parallel 4 --target mv-extrinsic-calibration" >&2
    return 1
  fi
  printf '%s/bin/mv-extrinsic-calibration\n' "$build_dir"
}

collect_directories() {
  local root="$1"
  local mode="$2"
  local entry path
  local -a items=()
  local -a details=()

  [[ -d "$root" ]] || return 1
  while IFS= read -r -d '' entry; do
    path="${entry#*|}"
    case "$mode" in
      camera)
        [[ -f "${path}/session.yaml" && -d "${path}/images" ]] || continue
        local image_count
        image_count=$(find "${path}/images" -maxdepth 1 -type f -print 2>/dev/null | wc -l)
        items+=("$path")
        details+=("${image_count} 个图像")
        ;;
      extrinsic)
        [[ -f "${path}/session.yaml" ]] || continue
        local extrinsic_image_count
        extrinsic_image_count=$(find "${path}/images" -maxdepth 1 -type f -print 2>/dev/null | wc -l)
        items+=("$path")
        details+=("${extrinsic_image_count} 个图像")
        ;;
    esac
  done < <(find "$root" -mindepth 1 -maxdepth 1 -type d \
    -printf '%T@|%p\0' 2>/dev/null | sort -z -rn)

  SELECTED_PATH=""
  if [[ "${#items[@]}" -eq 0 ]]; then
    return 1
  fi

  echo
  local index
  for index in "${!items[@]}"; do
    printf '  %d. %s  (%s)\n' "$((index + 1))" "$(basename "${items[index]}")" \
      "${details[index]}"
    printf '     %s\n' "${items[index]}"
  done
  echo "  q. 返回主菜单"
  local choice
  if ! read -r -p "请选择数据编号: " choice; then
    return 2
  fi
  case "$choice" in
    q|Q) return 2 ;;
  esac
  if [[ ! "$choice" =~ ^[0-9]+$ ]] || ((choice < 1 || choice > ${#items[@]})); then
    echo "无效的数据编号。" >&2
    return 2
  fi
  SELECTED_PATH="${items[choice-1]}"
}

collect_solution_directories() {
  local root="$1"
  local entry path session_dir
  local -a entries=()
  [[ -d "$root" ]] || return 1
  while IFS= read -r -d '' entry; do
    path="${entry#*|}"
    session_dir=$(dirname "$(dirname "$path")")
    [[ -f "${session_dir}/session.yaml" ]] || continue
    entries+=("$entry")
  done < <(find "$root" -mindepth 3 -maxdepth 3 -type d \
    -printf '%T@|%p\0' 2>/dev/null | sort -z -rn)

  SELECTED_PATH=""
  [[ "${#entries[@]}" -gt 0 ]] || return 1
  echo
  local index
  for index in "${!entries[@]}"; do
    path="${entries[index]#*|}"
    printf '  %d. %s  (extrinsics.yaml + report.yaml)\n' "$((index + 1))" "$(basename "$path")"
    printf '     会话: %s\n' "$(basename "$(dirname "$(dirname "$path")")")"
    printf '     %s\n' "$path"
  done
  echo "  q. 返回主菜单"
  local choice
  if ! read -r -p "请选择结果编号: " choice; then
    return 2
  fi
  case "$choice" in
    q|Q) return 2 ;;
  esac
  if [[ ! "$choice" =~ ^[0-9]+$ ]] || ((choice < 1 || choice > ${#entries[@]})); then
    echo "无效的结果编号。" >&2
    return 2
  fi
  SELECTED_PATH="${entries[choice-1]#*|}"
}

menu_camera_capture() {
  local binary
  binary=$(camera_binary_or_report) || return 0
  run_tool_from_menu "相机内参采集" "$binary" capture
}

menu_extrinsic_capture() {
  local binary
  binary=$(extrinsic_binary_or_report) || return 0
  local launcher="${SCRIPT_DIR}/run_simulation_vision.sh"
  if [[ ! -x "$launcher" ]]; then
    echo "仿真启动脚本不可执行: ${launcher}" >&2
    return 0
  fi
  echo "将启动 Talos 仿真并进入仿真外参采集。"
  run_tool_from_menu "仿真外参采集" env VISION_BIN="$binary" \
    SKIP_CAMERA_BACKEND_CHECK=1 "$launcher"
}

menu_camera_solve() {
  local binary
  binary=$(camera_binary_or_report) || return 0
  local selection_status=0
  collect_directories "${ARTIFACTS_ROOT}/camera_calibration" camera || selection_status=$?
  if [[ "$selection_status" -eq 1 ]]; then
    echo "没有找到可用于相机内参解算的采集目录。" >&2
    pause_menu
    return 0
  fi
  [[ "$selection_status" -eq 0 ]] || return 0
  run_tool_from_menu "相机内参解算: ${SELECTED_PATH}" "$binary" solve "$SELECTED_PATH"
}

menu_extrinsic_solve() {
  local binary
  binary=$(extrinsic_binary_or_report) || return 0
  local selection_status=0
  collect_directories "${ARTIFACTS_ROOT}/extrinsic_calibration" extrinsic || selection_status=$?
  if [[ "$selection_status" -eq 1 ]]; then
    echo "没有找到可用于仿真外参解算的采集目录。" >&2
    pause_menu
    return 0
  fi
  [[ "$selection_status" -eq 0 ]] || return 0
  run_tool_from_menu "仿真外参解算: ${SELECTED_PATH}" "$binary" solve "$SELECTED_PATH"
}

menu_extrinsic_visualize() {
  local binary
  binary=$(extrinsic_binary_or_report) || return 0
  local selection_status=0
  collect_solution_directories "${ARTIFACTS_ROOT}/extrinsic_calibration" || selection_status=$?
  if [[ "$selection_status" -eq 1 ]]; then
    echo "没有找到完整的外参解算结果。" >&2
    pause_menu
    return 0
  fi
  [[ "$selection_status" -eq 0 ]] || return 0
  run_tool_from_menu "Foxglove 外参三维验收: ${SELECTED_PATH}" \
    "$binary" visualize "$SELECTED_PATH"
}

menu_imu_capture() {
  local device output_dir
  if ! read -r -p "串口设备 [/dev/ttyUSB0]: " device; then
    return 0
  fi
  device="${device:-/dev/ttyUSB0}"
  if ! read -r -p "输出目录（留空使用默认目录）: " output_dir; then
    return 0
  fi
  if [[ -n "$output_dir" ]]; then
    run_tool_from_menu "IMU 轴向数据采集" "$SCRIPT_DIR/capture_imu_axes.sh" "$device" "$output_dir"
  else
    run_tool_from_menu "IMU 轴向数据采集" "$SCRIPT_DIR/capture_imu_axes.sh" "$device"
  fi
}

interactive_menu() {
  local choice
  while :; do
    cat <<'EOF'

========================================
        MiracleVision 标定工具
========================================
1. 相机内参采集
2. 仿真外参采集
3. 相机内参解算
4. 仿真外参解算
5. 仿真外参 Foxglove 三维验收
6. IMU 轴向数据采集
q. 退出
EOF
    if ! read -r -p "请选择功能: " choice; then
      echo
      return 0
    fi
    case "$choice" in
      1) menu_camera_capture ;;
      2) menu_extrinsic_capture ;;
      3) menu_camera_solve ;;
      4) menu_extrinsic_solve ;;
      5) menu_extrinsic_visualize ;;
      6) menu_imu_capture ;;
      q|Q) return 0 ;;
      *) echo "无效选项，请输入 1-6 或 q。" >&2 ;;
    esac
  done
}

if [[ "${1:-}" == "--help" ]]; then
  print_help
  exit 0
fi

if [[ "$#" -eq 0 ]]; then
  if [[ ! -t 0 && "${CALIBRATION_MENU:-0}" != "1" ]]; then
    echo "无参数模式需要交互式终端；自动化请使用显式命令行参数。" >&2
    exit 2
  fi
  interactive_menu
  exit 0
fi

# 兼容原有显式命令：build-directory 后跟 capture/solve，或直接 simulation。
BUILD_DIR="${PROJECT_ROOT}/build-camera"
MODE=""
if [[ "$#" -gt 0 ]]; then
  case "$1" in
    simulation|sim|extrinsic)
      MODE="simulation"
      shift
      ;;
    *)
      BUILD_DIR="$1"
      shift
      ;;
  esac
fi
if [[ -z "$MODE" ]]; then
  MODE="${1:-capture}"
  if [[ "$#" -gt 0 ]]; then
    shift
  fi
fi
BUILD_DIR=$(absolute_path "$BUILD_DIR")

if [[ "$MODE" == "simulation" ]]; then
  if [[ "$#" -gt 0 && "$1" == "capture" ]]; then
    shift
  fi
  if [[ "$#" -gt 0 ]]; then
    echo "Simulation mode accepts no arguments other than optional capture" >&2
    exit 2
  fi
  EXTRINSIC_BIN="${BUILD_DIR}/bin/mv-extrinsic-calibration"
  if [[ ! -x "$EXTRINSIC_BIN" ]]; then
    echo "Extrinsic capture executable not found: ${EXTRINSIC_BIN}" >&2
    echo "  cmake --build \"${BUILD_DIR}\" --parallel 4 --target mv-extrinsic-calibration" >&2
    exit 1
  fi
  SIMULATION_LAUNCHER="${SCRIPT_DIR}/run_simulation_vision.sh"
  [[ -x "$SIMULATION_LAUNCHER" ]] || {
    echo "Simulation launcher is not executable: ${SIMULATION_LAUNCHER}" >&2
    exit 1
  }
  export VISION_BIN="$EXTRINSIC_BIN"
  export SKIP_CAMERA_BACKEND_CHECK=1
  if [[ -z "${SIMULATOR_ROOT:-}" && -d "${PROJECT_ROOT}/../bevy_robomaster_simulator" ]]; then
    export SIMULATOR_ROOT="${PROJECT_ROOT}/../bevy_robomaster_simulator"
  fi
  exec "$SIMULATION_LAUNCHER"
fi

CALIBRATION_BIN="${BUILD_DIR}/bin/mv-camera-calibration"
if [[ ! -x "$CALIBRATION_BIN" ]]; then
  echo "Camera calibration executable not found: ${CALIBRATION_BIN}" >&2
  echo "  cmake --build \"${BUILD_DIR}\" --parallel 4 --target mv-camera-calibration" >&2
  exit 1
fi
exec "$CALIBRATION_BIN" "$MODE" "$@"
