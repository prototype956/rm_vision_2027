#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)
cd "$PROJECT_ROOT"
config_root="${RM_VISION_CONFIG_ROOT:-${PROJECT_ROOT}/src/config}"
[[ "$config_root" = /* ]] || config_root="${PROJECT_ROOT}/${config_root}"
export RM_VISION_CONFIG_ROOT="$config_root"
SELECTED_PATH=""
explicit_build_dir=""

# Public commands are validated before resolving/building any executable.
print_help() {
  cat <<EOF
用法:
  $0
  $0 [--build-dir DIR] <功能组> <动作> [参数]

功能组:
  intrinsic             capture | solve SESSION
  real-extrinsics       capture | solve SESSION
  simulation-extrinsics capture | solve SESSION | visualize SOLUTION [--once]
  imu-axis              capture [DEVICE [NEW_OUTPUT_DIRECTORY]]
  imu-drift             capture [--device DEVICE] [--duration SECONDS]
                        [--note TEXT] [--motor-state STATE] [--thermal-state STATE]
                        analyze SESSION

无参数进入标定与 IMU 菜单；<功能组> --help 查看分组用法。
--build-dir 指定的目录必须包含所需程序，不回退到其他构建。
未指定时按 BUILD_DIR、仿真 VISION_BUILD_DIR 和可用构建目录查找；
IMU 程序未找到时独立编译，不需要相机 SDK、OpenCV 或 OpenVINO。
菜单需要 Python 3 和 PyYAML。RM_VISION_CONFIG_ROOT 选择配置根目录，
相对路径按项目根目录解析；IMU 使用当前车辆 profile。
仿真采集自动启动 Talos，SIMULATOR_ROOT 可指定仿真器目录。
旧的“构建目录 + 动作”和 simulation 等命令形式已移除。
EOF
}

print_group_help() {
  case "$1" in
    intrinsic|real-extrinsics) echo "用法: $0 [--build-dir DIR] $1 capture | solve SESSION" ;;
    simulation-extrinsics)
      echo "用法: $0 [--build-dir DIR] $1 capture | solve SESSION | visualize SOLUTION [--once]" ;;
    imu-axis) echo "用法: $0 [--build-dir DIR] imu-axis capture [DEVICE [NEW_OUTPUT_DIRECTORY]]" ;;
    imu-drift)
      echo "用法: $0 [--build-dir DIR] imu-drift capture [--device DEVICE] [--duration SECONDS]"
      echo "      [--note TEXT] [--motor-state STATE] [--thermal-state STATE]"
      echo "      $0 [--build-dir DIR] imu-drift analyze SESSION"
      echo "默认采集 900 秒，电机与冷/热机状态为 unspecified。" ;;
  esac
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

  if [[ -n "$explicit_build_dir" ]]; then
    candidate=$(absolute_path "$explicit_build_dir")
    if [[ -x "${candidate}/bin/${binary}" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
    echo "指定构建目录缺少可执行程序: ${candidate}/bin/${binary}" >&2
    return 1
  fi
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

# Build only the serial/geometry dependencies shared by the two IMU tools.
# Compiler messages go to stderr because the caller captures the binary path.
build_imu_binary() {
  local binary="$1" source build_dir
  case "$binary" in
    mv-imu-axis-capture) source=imu_axis_capture_main.cpp; build_dir=build-imu-axis ;;
    mv-imu-drift) source=imu_drift_main.cpp; build_dir=build-imu-drift ;;
    *) return 1 ;;
  esac
  mkdir -p "$build_dir/bin" || return $?
  "${CXX:-c++}" -std=c++20 -O2 -Wall -Wextra -Wpedantic -pthread \
    -DPROJECT_ROOT_PATH="\"${PROJECT_ROOT}\"" -DSPDLOG_FMT_EXTERNAL -Isrc -I/usr/include/eigen3 \
    "test/imu/$source" src/hal/serial/controller_link.cpp src/hal/imu/serial_imu.cpp \
    src/runtime/real_geometry_config.cpp src/core/logger.cpp \
    -lyaml-cpp -lspdlog -lfmt -o "$build_dir/bin/$binary" >&2 || return $?
  printf '%s/bin/%s\n' "$PROJECT_ROOT/$build_dir" "$binary"
}

resolve_binary() {
  local kind="$1" binary="$2" build_dir
  if build_dir=$(resolve_build_dir "$kind" "$binary"); then
    printf '%s/bin/%s\n' "$build_dir" "$binary"
  elif [[ -n "$explicit_build_dir" ]]; then
    return 1
  elif [[ "$binary" == mv-imu-* ]]; then
    build_imu_binary "$binary"
  else
    echo "未找到 $binary，请先构建对应 CMake 目标。" >&2
    return 1
  fi
}

command_error() {
  echo "参数错误: $2" >&2
  print_group_help "$1" >&2
  return 2
}

validate_command() {
  local group="$1" action="${2:-}" key value
  shift
  [[ $# -gt 0 ]] || { command_error "$group" "必须指定动作"; return 2; }
  shift
  case "$group:$action" in
    intrinsic:capture|real-extrinsics:capture|simulation-extrinsics:capture)
      [[ $# -eq 0 ]] || { command_error "$group" "capture 不接受额外参数"; return 2; } ;;
    intrinsic:solve|real-extrinsics:solve|simulation-extrinsics:solve|imu-drift:analyze)
      [[ $# -eq 1 && -n "$1" && "$1" != -* ]] || {
        command_error "$group" "必须指定一个会话目录"; return 2;
      } ;;
    simulation-extrinsics:visualize)
      [[ $# -ge 1 && $# -le 2 && -n "$1" && "$1" != -* ]] &&
        { [[ $# -eq 1 ]] || [[ "$2" == --once ]]; } || {
          command_error "$group" "必须指定结果目录，可选 --once"; return 2;
        } ;;
    imu-axis:capture)
      [[ $# -le 2 ]] && { [[ $# -eq 0 ]] || [[ "$1" != -* ]]; } &&
        { [[ $# -lt 2 ]] || [[ -n "$2" && "$2" != -* ]]; } || {
          command_error "$group" "仅接受可选设备和新输出目录"; return 2;
        } ;;
    imu-drift:capture)
      while [[ $# -gt 0 ]]; do
        [[ $# -ge 2 ]] || { command_error "$group" "选项缺少值"; return 2; }
        key="$1"; value="$2"; shift 2
        case "$key" in
          --device|--note|--motor-state|--thermal-state) ;;
          --duration)
            [[ "$value" =~ ^[+]?[0-9]+([.][0-9]*)?([eE][+-]?[0-9]+)?$ ||
               "$value" =~ ^[+]?[.][0-9]+([eE][+-]?[0-9]+)?$ ]] &&
              awk -v duration="$value" 'BEGIN { seconds = duration + 0; exit !(seconds > 0 && seconds <= 86400) }' || {
                command_error "$group" "duration 必须在 (0,86400] 秒范围内"; return 2;
              } ;;
          *) command_error "$group" "未知选项: $key"; return 2 ;;
        esac
      done ;;
    *) command_error "$group" "不支持的动作: $action"; return 2 ;;
  esac
}

# Both menu and explicit commands use this dispatcher; tool exit codes propagate.
run_command() {
  local group="$1" action="${2:-}" binary kind=camera launcher
  validate_command "$@" || return $?
  shift 2
  case "$group" in
    intrinsic|real-extrinsics) binary=mv-camera-calibration ;;
    simulation-extrinsics) binary=mv-extrinsic-calibration; kind=extrinsic ;;
    imu-axis) binary=mv-imu-axis-capture ;;
    imu-drift) binary=mv-imu-drift ;;
  esac
  binary=$(resolve_binary "$kind" "$binary") || return $?
  case "$group:$action" in
    real-extrinsics:*) "$binary" "extrinsics-$action" "$@" ;;
    imu-axis:capture) "$binary" "$@" ;;
    simulation-extrinsics:capture)
      launcher="$SCRIPT_DIR/run_simulation_vision.sh"
      [[ -x "$launcher" ]] || { echo "仿真启动脚本不可执行: $launcher" >&2; return 1; }
      env VISION_BIN="$binary" SKIP_CAMERA_BACKEND_CHECK=1 "$launcher" capture ;;
    *) "$binary" "$action" "$@" ;;
  esac
}

# Python parses YAML; a temporary NUL-delimited list preserves arbitrary path spaces.
select_session() {
  local mode="$1" listing status index choice
  local -a records=()
  listing=$(mktemp)
  if python3 "$SCRIPT_DIR/internal/calibration_menu_support.py" "$mode" > "$listing"; then
    mapfile -d '' -t records < "$listing"
    unlink "$listing"
  else
    status=$?
    unlink "$listing"
    echo "会话列表读取失败，状态码: $status" >&2
    return 1
  fi
  SELECTED_PATH=""
  if [[ "${#records[@]}" -eq 0 ]]; then
    echo "没有找到可用会话或结果。" >&2
    return 1
  fi
  for ((index=0; index<${#records[@]}; index+=2)); do
    printf '  %d. %s  (%s)\n     %s\n' "$((index / 2 + 1))" \
      "$(basename "${records[index]}")" "${records[index+1]}" "${records[index]}"
  done
  echo "  q. 返回上级菜单"
  read -r -p "请选择编号: " choice || return 1
  [[ "$choice" != q && "$choice" != Q ]] || return 1
  if [[ ! "$choice" =~ ^[0-9]+$ || "${#choice}" -gt 8 ]]; then
    echo "无效编号。" >&2
    return 1
  fi
  local number=$((10#$choice))
  if ((number < 1 || number > ${#records[@]} / 2)); then
    echo "无效编号。" >&2
    return 1
  fi
  SELECTED_PATH="${records[(number-1)*2]}"
}

menu_solve() {
  local mode="$1" description="$2" group="$3" action="$4"
  select_session "$mode" || return 0
  run_tool_from_menu "$description" run_command "$group" "$action" "$SELECTED_PATH"
}

menu_imu_capture() {
  local device output_dir
  read -r -p "串口设备（留空使用当前车辆 profile）: " device || return 0
  read -r -p "输出目录（留空使用默认目录）: " output_dir || return 0
  if [[ -n "$output_dir" ]]; then
    run_tool_from_menu "IMU 安装轴向采集" run_command imu-axis capture "$device" "$(absolute_path "$output_dir")"
  elif [[ -n "$device" ]]; then
    run_tool_from_menu "IMU 安装轴向采集" run_command imu-axis capture "$device"
  else
    run_tool_from_menu "IMU 安装轴向采集" run_command imu-axis capture
  fi
}

menu_drift_capture() {
  local device duration motor thermal note
  read -r -p "串口设备（留空使用当前车辆 profile）: " device || return 0
  read -r -p "采集时长，秒 [900]: " duration || return 0
  read -r -p "电机状态 [unspecified]: " motor || return 0
  read -r -p "冷/热机状态 [unspecified]: " thermal || return 0
  read -r -p "实验备注（可留空）: " note || return 0
  local -a args=(capture --duration "${duration:-900}" --motor-state "${motor:-unspecified}"
                --thermal-state "${thermal:-unspecified}" --note "$note")
  [[ -z "$device" ]] || args+=(--device "$device")
  run_tool_from_menu "IMU 静止漂移采集" run_command imu-drift "${args[@]}"
}

menu_drift_analyze() {
  select_session drift || return 0
  run_tool_from_menu "IMU 漂移离线分析" run_command imu-drift analyze "$SELECTED_PATH"
}

submenu() {
  local group="$1" choice
  while :; do
    case "$group" in
      intrinsic) printf '\n相机内参\n1. 采集\n2. 解算已有会话\n' ;;
      real) printf '\n实机相机—云台外参\n1. 采集\n2. 审核并解算已有会话\n' ;;
      simulation) printf '\n仿真相机—云台外参\n1. 采集\n2. 解算已有会话\n3. Foxglove 三维验收\n' ;;
      imu) printf '\nIMU 检查\n1. 安装轴向采集\n2. 静止漂移采集\n3. 漂移数据离线分析\n' ;;
    esac
    echo "q. 返回主菜单"
    read -r -p "请选择功能: " choice || return 0
    case "$group:$choice" in
      *:q|*:Q) return 0 ;;
      intrinsic:1) run_tool_from_menu "相机内参采集" run_command intrinsic capture ;;
      intrinsic:2) menu_solve intrinsic "相机内参解算" intrinsic solve ;;
      real:1) run_tool_from_menu "实机外参采集" run_command real-extrinsics capture ;;
      real:2) menu_solve real "实机外参审核解算" real-extrinsics solve ;;
      simulation:1) run_tool_from_menu "仿真外参采集" run_command simulation-extrinsics capture ;;
      simulation:2) menu_solve simulation "仿真外参解算" simulation-extrinsics solve ;;
      simulation:3) menu_solve simulation-solutions "Foxglove 外参验收" simulation-extrinsics visualize ;;
      imu:1) menu_imu_capture ;;
      imu:2) menu_drift_capture ;;
      imu:3) menu_drift_analyze ;;
      *) echo "无效选项。" >&2 ;;
    esac
  done
}

interactive_menu() {
  local choice
  while :; do
    cat <<'EOF'

========================================
        标定与 IMU 工具
========================================
1. 相机内参
2. 实机相机—云台外参
3. 仿真相机—云台外参
4. IMU 检查
q. 退出
EOF
    read -r -p "请选择功能: " choice || return 0
    case "$choice" in
      1) submenu intrinsic ;;
      2) submenu real ;;
      3) submenu simulation ;;
      4) submenu imu ;;
      q|Q) return 0 ;;
      *) echo "无效选项，请输入 1-4 或 q。" >&2 ;;
    esac
  done
}

if [[ "${1:-}" == --help && $# -eq 1 ]]; then
  print_help
  exit 0
fi

if [[ "${1:-}" == --build-dir ]]; then
  if [[ $# -lt 2 || -z "$2" || "$2" == -* ]]; then
    echo "--build-dir 必须指定目录。" >&2
    exit 2
  fi
  explicit_build_dir="$2"
  shift 2
fi

if [[ $# -eq 0 ]]; then
  if [[ ! -t 0 && "${CALIBRATION_MENU:-0}" != 1 ]]; then
    echo "无参数模式需要交互式终端；自动化请使用显式命令行参数。" >&2
    exit 2
  fi
  python3 "$SCRIPT_DIR/internal/calibration_menu_support.py" --help > /dev/null
  interactive_menu
  exit 0
fi

case "$1" in
  intrinsic|real-extrinsics|simulation-extrinsics|imu-axis|imu-drift) ;;
  --help)
    [[ $# -eq 1 ]] && { print_help; exit 0; }
    echo "--help 不接受额外参数。" >&2
    exit 2 ;;
  *)
    echo "未知功能组或已移除的旧命令形式: $1" >&2
    print_help >&2
    exit 2 ;;
esac
if [[ $# -eq 2 && "$2" == --help ]]; then
  print_group_help "$1"
  exit 0
fi
run_command "$@"
