#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VISION_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(dirname -- "${VISION_ROOT}")"

SIMULATOR_ROOT="${SIMULATOR_ROOT:-${WORKSPACE_ROOT}/bevy_robomaster_simulator}"
SIMULATOR_BIN="${SIMULATOR_BIN:-${SIMULATOR_ROOT}/target/release/daedalus}"
VISION_BIN="${VISION_BIN:-${VISION_ROOT}/build-openvino/bin/mv-vision-main}"
OPENVINO_SETUP="${OPENVINO_SETUP:-/opt/intel/openvino_2024.0.0/setupvars.sh}"

SIMULATOR_CPUSET="${SIMULATOR_CPUSET:-8-15}"
VISION_CPUSET="${VISION_CPUSET:-0-7}"
SIMULATOR_NICE="${SIMULATOR_NICE:-5}"
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-15}"
RUSTC_BIN="${RUSTC_BIN:-rustc}"

MAIN_CONFIG="${VISION_ROOT}/src/config/app/main.yaml"
TALOS_CONFIG="${VISION_ROOT}/src/config/hal/camera/talos.yaml"

SIMULATOR_PID=""
VISION_PID=""
CLEANED_UP=0

die() {
  echo "[launcher] ERROR: $*" >&2
  exit 1
}

# 同时终止两个子进程；幂等保护避免信号处理和 EXIT trap 重复清理。
cleanup() {
  if ((CLEANED_UP != 0)); then
    return
  fi
  CLEANED_UP=1
  trap - EXIT INT TERM

  for pid in "${VISION_PID}" "${SIMULATOR_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done

  for pid in "${VISION_PID}" "${SIMULATOR_PID}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

[[ -d "${SIMULATOR_ROOT}" ]] || die "simulator repository not found: ${SIMULATOR_ROOT}"
[[ -d "${SIMULATOR_ROOT}/assets" ]] || die "simulator assets directory not found: ${SIMULATOR_ROOT}/assets"
[[ -x "${SIMULATOR_BIN}" ]] || die "simulator binary not found; build it with: cd '${SIMULATOR_ROOT}' && cargo build --release --no-default-features --features talos"
[[ -x "${VISION_BIN}" ]] || die "vision binary not found; build the Release target first: ${VISION_BIN}"
[[ -f "${OPENVINO_SETUP}" ]] || die "OpenVINO setup script not found: ${OPENVINO_SETUP}"
[[ -f "${MAIN_CONFIG}" ]] || die "vision main config not found: ${MAIN_CONFIG}"
[[ -f "${TALOS_CONFIG}" ]] || die "Talos camera config not found: ${TALOS_CONFIG}"
command -v "${RUSTC_BIN}" >/dev/null 2>&1 || die "rustc is required to locate simulator runtime libraries"

CAMERA_BACKEND="$(awk '$1 == "backend:" { print $2; exit }' "${MAIN_CONFIG}")"
[[ "${CAMERA_BACKEND}" == "talos" ]] || die "set camera.backend to talos in ${MAIN_CONFIG} before starting simulation"
TALOS_META="$(awk '$1 == "meta_path:" { print $2; exit }' "${TALOS_CONFIG}")"
TALOS_IMAGE_POOL="$(awk '$1 == "image_pool_path:" { print $2; exit }' "${TALOS_CONFIG}")"
[[ -n "${TALOS_META}" && -n "${TALOS_IMAGE_POOL}" ]] || die "Talos shared-memory paths are missing in ${TALOS_CONFIG}"

# Talos uses persistent mmap-backed files. Remember the old metadata timestamp so a
# previous run cannot make the launcher report ready before this simulator publishes.
TALOS_META_MTIME_BEFORE=""
if [[ -e "${TALOS_META}" ]]; then
  TALOS_META_MTIME_BEFORE="$(stat -c '%y' "${TALOS_META}")"
fi

talos_is_ready() {
  # 两个文件必须非空，且元数据必须由本次启动的仿真进程重新发布。
  [[ -s "${TALOS_META}" && -s "${TALOS_IMAGE_POOL}" ]] || return 1
  [[ -z "${TALOS_META_MTIME_BEFORE}" ]] ||
    [[ "$(stat -c '%y' "${TALOS_META}")" != "${TALOS_META_MTIME_BEFORE}" ]]
}

# OpenVINO exports library/plugin paths inherited by the vision child process.
# shellcheck disable=SC1090
set +u
source "${OPENVINO_SETUP}"
set -u

# Bevy's dynamic_linking feature keeps both Bevy and Rust's standard library outside the
# executable. Cargo normally supplies these paths for `cargo run`; direct execution must do so.
RUST_SYSROOT="$("${RUSTC_BIN}" --print sysroot)"
RUST_HOST="$("${RUSTC_BIN}" -vV | awk '$1 == "host:" { print $2; exit }')"
SIMULATOR_DEPS_DIR="${SIMULATOR_ROOT}/target/release/deps"
RUST_STD_LIB_DIR="${RUST_SYSROOT}/lib/rustlib/${RUST_HOST}/lib"
[[ -d "${SIMULATOR_DEPS_DIR}" ]] || die "simulator dependency directory not found: ${SIMULATOR_DEPS_DIR}"
[[ -d "${RUST_STD_LIB_DIR}" ]] || die "Rust standard-library directory not found: ${RUST_STD_LIB_DIR}"
SIMULATOR_LD_LIBRARY_PATH="${SIMULATOR_DEPS_DIR}:${RUST_STD_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

UNRESOLVED_LIBRARIES="$(LD_LIBRARY_PATH="${SIMULATOR_LD_LIBRARY_PATH}" ldd "${SIMULATOR_BIN}" | awk '/not found/ { print $1 }')"
[[ -z "${UNRESOLVED_LIBRARIES}" ]] || die "simulator has unresolved shared libraries: ${UNRESOLVED_LIBRARIES//$'\n'/, }"

echo "[launcher] simulator: CPUs ${SIMULATOR_CPUSET}, nice ${SIMULATOR_NICE}"
echo "[launcher] vision:    CPUs ${VISION_CPUSET}"

(
  cd -- "${SIMULATOR_ROOT}"
  # A directly executed Bevy binary otherwise resolves assets relative to target/release.
  # Point it at the repository root so AssetPlugin finds ${SIMULATOR_ROOT}/assets.
  export BEVY_ASSET_ROOT="${SIMULATOR_ROOT}"
  export LD_LIBRARY_PATH="${SIMULATOR_LD_LIBRARY_PATH}"
  exec taskset -c "${SIMULATOR_CPUSET}" nice -n "${SIMULATOR_NICE}" "${SIMULATOR_BIN}"
) &
SIMULATOR_PID=$!

echo "[launcher] waiting for Talos shared memory..."
DEADLINE=$((SECONDS + STARTUP_TIMEOUT_SEC))
while ! talos_is_ready; do
  if ! kill -0 "${SIMULATOR_PID}" 2>/dev/null; then
    wait "${SIMULATOR_PID}" || true
    die "simulator exited before Talos shared memory became ready"
  fi
  if ((SECONDS >= DEADLINE)); then
    die "Talos shared memory was not ready within ${STARTUP_TIMEOUT_SEC}s"
  fi
  sleep 0.1
done

echo "[launcher] Talos ready; starting vision"
(
  cd -- "${VISION_ROOT}"
  exec taskset -c "${VISION_CPUSET}" "${VISION_BIN}"
) &
VISION_PID=$!

set +e
wait -n "${SIMULATOR_PID}" "${VISION_PID}"
EXIT_STATUS=$?
set -e

if ! kill -0 "${VISION_PID}" 2>/dev/null; then
  echo "[launcher] vision process exited with status ${EXIT_STATUS}"
else
  echo "[launcher] simulator process exited with status ${EXIT_STATUS}"
fi

exit "${EXIT_STATUS}"
