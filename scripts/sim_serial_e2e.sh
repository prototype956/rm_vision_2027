#!/usr/bin/env bash

set -euo pipefail

EX_ROOT="/home/nuc/EX_MiracleVision"
AT_ROOT="/home/nuc/at_vision_simulator"

AT_BIN="${AT_BIN:-$AT_ROOT/target/release/daedalus}"
EX_BIN="${EX_BIN:-$EX_ROOT/build/src/app/mv-vision-main}"
EX_CFG="${EX_CFG:-$EX_ROOT/src/config/vision.yaml}"

SIM_ENDPOINT="${SIM_ENDPOINT:-127.0.0.1:19091}"
AT_SERIAL_STATUS_PERIOD_MS="${AT_SERIAL_STATUS_PERIOD_MS:-20}"
AT_ENEMY_COLOR="${AT_ENEMY_COLOR:-blue}"
AT_BULLET_SPEED_MPS="${AT_BULLET_SPEED_MPS:-25.0}"
AT_SERIAL_CMD_TIMEOUT_MS="${AT_SERIAL_CMD_TIMEOUT_MS:-100}"

RECOVERY_DEADLINE_MS=2000

LOG_DIR="$EX_ROOT/logs"
mkdir -p "$LOG_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
AT_LOG="$LOG_DIR/e2e_at_${TS}.log"
EX_LOG="$LOG_DIR/e2e_ex_${TS}.log"
SUMMARY_LOG="$LOG_DIR/e2e_${TS}.log"

AT_PID=""
EX_PID=""

log() {
	echo "[$(date '+%F %T')] $*" | tee -a "$SUMMARY_LOG"
}

fail() {
	log "FAIL: $*"
	log "AT_LOG=$AT_LOG"
	log "EX_LOG=$EX_LOG"
	exit 1
}

cleanup() {
	set +e
	if [[ -n "$EX_PID" ]] && kill -0 "$EX_PID" 2>/dev/null; then
		kill "$EX_PID" 2>/dev/null
		wait "$EX_PID" 2>/dev/null
	fi
	if [[ -n "$AT_PID" ]] && kill -0 "$AT_PID" 2>/dev/null; then
		kill "$AT_PID" 2>/dev/null
		wait "$AT_PID" 2>/dev/null
	fi
}

trap cleanup EXIT

now_ms() {
	date +%s%3N
}

wait_for_pattern() {
	local file="$1"
	local regex="$2"
	local timeout_s="$3"

	local start="$(date +%s)"
	while true; do
		if grep -Eq "$regex" "$file"; then
			return 0
		fi
		local now
		now="$(date +%s)"
		if (( now - start >= timeout_s )); then
			return 1
		fi
		sleep 0.1
	done
}

wait_for_pattern_since_line() {
	local file="$1"
	local start_line="$2"
	local regex="$3"
	local timeout_ms="$4"

	local begin_ms
	begin_ms="$(now_ms)"
	while true; do
		if tail -n +"$start_line" "$file" | grep -Eq "$regex"; then
			return 0
		fi
		local cur_ms
		cur_ms="$(now_ms)"
		if (( cur_ms - begin_ms >= timeout_ms )); then
			return 1
		fi
		sleep 0.05
	done
}

wait_for_port() {
	local host="$1"
	local port="$2"
	local timeout_s="$3"

	python3 - "$host" "$port" "$timeout_s" <<'PY'
import socket
import sys
import time

host = sys.argv[1]
port = int(sys.argv[2])
timeout_s = float(sys.argv[3])
deadline = time.time() + timeout_s

while time.time() < deadline:
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		sock.settimeout(0.2)
		try:
				sock.connect((host, port))
				sock.close()
				sys.exit(0)
		except OSError:
				pass
		finally:
				sock.close()
		time.sleep(0.05)

sys.exit(1)
PY
}

ensure_binaries() {
	if [[ ! -x "$AT_BIN" ]]; then
		log "AT binary not found, building release target"
		(cd "$AT_ROOT" && cargo build --release)
	fi

	if [[ ! -x "$EX_BIN" ]]; then
		log "EX binary not found, building mv-vision-main"
		(cd "$EX_ROOT" && cmake --build build -j"$(nproc)" --target mv-vision-main)
	fi
}

start_at() {
	log "Starting AT simulator"
	(
		cd "$AT_ROOT"
		AT_SERIAL_BIND_ADDR="$SIM_ENDPOINT" \
			AT_SERIAL_STATUS_PERIOD_MS="$AT_SERIAL_STATUS_PERIOD_MS" \
			AT_ENEMY_COLOR="$AT_ENEMY_COLOR" \
			AT_BULLET_SPEED_MPS="$AT_BULLET_SPEED_MPS" \
			AT_SERIAL_CMD_TIMEOUT_MS="$AT_SERIAL_CMD_TIMEOUT_MS" \
			"$AT_BIN"
	) >"$AT_LOG" 2>&1 &
	AT_PID="$!"
}

start_ex() {
	log "Starting EX vision process"
	(
		cd "$EX_ROOT"
		"$EX_BIN" "$EX_CFG"
	) >"$EX_LOG" 2>&1 &
	EX_PID="$!"
}

stop_at() {
	if [[ -n "$AT_PID" ]] && kill -0 "$AT_PID" 2>/dev/null; then
		log "Stopping AT simulator to trigger disconnect"
		kill "$AT_PID" 2>/dev/null
		wait "$AT_PID" 2>/dev/null || true
	fi
	AT_PID=""
}

assert_no_error_state() {
	if grep -Eq "SetError\(|ERROR state|SystemState::ERROR" "$EX_LOG"; then
		fail "EX log indicates error-state transition"
	fi
}

main() {
	log "=== Sim Serial E2E Start ==="
	ensure_binaries

	local host="${SIM_ENDPOINT%:*}"
	local port="${SIM_ENDPOINT##*:}"

	start_at
	wait_for_port "$host" "$port" 8 || fail "AT serial endpoint not ready: $SIM_ENDPOINT"

	start_ex
	wait_for_pattern "$EX_LOG" "connected to $SIM_ENDPOINT" 10 || \
		fail "EX did not establish initial serial connection"
	log "Initial serial connection established"

	stop_at
	if wait_for_pattern "$EX_LOG" "remote closed sim serial connection|send failed, switch to soft-degrade mode and reconnect later|initial connect to .* failed, entering soft-degrade reconnect mode" 10; then
		log "Disconnect/degrade event observed"
	else
		log "WARN: disconnect log not observed in 10s (possible idle socket), continue with reconnect SLA check"
	fi

	local reconnect_line_start
	reconnect_line_start="$(wc -l < "$EX_LOG")"
	start_at
	wait_for_port "$host" "$port" 8 || fail "AT serial endpoint not ready after restart"

	wait_for_pattern_since_line "$EX_LOG" "$reconnect_line_start" "connected to $SIM_ENDPOINT" "$RECOVERY_DEADLINE_MS" || \
		fail "Reconnect did not complete within ${RECOVERY_DEADLINE_MS}ms"

	log "Reconnect succeeded within ${RECOVERY_DEADLINE_MS}ms"
	assert_no_error_state

	log "PASS: serial disconnect/reconnect acceptance passed"
	log "AT_LOG=$AT_LOG"
	log "EX_LOG=$EX_LOG"
	log "=== Sim Serial E2E End ==="
}

main "$@"
