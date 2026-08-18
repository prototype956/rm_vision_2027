#!/usr/bin/env python3
"""Bounded, read-only analysis for rm_vision_2027 MCAP recordings."""

from __future__ import annotations

import argparse
import base64
import collections
import datetime as dt
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
import hashlib
import json
import math
from pathlib import Path
import random
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Iterator, Sequence


DEFAULT_LIMIT = 20
MAX_STDOUT_LIMIT = 200
DEFAULT_FRAME_COUNT = 6
MAX_FRAME_COUNT = 50
RESERVOIR_SIZE = 10_000
IMAGE_TOPIC = "/vision/camera/image"

LAYER_TOPICS = {
    "detector": ["/vision/armor/annotations"],
    "pnp": [
        "/simulation/ground_truth/annotations",
        "/vision/pnp/corners",
        "/vision/pnp/reprojection",
        "/vision/pnp/error_vectors",
        "/vision/corner_refiner/axes",
        "/vision/corner_refiner/candidates",
    ],
    "prediction": [
        "/vision/prediction/current_annotations",
        "/vision/prediction/future_annotations",
    ],
    "control": ["/vision/control/selected_armor_annotations"],
}
LAYER_TOPICS["all"] = list(dict.fromkeys(sum(LAYER_TOPICS.values(), [])))


class AnalysisError(RuntimeError):
    """An actionable input, dependency, or MCAP decoding error."""


MISSING = object()


def find_mcap() -> str:
    binary = shutil.which("mcap")
    if binary is None:
        raise AnalysisError("mcap CLI was not found on PATH; install a compatible CLI first")
    return binary


def validate_input(path_text: str) -> Path:
    path = Path(path_text).expanduser().resolve()
    if not path.is_file():
        raise AnalysisError(f"MCAP file does not exist: {path}")
    if path.suffix.lower() != ".mcap":
        raise AnalysisError(f"expected a .mcap file: {path}")
    return path


def run_mcap(arguments: Sequence[str]) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        [find_mcap(), *arguments], text=True, capture_output=True, check=False
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or "unknown mcap error"
        raise AnalysisError(detail)
    return result


def parse_time_ns(value: str) -> int:
    text = value.strip()
    if re.fullmatch(r"[+-]?\d+", text):
        number = int(text)
        return number if abs(number) >= 100_000_000_000 else number * 1_000_000_000
    if re.fullmatch(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)", text):
        try:
            nanos = Decimal(text) * Decimal(1_000_000_000)
            return int(nanos.to_integral_value(rounding=ROUND_HALF_UP))
        except InvalidOperation as error:
            raise AnalysisError(f"invalid decimal time: {value!r}") from error

    match = re.fullmatch(
        r"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})(?:\.(\d{1,9}))?(Z|[+-]\d{2}:\d{2})",
        text,
    )
    if match is None:
        raise AnalysisError(
            f"invalid time {value!r}; use epoch seconds, epoch nanoseconds, or RFC3339"
        )
    base_text, fraction, zone = match.groups()
    base = dt.datetime.fromisoformat(base_text + ("+00:00" if zone == "Z" else zone))
    seconds = int(base.timestamp())
    nanos = int((fraction or "0").ljust(9, "0"))
    return seconds * 1_000_000_000 + nanos


def format_time_ns(nanos: int) -> str:
    seconds, fraction = divmod(nanos, 1_000_000_000)
    timestamp = dt.datetime.fromtimestamp(seconds, tz=dt.timezone.utc)
    return timestamp.strftime("%Y-%m-%dT%H:%M:%S") + f".{fraction:09d}Z"


def add_time_arguments(parser: argparse.ArgumentParser, required: bool = False) -> None:
    parser.add_argument(
        "--start", required=required, help="inclusive epoch seconds/ns or RFC3339 time"
    )
    parser.add_argument(
        "--end", required=required, help="exclusive epoch seconds/ns or RFC3339 time"
    )


def time_bounds(arguments: argparse.Namespace) -> tuple[int | None, int | None]:
    start = parse_time_ns(arguments.start) if arguments.start else None
    end = parse_time_ns(arguments.end) if arguments.end else None
    if start is not None and end is not None and start >= end:
        raise AnalysisError("--start must be earlier than --end")
    return start, end


def cat_command(
    path: Path, topics: Sequence[str], start_ns: int | None, end_ns: int | None
) -> list[str]:
    command = [find_mcap(), "cat", str(path), "--topics", ",".join(topics), "--json"]
    if start_ns is not None:
        command.extend(["--start-nsecs", str(start_ns)])
    if end_ns is not None:
        command.extend(["--end-nsecs", str(end_ns)])
    return command


def stream_messages(
    path: Path, topics: Sequence[str], start_ns: int | None = None, end_ns: int | None = None
) -> Iterator[dict[str, Any]]:
    process = subprocess.Popen(
        cat_command(path, topics, start_ns, end_ns),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=1,
    )
    assert process.stdout is not None
    try:
        for line in process.stdout:
            if not line.strip():
                continue
            try:
                message = json.loads(line)
                log_time = re.search(r'"log_time":([+-]?(?:\d+(?:\.\d*)?|\.\d+))', line)
                if log_time is not None:
                    message["_log_time_ns"] = parse_time_ns(log_time.group(1))
                yield message
            except json.JSONDecodeError as error:
                raise AnalysisError(f"mcap emitted invalid JSON: {error}") from error
        stderr = process.stderr.read() if process.stderr is not None else ""
        return_code = process.wait()
        if return_code != 0:
            raise AnalysisError(stderr.strip() or f"mcap cat exited with {return_code}")
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if process.stdout is not None:
            process.stdout.close()
        if process.stderr is not None:
            process.stderr.close()


def path_tokens(path: str) -> list[str | int]:
    normalized = path[5:] if path.startswith("data.") else path
    if not normalized:
        raise AnalysisError("field path must not be empty")
    tokens: list[str | int] = []
    for component in normalized.split("."):
        match = re.fullmatch(r"([^\[\]]+)((?:\[\d+\])*)", component)
        if match is None:
            raise AnalysisError(f"invalid field path: {path}")
        tokens.append(match.group(1))
        tokens.extend(int(index) for index in re.findall(r"\[(\d+)\]", match.group(2)))
    return tokens


def resolve_path(data: Any, path: str) -> Any:
    current = data
    for token in path_tokens(path):
        if isinstance(token, str) and isinstance(current, dict) and token in current:
            current = current[token]
        elif isinstance(token, int) and isinstance(current, list) and token < len(current):
            current = current[token]
        else:
            return MISSING
    return current


def parse_comparison_value(text: str) -> Any:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text


def compare(actual: Any, operation: str, expected: Any) -> bool:
    if operation == "exists":
        return actual is not MISSING
    if actual is MISSING:
        return False
    if operation == "contains":
        try:
            return expected in actual
        except TypeError:
            return False
    operators = {
        "eq": lambda left, right: left == right,
        "ne": lambda left, right: left != right,
        "lt": lambda left, right: left < right,
        "le": lambda left, right: left <= right,
        "gt": lambda left, right: left > right,
        "ge": lambda left, right: left >= right,
    }
    try:
        return operators[operation](actual, expected)
    except (KeyError, TypeError):
        return False


def message_matches(data: Any, conditions: Sequence[Sequence[str]]) -> bool:
    for field, operation, raw_expected in conditions:
        expected = parse_comparison_value(raw_expected)
        if not compare(resolve_path(data, field), operation, expected):
            return False
    return True


class FieldAccumulator:
    """Exact basic statistics plus deterministic, bounded quantile sampling."""

    def __init__(self, seed: int) -> None:
        self.total = 0
        self.missing = 0
        self.null = 0
        self.non_numeric = 0
        self.numeric = 0
        self.minimum = math.inf
        self.maximum = -math.inf
        self.total_value = 0.0
        self.sample: list[float] = []
        self.random = random.Random(seed)

    def add(self, value: Any) -> None:
        self.total += 1
        if value is MISSING:
            self.missing += 1
            return
        if value is None:
            self.null += 1
            return
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            self.non_numeric += 1
            return
        number = float(value)
        if not math.isfinite(number):
            self.non_numeric += 1
            return
        self.numeric += 1
        self.minimum = min(self.minimum, number)
        self.maximum = max(self.maximum, number)
        self.total_value += number
        if len(self.sample) < RESERVOIR_SIZE:
            self.sample.append(number)
        else:
            index = self.random.randrange(self.numeric)
            if index < RESERVOIR_SIZE:
                self.sample[index] = number

    @staticmethod
    def quantile(values: Sequence[float], probability: float) -> float | None:
        if not values:
            return None
        ordered = sorted(values)
        position = (len(ordered) - 1) * probability
        lower = math.floor(position)
        upper = math.ceil(position)
        if lower == upper:
            return ordered[lower]
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    def result(self) -> dict[str, Any]:
        return {
            "messages": self.total,
            "numeric": self.numeric,
            "missing": self.missing,
            "null": self.null,
            "non_numeric": self.non_numeric,
            "min": self.minimum if self.numeric else None,
            "max": self.maximum if self.numeric else None,
            "mean": self.total_value / self.numeric if self.numeric else None,
            "p50": self.quantile(self.sample, 0.50),
            "p95": self.quantile(self.sample, 0.95),
            "quantiles_exact": self.numeric <= RESERVOIR_SIZE,
            "quantile_sample_size": len(self.sample),
        }


def calculate_stats(
    path: Path,
    topic: str,
    fields: Sequence[str],
    start_ns: int | None = None,
    end_ns: int | None = None,
) -> dict[str, Any]:
    accumulators = {
        field: FieldAccumulator(int(hashlib.sha256(field.encode()).hexdigest()[:8], 16))
        for field in fields
    }
    messages = 0
    for message in stream_messages(path, [topic], start_ns, end_ns):
        messages += 1
        data = message.get("data", {})
        for field, accumulator in accumulators.items():
            accumulator.add(resolve_path(data, field))
    if messages == 0:
        raise AnalysisError(f"no messages found for {topic} in the selected time window")
    result = {field: accumulator.result() for field, accumulator in accumulators.items()}
    missing_fields = [field for field, value in result.items() if value["missing"] == messages]
    if missing_fields:
        raise AnalysisError("fields not found on any message: " + ", ".join(missing_fields))
    return {"topic": topic, "messages": messages, "fields": result}


def parse_info(text: str) -> dict[str, Any]:
    result: dict[str, Any] = {"channels": []}
    simple_fields = {"library", "profile", "messages", "duration", "start", "end"}
    for line in text.splitlines():
        stripped = line.strip()
        if ":" in stripped:
            key, value = stripped.split(":", 1)
            if key in simple_fields and key not in result:
                result[key] = value.strip()
        channel = re.match(
            r"\((\d+)\)\s+(\S+)\s+(\d+)\s+msgs.*?:\s+(.+?)\s+\[([^\]]+)\]\s*$",
            stripped,
        )
        if channel:
            channel_id, topic, messages, schema, encoding = channel.groups()
            result["channels"].append(
                {
                    "id": int(channel_id),
                    "topic": topic,
                    "messages": int(messages),
                    "schema": schema.strip(),
                    "encoding": encoding,
                }
            )
    if isinstance(result.get("messages"), str) and result["messages"].isdigit():
        result["messages"] = int(result["messages"])
    result["channel_count"] = len(result["channels"])
    return result


def doctor_summary(path: Path) -> dict[str, Any]:
    process = subprocess.Popen(
        [find_mcap(), "doctor", str(path)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )
    assert process.stdout is not None
    groups: dict[tuple[str, str], dict[str, Any]] = {}
    total = 0
    for raw_line in process.stdout:
        line = raw_line.strip()
        if not line:
            continue
        total += 1
        order_match = re.search(r'Message\.log_time .* on "([^"]+)" is less than', line)
        if order_match:
            key = ("out_of_order_log_time", order_match.group(1))
        elif "Header.profile field" in line:
            key = ("unknown_profile", "")
        elif line.lower().startswith("warning"):
            key = ("other_warning", "")
        else:
            key = ("diagnostic", "")
        group = groups.setdefault(
            key,
            {"kind": key[0], "topic": key[1] or None, "count": 0, "examples": []},
        )
        group["count"] += 1
        if len(group["examples"]) < 3:
            group["examples"].append(line)
    return_code = process.wait()
    process.stdout.close()
    group_list = sorted(
        groups.values(),
        key=lambda item: (-item["count"], item["kind"], item["topic"] or ""),
    )
    warnings = sum(item["count"] for item in group_list if item["kind"] != "diagnostic")
    return {
        "status": "invalid" if return_code else ("valid_with_warnings" if warnings else "valid"),
        "return_code": return_code,
        "diagnostic_lines": total,
        "warning_lines": warnings,
        "groups": group_list,
    }


def command_inspect(arguments: argparse.Namespace) -> int:
    path = validate_input(arguments.file)
    info_text = run_mcap(["info", str(path)]).stdout.strip()
    result = {
        "file": str(path),
        "size_bytes": path.stat().st_size,
        "info": parse_info(info_text),
        "doctor": doctor_summary(path),
    }
    if arguments.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(info_text)
        doctor = result["doctor"]
        print(f"\ndoctor: {doctor['status']} ({doctor['warning_lines']} warning lines)")
        for group in doctor["groups"]:
            suffix = f" [{group['topic']}]" if group["topic"] else ""
            print(f"  {group['kind']}{suffix}: {group['count']}")
            for example in group["examples"][: arguments.examples]:
                print(f"    {example}")
    return 1 if result["doctor"]["return_code"] else 0


def command_query(arguments: argparse.Namespace) -> int:
    path = validate_input(arguments.file)
    start_ns, end_ns = time_bounds(arguments)
    if arguments.limit == 0 and arguments.output is None:
        raise AnalysisError("--limit 0 is allowed only with --output")
    if arguments.output is None and arguments.limit > MAX_STDOUT_LIMIT:
        raise AnalysisError(f"stdout limit may not exceed {MAX_STDOUT_LIMIT}; use --output")

    limit = arguments.limit
    output_path = Path(arguments.output).expanduser().resolve() if arguments.output else None
    if output_path:
        if output_path == path or output_path.suffix.lower() == ".mcap":
            raise AnalysisError("query output must not be an MCAP path")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            destination = output_path.open("x", encoding="utf-8")
        except FileExistsError as error:
            raise AnalysisError(f"refusing to overwrite existing output: {output_path}") from error
    else:
        destination = sys.stdout
    matched = 0
    seen_fields = collections.Counter()
    try:
        for message in stream_messages(path, [arguments.topic], start_ns, end_ns):
            data = message.get("data", {})
            if not message_matches(data, arguments.where):
                continue
            if arguments.field:
                selected: dict[str, Any] = {}
                for field in arguments.field:
                    value = resolve_path(data, field)
                    if value is not MISSING:
                        seen_fields[field] += 1
                    selected[field] = None if value is MISSING else value
                record = {
                    "topic": message.get("topic"),
                    "sequence": message.get("sequence"),
                    "log_time": message.get("log_time"),
                    "data": selected,
                }
            else:
                record = {key: value for key, value in message.items() if not key.startswith("_")}
            destination.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
            matched += 1
            if limit and matched >= limit:
                break
    finally:
        if output_path:
            destination.close()
    if matched == 0:
        raise AnalysisError(f"no matching messages found for {arguments.topic}")
    missing_fields = [field for field in arguments.field if not seen_fields[field]]
    if missing_fields:
        raise AnalysisError(
            "fields not found on any returned message: " + ", ".join(missing_fields)
        )
    if output_path:
        print(json.dumps({"messages": matched, "output": str(output_path)}, ensure_ascii=False))
    return 0


def print_stats(result: dict[str, Any], as_json: bool) -> None:
    if as_json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return
    print(f"topic: {result['topic']} ({result['messages']} messages)")
    for field, values in result["fields"].items():
        approximate = "~" if not values["quantiles_exact"] else ""
        print(
            f"  {field}: n={values['numeric']} null={values['null']} missing={values['missing']} "
            f"min={values['min']} max={values['max']} mean={values['mean']} "
            f"p50={approximate}{values['p50']} p95={approximate}{values['p95']}"
        )


def command_stats(arguments: argparse.Namespace) -> int:
    path = validate_input(arguments.file)
    start_ns, end_ns = time_bounds(arguments)
    result = calculate_stats(path, arguments.topic, arguments.field, start_ns, end_ns)
    print_stats(result, arguments.json)
    return 0


def scan_latest_and_categories(
    path: Path,
    topic: str,
    category_fields: Sequence[str],
    start_ns: int | None,
    end_ns: int | None,
) -> tuple[dict[str, Any], dict[str, collections.Counter[Any]], int]:
    latest: dict[str, Any] | None = None
    categories = {field: collections.Counter() for field in category_fields}
    messages = 0
    for message in stream_messages(path, [topic], start_ns, end_ns):
        messages += 1
        data = message.get("data", {})
        latest = data
        for field, counter in categories.items():
            value = resolve_path(data, field)
            if value is not MISSING:
                counter[str(value)] += 1
    if latest is None:
        raise AnalysisError(f"no messages found for {topic}")
    return latest, categories, messages


def performance_preset(path: Path, start_ns: int | None, end_ns: int | None) -> dict[str, Any]:
    armor = calculate_stats(
        path,
        "/vision/armor/stats",
        ["preprocess_ms", "inference_ms", "postprocess_ms", "total_ms"],
        start_ns,
        end_ns,
    )
    debug = calculate_stats(
        path,
        "/vision/debug/stats",
        ["jpeg_encode_ms", "publish_latency_ms"],
        start_ns,
        end_ns,
    )
    latest, _, _ = scan_latest_and_categories(
        path, "/vision/debug/stats", [], start_ns, end_ns
    )
    return {
        "armor": armor,
        "debug": debug,
        "final_counters": {
            field: latest.get(field)
            for field in [
                "source_invalid_frames",
                "rate_limited_frames",
                "queue_overwritten_frames",
            ]
        },
    }


def pnp_preset(path: Path, start_ns: int | None, end_ns: int | None) -> dict[str, Any]:
    latest: dict[str, Any] | None = None
    latest_summary_sequence = -1
    attempt_status = collections.Counter()
    attempt_source = collections.Counter()
    messages = 0
    for message in stream_messages(path, ["/vision/pnp/stats"], start_ns, end_ns):
        messages += 1
        data = message.get("data", {})
        summary_sequence = int(data.get("summary_sequence", -1))
        if summary_sequence >= latest_summary_sequence:
            latest_summary_sequence = summary_sequence
            latest = data
        for attempt in data.get("attempts", []):
            attempt_status[str(attempt.get("status", "unknown"))] += 1
            attempt_source[str(attempt.get("source", "unknown"))] += 1
    if latest is None:
        raise AnalysisError("no messages found for /vision/pnp/stats")
    return {
        "messages": messages,
        "summary_sequence": latest_summary_sequence,
        "summary": latest.get("summary", {}),
        "groups": latest.get("groups", {}),
        "solve": latest.get("solve", {}),
        "refinement": latest.get("refinement", {}),
        "attempt_status_counts": dict(attempt_status),
        "attempt_source_counts": dict(attempt_source),
    }


def prediction_preset(path: Path, start_ns: int | None, end_ns: int | None) -> dict[str, Any]:
    stats = calculate_stats(
        path, "/vision/prediction/state", ["dt_s", "nis"], start_ns, end_ns
    )
    latest, categories, messages = scan_latest_and_categories(
        path,
        "/vision/prediction/state",
        ["tracker_state", "reset_reason"],
        start_ns,
        end_ns,
    )
    return {
        "messages": messages,
        "numeric": stats,
        "tracker_state_counts": dict(categories["tracker_state"]),
        "reset_reason_counts": dict(categories["reset_reason"]),
        "final_state": {
            field: latest.get(field)
            for field in ["sequence", "tracker_state", "label", "reset_reason"]
        },
    }


def control_preset(path: Path, start_ns: int | None, end_ns: int | None) -> dict[str, Any]:
    fields = [
        "control_period_s",
        "deadline_lateness_us",
        "prediction_age_s",
        "measurement_age_s",
        "sink_send_time_us",
        "yaw_published_minus_measured",
        "pitch_published_minus_measured",
    ]
    stats = calculate_stats(path, "/vision/control/tracking", fields, start_ns, end_ns)
    latest, categories, messages = scan_latest_and_categories(
        path,
        "/vision/control/tracking",
        ["command_source", "mpc_failure_reason", "feedback_source", "fallback_active"],
        start_ns,
        end_ns,
    )
    return {
        "messages": messages,
        "numeric": stats,
        "command_source_counts": dict(categories["command_source"]),
        "mpc_failure_reason_counts": dict(categories["mpc_failure_reason"]),
        "feedback_source_counts": dict(categories["feedback_source"]),
        "fallback_active_counts": dict(categories["fallback_active"]),
        "final_state": {
            field: latest.get(field)
            for field in [
                "source_sequence",
                "mpc_valid",
                "command_source",
                "mpc_failure_reason",
                "fallback_active",
            ]
        },
    }


def nested_value(data: Any, path: str) -> Any:
    value = resolve_path(data, path)
    return None if value is MISSING else value


def acceptance_checks(result: dict[str, Any]) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    performance = result.get("performance")
    if performance:
        actual = nested_value(performance, "armor.fields.total_ms.p95")
        checks.append(
            {
                "name": "detector_total_ms_p95",
                "actual": actual,
                "limit": 16.7,
                "pass": actual is not None and actual <= 16.7,
            }
        )
    pnp = result.get("pnp")
    if pnp:
        definitions = [
            (
                "ground_truth_reprojection_rmse_px_p95",
                "summary.ground_truth.reprojection_rmse_px.p95",
                0.1,
            ),
            (
                "ground_truth_position_error_m_p95",
                "summary.ground_truth.position_error_m.p95",
                0.005,
            ),
            (
                "ground_truth_rotation_error_deg_p95",
                "summary.ground_truth.rotation_error_deg.p95",
                0.5,
            ),
        ]
        for name, path, limit in definitions:
            actual = nested_value(pnp, path)
            checks.append(
                {
                    "name": name,
                    "actual": actual,
                    "limit": limit,
                    "pass": actual is not None and actual <= limit,
                }
            )
    return checks


def command_preset(arguments: argparse.Namespace) -> int:
    path = validate_input(arguments.file)
    start_ns, end_ns = time_bounds(arguments)
    functions = {
        "performance": performance_preset,
        "pnp": pnp_preset,
        "prediction": prediction_preset,
        "control": control_preset,
    }
    names = list(functions) if arguments.preset == "all" else [arguments.preset]
    result: dict[str, Any] = {"file": str(path), "window": {"start_ns": start_ns, "end_ns": end_ns}}
    for name in names:
        try:
            result[name] = functions[name](path, start_ns, end_ns)
        except AnalysisError as error:
            if arguments.preset != "all":
                raise
            result[name] = {"available": False, "error": str(error)}
    failed = False
    if arguments.check:
        result["checks"] = acceptance_checks(result)
        failed = any(not check["pass"] for check in result["checks"])
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 3 if failed else 0


def message_timestamp_ns(message: dict[str, Any]) -> int:
    timestamp = message.get("data", {}).get("timestamp")
    if isinstance(timestamp, str):
        return parse_time_ns(timestamp)
    if isinstance(timestamp, dict) and "sec" in timestamp and "nsec" in timestamp:
        return int(timestamp["sec"]) * 1_000_000_000 + int(timestamp["nsec"])
    if "_log_time_ns" in message:
        return int(message["_log_time_ns"])
    log_time = message.get("log_time")
    if isinstance(log_time, (int, float)):
        return int(round(float(log_time) * 1_000_000_000))
    raise AnalysisError("message has no usable timestamp")


def color_rgba(value: Any, default: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    if not isinstance(value, dict):
        return default
    return tuple(
        max(0, min(255, round(float(value.get(component, fallback)) * 255)))
        for component, fallback in zip(
            ("r", "g", "b", "a"),
            (default[0] / 255, default[1] / 255, default[2] / 255, default[3] / 255),
        )
    )  # type: ignore[return-value]


def draw_annotations(image: Any, annotations: dict[str, Any]) -> Any:
    from PIL import ImageDraw, ImageFont

    overlay = image.convert("RGBA")
    draw = ImageDraw.Draw(overlay, "RGBA")
    for annotation in annotations.get("points", []):
        points = [
            (float(point.get("x", 0)), float(point.get("y", 0)))
            for point in annotation.get("points", [])
        ]
        if not points:
            continue
        color = color_rgba(annotation.get("outlineColor"), (255, 255, 0, 255))
        width = max(1, round(float(annotation.get("thickness", 2))))
        kind = str(annotation.get("type", "POINTS"))
        if kind == "LINE_LOOP" and len(points) > 1:
            draw.line(points + [points[0]], fill=color, width=width)
        elif kind in {"LINE_STRIP", "LINE_LIST"} and len(points) > 1:
            if kind == "LINE_LIST":
                for index in range(0, len(points) - 1, 2):
                    draw.line(points[index : index + 2], fill=color, width=width)
            else:
                draw.line(points, fill=color, width=width)
        else:
            radius = max(2, width)
            for x, y in points:
                draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)
    for circle in annotations.get("circles", []):
        center = circle.get("position") or circle.get("center") or {}
        x, y = float(center.get("x", 0)), float(center.get("y", 0))
        radius = float(circle.get("diameter", circle.get("radius", 4)))
        if "diameter" in circle:
            radius /= 2
        fill = color_rgba(circle.get("fillColor"), (0, 0, 0, 0))
        outline = color_rgba(circle.get("outlineColor"), (255, 255, 0, 255))
        width = max(1, round(float(circle.get("thickness", 2))))
        draw.ellipse(
            (x - radius, y - radius, x + radius, y + radius),
            fill=fill,
            outline=outline,
            width=width,
        )
    font = ImageFont.load_default()
    for text in annotations.get("texts", []):
        position = text.get("position", {})
        xy = (float(position.get("x", 0)), float(position.get("y", 0)))
        content = str(text.get("text", ""))
        foreground = color_rgba(text.get("textColor"), (255, 255, 255, 255))
        background = color_rgba(text.get("backgroundColor"), (0, 0, 0, 170))
        try:
            box = draw.textbbox(xy, content, font=font)
            draw.rectangle(box, fill=background)
        except AttributeError:
            pass
        draw.text(xy, content, fill=foreground, font=font)
    return overlay.convert("RGB")


def select_images(
    path: Path, start_ns: int, end_ns: int, count: int
) -> list[tuple[int, dict[str, Any]]]:
    span = end_ns - start_ns
    if count == 1:
        targets = [start_ns + span // 2]
    else:
        targets = [start_ns + ((span - 1) * index) // (count - 1) for index in range(count)]
    candidates: list[tuple[int, dict[str, Any]] | None] = [None] * len(targets)
    for message in stream_messages(path, [IMAGE_TOPIC], start_ns, end_ns):
        timestamp_ns = message_timestamp_ns(message)
        for index, target in enumerate(targets):
            current = candidates[index]
            if current is None or abs(timestamp_ns - target) < abs(current[0] - target):
                candidates[index] = (timestamp_ns, message)
    selected: list[tuple[int, dict[str, Any]]] = []
    selected_times: set[int] = set()
    for candidate in candidates:
        if candidate is not None and candidate[0] not in selected_times:
            selected.append(candidate)
            selected_times.add(candidate[0])
    if not selected:
        raise AnalysisError("no camera images found in the selected time window")
    return selected


def default_output_dir(path: Path) -> Path:
    safe_stem = re.sub(r"[^A-Za-z0-9_.-]+", "-", path.stem).strip("-") or "recording"
    root = Path("/tmp/mcap-analysis")
    root.mkdir(parents=True, exist_ok=True)
    return Path(tempfile.mkdtemp(prefix=f"{safe_stem}-frames-", dir=root))


def command_frames(arguments: argparse.Namespace) -> int:
    from io import BytesIO
    try:
        from PIL import Image
    except ImportError as error:
        raise AnalysisError("Pillow is required for frame extraction and overlays") from error

    path = validate_input(arguments.file)
    start_ns, end_ns = time_bounds(arguments)
    assert start_ns is not None and end_ns is not None
    if arguments.count < 1 or arguments.count > MAX_FRAME_COUNT:
        raise AnalysisError(f"--count must be between 1 and {MAX_FRAME_COUNT}")
    topics = LAYER_TOPICS[arguments.layers]
    selected = select_images(path, start_ns, end_ns, arguments.count)
    selected_times = {timestamp for timestamp, _ in selected}
    by_time: dict[int, dict[str, dict[str, Any]]] = {timestamp: {} for timestamp in selected_times}
    for message in stream_messages(path, topics, start_ns, end_ns):
        timestamp = message_timestamp_ns(message)
        if timestamp in by_time:
            by_time[timestamp][str(message.get("topic"))] = message.get("data", {})

    output = (
        Path(arguments.output).expanduser().resolve()
        if arguments.output
        else default_output_dir(path)
    )
    if arguments.output and output.exists():
        if not output.is_dir() or any(output.iterdir()):
            raise AnalysisError(f"explicit frame output must be a new or empty directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, Any] = {
        "source": str(path),
        "source_size_bytes": path.stat().st_size,
        "window": {"start_ns": start_ns, "end_ns": end_ns},
        "layers": arguments.layers,
        "annotation_topics": topics,
        "frames": [],
    }
    for index, (timestamp_ns, message) in enumerate(selected):
        data = message.get("data", {})
        if data.get("format") not in {"jpeg", "jpg"}:
            raise AnalysisError(
                f"unsupported image format at {format_time_ns(timestamp_ns)}: "
                f"{data.get('format')}"
            )
        try:
            jpeg = base64.b64decode(data["data"], validate=True)
            image = Image.open(BytesIO(jpeg))
            image.load()
        except (KeyError, ValueError, OSError) as error:
            raise AnalysisError(
                f"invalid JPEG payload at {format_time_ns(timestamp_ns)}"
            ) from error
        basename = f"{index:02d}-{timestamp_ns}"
        raw_path = output / f"{basename}-raw.jpg"
        overlay_path = output / f"{basename}-{arguments.layers}.png"
        raw_path.write_bytes(jpeg)
        rendered = image.convert("RGB")
        available = by_time.get(timestamp_ns, {})
        for topic in topics:
            if topic in available:
                rendered = draw_annotations(rendered, available[topic])
        rendered.save(overlay_path, format="PNG")
        manifest["frames"].append(
            {
                "timestamp_ns": timestamp_ns,
                "timestamp": format_time_ns(timestamp_ns),
                "sequence": message.get("sequence"),
                "raw": str(raw_path),
                "overlay": str(overlay_path),
                "available_annotations": [topic for topic in topics if topic in available],
                "missing_annotations": [topic for topic in topics if topic not in available],
            }
        )
    manifest_path = output / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {"frames": len(selected), "output": str(output), "manifest": str(manifest_path)},
            ensure_ascii=False,
        )
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Bounded, read-only analysis for rm_vision_2027 MCAP recordings"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser("inspect", help="summarize structure and diagnostics")
    inspect_parser.add_argument("file")
    inspect_parser.add_argument("--json", action="store_true")
    inspect_parser.add_argument("--examples", type=int, default=1, choices=range(0, 4))
    inspect_parser.set_defaults(handler=command_inspect)

    query_parser = subparsers.add_parser("query", help="query decoded messages")
    query_parser.add_argument("file")
    query_parser.add_argument("--topic", required=True)
    query_parser.add_argument("--field", action="append", default=[])
    query_parser.add_argument(
        "--where",
        action="append",
        nargs=3,
        metavar=("FIELD", "OP", "VALUE"),
        default=[],
        help="OP is eq, ne, lt, le, gt, ge, contains, or exists",
    )
    query_parser.add_argument("--limit", type=int, default=DEFAULT_LIMIT)
    query_parser.add_argument("--output")
    add_time_arguments(query_parser)
    query_parser.set_defaults(handler=command_query)

    stats_parser = subparsers.add_parser("stats", help="calculate bounded numeric statistics")
    stats_parser.add_argument("file")
    stats_parser.add_argument("--topic", required=True)
    stats_parser.add_argument("--field", action="append", required=True)
    stats_parser.add_argument("--json", action="store_true")
    add_time_arguments(stats_parser)
    stats_parser.set_defaults(handler=command_stats)

    preset_parser = subparsers.add_parser("preset", help="run rm_vision diagnostic presets")
    preset_parser.add_argument("file")
    preset_parser.add_argument(
        "preset", choices=["performance", "pnp", "prediction", "control", "all"]
    )
    preset_parser.add_argument("--check", action="store_true")
    add_time_arguments(preset_parser)
    preset_parser.set_defaults(handler=command_preset)

    frames_parser = subparsers.add_parser("frames", help="extract synchronized image evidence")
    frames_parser.add_argument("file")
    frames_parser.add_argument("--count", type=int, default=DEFAULT_FRAME_COUNT)
    frames_parser.add_argument("--layers", choices=list(LAYER_TOPICS), default="pnp")
    frames_parser.add_argument("--output")
    add_time_arguments(frames_parser, required=True)
    frames_parser.set_defaults(handler=command_frames)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    arguments = parser.parse_args(argv)
    try:
        if hasattr(arguments, "limit") and arguments.limit < 0:
            raise AnalysisError("--limit must not be negative")
        for condition in getattr(arguments, "where", []):
            if condition[1] not in {"eq", "ne", "lt", "le", "gt", "ge", "contains", "exists"}:
                raise AnalysisError(f"unsupported comparison operation: {condition[1]}")
        return int(arguments.handler(arguments))
    except (AnalysisError, BrokenPipeError) as error:
        if isinstance(error, BrokenPipeError):
            return 0
        print(f"mcap-analyze: error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
