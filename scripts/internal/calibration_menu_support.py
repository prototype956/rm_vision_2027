#!/usr/bin/env python3
"""Read-only configuration and session discovery for the calibration menu."""

import argparse
import math
import os
from pathlib import Path
import sys

try:
    import yaml
except ImportError:
    sys.exit("标定菜单需要 Python 3 和 PyYAML（系统包 python3-yaml）。")

PROJECT_ROOT = Path(__file__).resolve().parents[2]


def load_map(path):
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict) or value.get("schema_version") != 1:
        raise ValueError("需要 schema_version: 1 的 YAML 清单")
    return value


def resolve(path):
    path = Path(path)
    return path if path.is_absolute() else PROJECT_ROOT / path


def session_root(mode):
    if mode == "drift":
        return PROJECT_ROOT / "artifacts/imu_drift"
    config_root = resolve(os.environ.get("RM_VISION_CONFIG_ROOT") or "src/config")
    config_name = "extrinsic_calibration" if mode.startswith("simulation") else "camera_calibration"
    config = load_map(config_root / "tool" / f"{config_name}.yaml")
    output = config.get("output_dir")
    if not isinstance(output, str) or not output:
        raise ValueError("标定配置 output_dir 必须为非空路径")
    root = resolve(output)
    return root / "extrinsics" if mode == "real" else root


def board_valid(board, columns, rows):
    if not isinstance(board, dict):
        return False
    values = [board.get(columns), board.get(rows), board.get("square_size_mm")]
    return (all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                for v in values)
            and values[0] >= 2 and values[1] >= 2 and values[2] > 0)


def numbers(value, count):
    return (isinstance(value, list) and len(value) == count
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in value))


def extrinsics_valid(value):
    return (isinstance(value, dict) and value.get("schema_version") == 1
            and isinstance(value.get("placeholder"), bool)
            and all(numbers(value.get(key), 9)
                    for key in ("gimbal_r_imu", "gimbal_r_camera", "gimbal_r_muzzle"))
            and all(numbers(value.get(key), 3)
                    for key in ("gimbal_t_camera_m", "gimbal_t_muzzle_m")))


def transform_valid(value):
    return (isinstance(value, dict) and numbers(value.get("translation"), 3)
            and numbers(value.get("rotation_xyzw"), 4))


def active_images(folder, samples):
    if not isinstance(samples, list):
        raise ValueError("samples 必须为序列")
    active = []
    for sample in samples:
        if not isinstance(sample, dict) or not isinstance(sample.get("active"), bool):
            raise ValueError("样本必须具有 active 布尔字段")
        if not sample["active"]:
            continue
        image = sample.get("image")
        if not isinstance(image, str) or not image:
            raise ValueError("有效样本缺少 image 路径")
        path = Path(image)
        if path.is_absolute() or ".." in path.parts or not (folder / path).is_file():
            raise ValueError("有效样本图像不存在或路径不合法")
        active.append(sample)
    if not active:
        raise ValueError("没有有效样本")
    return active


def validate_session(folder, mode):
    value = load_map(folder / "session.yaml")
    if value.get("status") in {"capturing", "collecting"}:
        raise ValueError("采集尚未结束")
    kind = value.get("kind")
    if mode == "drift":
        if kind != "imu_static_drift" or not extrinsics_valid(value.get("extrinsics")):
            raise ValueError("不是漂移会话或缺少外参快照")
        if not (folder / "samples.csv").is_file():
            raise ValueError("缺少 samples.csv")
        return f"有效样本 {value.get('accepted', '未知')}"
    if mode == "intrinsic":
        if kind not in (None, "camera_intrinsics") or not isinstance(value.get("camera"), dict):
            raise ValueError("不是内参会话")
        camera = value["camera"]
        if (not isinstance(camera.get("device_name"), str)
                or not all(isinstance(camera.get(key), int) and not isinstance(camera[key], bool)
                           for key in ("sensor_width", "sensor_height", "output_width",
                                       "output_height", "roi_offset_x", "roi_offset_y", "exposure_us"))):
            raise ValueError("内参相机元数据不完整")
        if not board_valid(value.get("board"), "columns", "rows"):
            raise ValueError("内参棋盘参数缺失或无效")
        images = list((folder / "images").glob("*.png"))
        if not images:
            raise ValueError("没有用于解算的 PNG 图像")
        return f"图像 {len(images)}"
    if mode == "real":
        if kind != "single_yaw_handeye":
            raise ValueError("不是实机手眼会话")
        if (not isinstance(value.get("intrinsics"), dict)
                or not extrinsics_valid(value.get("extrinsics"))):
            raise ValueError("缺少内外参快照")
        intrinsics = value["intrinsics"]
        if (not numbers(intrinsics.get("camera_matrix"), 9)
                or not numbers(intrinsics.get("distortion"), 5)):
            raise ValueError("内参矩阵缺失或无效")
        if not board_valid(value.get("board"), "columns", "rows"):
            raise ValueError("实机棋盘参数无效")
        active = active_images(folder, value.get("samples"))
        if not all(numbers(s.get("world_q_imu_wxyz"), 4) for s in active):
            raise ValueError("有效样本缺少 IMU 姿态")
        return f"有效样本 {len(active)}"
    if (kind is not None or not isinstance(value.get("source"), dict)
            or value["source"].get("backend") != "talos"
            or value.get("translation_unit") != "m"
            or value.get("transform_convention") != "parent_t_child"):
        raise ValueError("不是仿真外参会话")
    if not board_valid(value.get("board"), "pattern_cols", "pattern_rows"):
        raise ValueError("仿真棋盘参数无效")
    active = active_images(folder, value.get("samples"))
    if not all(isinstance(s.get("camera"), dict)
               and transform_valid(s.get("world_t_gimbal"))
               and transform_valid(s.get("gimbal_t_camera_optical")) for s in active):
        raise ValueError("有效样本缺少同帧几何")
    return f"有效样本 {len(active)}"


def discover(mode):
    root = session_root(mode)
    if not root.is_dir():
        return []
    entries = []
    for folder in root.iterdir():
        if not folder.is_dir():
            continue
        try:
            detail = validate_session(folder, mode)
            if mode != "simulation-solutions":
                entries.append((folder, detail))
                continue
            for solution in (folder / "solutions").glob("*"):
                if not solution.is_dir():
                    continue
                try:
                    extrinsics = load_map(solution / "extrinsics.yaml")
                    report = load_map(solution / "report.yaml")
                    if (extrinsics.get("target") != "gimbal_t_camera_optical"
                            or not transform_valid(extrinsics.get("gimbal_t_camera_optical"))
                            or report.get("status") != "solved"):
                        raise ValueError("不是成功的仿真外参结果")
                    entries.append((solution, f"会话 {folder.name}"))
                except (OSError, ValueError, yaml.YAMLError) as error:
                    print(f"跳过 {solution}: {error}", file=sys.stderr)
        except (OSError, ValueError, yaml.YAMLError) as error:
            print(f"跳过 {folder}: {error}", file=sys.stderr)
    return sorted(entries, key=lambda entry: (entry[0].stat().st_mtime, str(entry[0])), reverse=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("intrinsic", "real", "simulation",
                                         "simulation-solutions", "drift"))
    args = parser.parse_args()
    try:
        for path, detail in discover(args.mode):
            # NUL delimiters preserve spaces and shell metacharacters in paths.
            sys.stdout.buffer.write(os.fsencode(path) + b"\0" + detail.encode() + b"\0")
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"无法读取标定配置: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
