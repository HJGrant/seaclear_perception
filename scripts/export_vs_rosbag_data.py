#!/usr/bin/env python3
"""Export selected ROS 2 bag topics to CSV and image frames.

Example:
  source /opt/ros/humble/setup.bash
  source /workspaces/oakd_isaac_ros/install/setup.bash
  python3 export_vs_rosbag_data.py \
    /path/to/ctrl_bag_2026-06-08_11-59-10-staged_pid_xy-kp_0.050-kd_0.0008 \
    --output-dir /path/to/exported_vs_data \
    --plot

The CSV time axis is bag-relative time in seconds (`t_rel_sec`), so all exported
topics can be plotted on the same x-axis without reopening the rosbag.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any


DEFAULT_DATA_TOPICS = (
    "/sc2/visual_servo/centroid_error",
    "/sc2/velocity_controller_node/outer/console_signals",
    "/sc2/dvl/dvl",
    "/waterlinked_dvl_driver/velocity_report",
)

DEFAULT_IMAGE_TOPIC = "/oak/rgb/image_raw"


def sanitize_topic(topic: str) -> str:
    name = topic.strip("/") or "root"
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name)


def stamp_to_ns(stamp: Any) -> int | None:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nsec = getattr(stamp, "nanosec", None)
    if sec is None or nsec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nsec)


def message_header_stamp_ns(msg: Any) -> int | None:
    header = getattr(msg, "header", None)
    return stamp_to_ns(getattr(header, "stamp", None))


def is_scalar(value: Any) -> bool:
    return value is None or isinstance(value, (bool, int, float, str))


def flatten_value(
    value: Any,
    prefix: str,
    out: dict[str, Any],
    *,
    max_sequence_length: int,
) -> None:
    if is_scalar(value):
        out[prefix] = value
        return

    if isinstance(value, (bytes, bytearray)):
        out[prefix] = f"<{len(value)} bytes>"
        return

    if isinstance(value, (list, tuple)):
        out[f"{prefix}.length"] = len(value)
        if len(value) > max_sequence_length:
            out[f"{prefix}.truncated"] = True
            return
        for i, item in enumerate(value):
            flatten_value(
                item,
                f"{prefix}.{i}",
                out,
                max_sequence_length=max_sequence_length,
            )
        return

    fields = getattr(value, "get_fields_and_field_types", None)
    if callable(fields):
        for field_name in fields().keys():
            flatten_value(
                getattr(value, field_name),
                f"{prefix}.{field_name}" if prefix else field_name,
                out,
                max_sequence_length=max_sequence_length,
            )
        return

    out[prefix] = str(value)


def flatten_message(msg: Any, *, max_sequence_length: int) -> dict[str, Any]:
    row: dict[str, Any] = {}
    flatten_value(msg, "", row, max_sequence_length=max_sequence_length)
    return row


def sorted_fieldnames(rows: list[dict[str, Any]]) -> list[str]:
    priority = ["bag_time_ns", "bag_time_sec", "t_rel_sec", "header_stamp_ns", "header_stamp_sec"]
    keys = set()
    for row in rows:
        keys.update(row.keys())
    rest = sorted(k for k in keys if k not in priority)
    return [k for k in priority if k in keys] + rest


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = sorted_fieldnames(rows)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def import_rosbag_tools() -> tuple[Any, Any, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:
        raise RuntimeError(
            "ROS 2 Python modules are not importable. Run this in the ROS 2 "
            "container/session, after sourcing /opt/ros/<distro>/setup.bash and "
            "your workspace install/setup.bash."
        ) from exc
    return rosbag2_py, deserialize_message, get_message


def image_msg_to_array(msg: Any, *, prefer_bgr: bool = True) -> Any:
    try:
        from cv_bridge import CvBridge

        bridge = CvBridge()
        if prefer_bgr and getattr(msg, "encoding", "").lower() in {"rgb8", "bgr8", "rgba8", "bgra8"}:
            return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception:
        pass

    try:
        import numpy as np
    except Exception as exc:
        raise RuntimeError("Image export needs either cv_bridge or numpy installed.") from exc

    encoding = getattr(msg, "encoding", "").lower()
    height = int(msg.height)
    width = int(msg.width)
    data = bytes(msg.data)

    dtype = np.uint16 if encoding in {"mono16", "16uc1"} else np.uint8
    channels_by_encoding = {
        "mono8": 1,
        "8uc1": 1,
        "mono16": 1,
        "16uc1": 1,
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise RuntimeError(f"Unsupported image encoding without cv_bridge: {msg.encoding!r}")

    arr = np.frombuffer(data, dtype=dtype)
    if channels == 1:
        arr = arr.reshape((height, width))
    else:
        arr = arr.reshape((height, width, channels))
        if prefer_bgr and encoding in {"rgb8", "rgba8"}:
            arr = arr[..., [2, 1, 0] + ([3] if channels == 4 else [])]
    return arr


def write_image(path: Path, msg: Any, image_format: str) -> None:
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("Image export needs OpenCV Python (`cv2`) installed.") from exc

    arr = image_msg_to_array(msg, prefer_bgr=True)
    params: list[int] = []
    fmt = image_format.lower()
    if fmt in {"jpg", "jpeg"}:
        params = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
    elif fmt == "png":
        params = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]

    ok = cv2.imwrite(str(path), arr, params)
    if not ok:
        raise RuntimeError(f"cv2.imwrite failed for {path}")


def timestamp_filename(ns: int, suffix: str) -> str:
    sec = ns // 1_000_000_000
    nsec = ns % 1_000_000_000
    return f"{sec}_{nsec:09d}.{suffix}"


def numeric_columns(rows: list[dict[str, Any]]) -> list[str]:
    if not rows:
        return []
    skip_prefixes = ("bag_time", "header_stamp")
    cols = []
    for key in sorted_fieldnames(rows):
        if key == "t_rel_sec" or key.startswith(skip_prefixes):
            continue
        values = [row.get(key) for row in rows[: min(len(rows), 200)]]
        numeric = False
        for value in values:
            if value in ("", None):
                continue
            if isinstance(value, bool):
                continue
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                numeric = True
                break
        if numeric:
            cols.append(key)
    return cols


def preferred_plot_columns(topic: str, rows: list[dict[str, Any]], max_cols: int) -> list[str]:
    cols = numeric_columns(rows)
    if topic.endswith("/centroid_error"):
        preferred = ["x", "y", "z"]
    elif topic.endswith("/dvl/dvl"):
        preferred = [
            "twist.twist.linear.x",
            "twist.twist.linear.y",
            "twist.twist.linear.z",
            "twist.twist.angular.x",
            "twist.twist.angular.y",
            "twist.twist.angular.z",
        ]
    elif topic.endswith("/velocity_report"):
        preferred = [
            "altitude",
            "velocity.x",
            "velocity.y",
            "velocity.z",
            "velocity_valid",
            "fom",
        ]
    else:
        preferred = cols

    selected = [c for c in preferred if c in cols]
    if not selected:
        selected = [c for c in cols if "covariance" not in c.lower()]
    return selected[:max_cols]


def make_plots(output_dir: Path, rows_by_topic: dict[str, list[dict[str, Any]]], *, max_cols: int) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError("Plotting needs matplotlib installed. Re-run without --plot if unavailable.") from exc

    plot_dir = output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    for topic, rows in rows_by_topic.items():
        cols = preferred_plot_columns(topic, rows, max_cols)
        if not rows or not cols:
            continue

        fig, ax = plt.subplots(figsize=(10, 4.8), constrained_layout=True)
        t = [float(row["t_rel_sec"]) for row in rows]
        for col in cols:
            y = []
            for row in rows:
                value = row.get(col)
                y.append(float(value) if isinstance(value, (int, float)) else math.nan)
            ax.plot(t, y, label=col, linewidth=1.4)
        ax.set_title(topic)
        ax.set_xlabel("time from bag start [s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize="small")
        fig.savefig(plot_dir / f"{sanitize_topic(topic)}.png", dpi=160)
        plt.close(fig)


def export_bag(args: argparse.Namespace) -> dict[str, Any]:
    rosbag2_py, deserialize_message, get_message = import_rosbag_tools()

    bag_path = Path(args.bag).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    csv_dir = output_dir / "csv"
    image_dir = output_dir / "images" / sanitize_topic(args.image_topic)
    output_dir.mkdir(parents=True, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=args.storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected_data_topics = tuple(args.data_topic)
    topics_to_read = set(selected_data_topics)
    if not args.no_images and args.image_topic:
        topics_to_read.add(args.image_topic)

    missing = sorted(topic for topic in topics_to_read if topic not in topic_types)
    if missing:
        print("WARNING: missing topic(s): " + ", ".join(missing), file=sys.stderr)

    msg_classes = {}
    for topic in topics_to_read:
        if topic in topic_types:
            msg_classes[topic] = get_message(topic_types[topic])

    rows_by_topic: dict[str, list[dict[str, Any]]] = defaultdict(list)
    exported_counts = defaultdict(int)
    image_count = 0
    first_bag_time_ns: int | None = None
    start_ns = int(args.start_sec * 1_000_000_000) if args.start_sec is not None else None
    end_ns = int(args.end_sec * 1_000_000_000) if args.end_sec is not None else None

    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if first_bag_time_ns is None:
            first_bag_time_ns = int(bag_time_ns)
        rel_ns = int(bag_time_ns) - first_bag_time_ns
        if start_ns is not None and rel_ns < start_ns:
            continue
        if end_ns is not None and rel_ns > end_ns:
            continue
        if topic not in topics_to_read or topic not in msg_classes:
            continue

        msg = deserialize_message(serialized, msg_classes[topic])
        header_ns = message_header_stamp_ns(msg)

        if topic in selected_data_topics:
            row = {
                "bag_time_ns": int(bag_time_ns),
                "bag_time_sec": int(bag_time_ns) / 1e9,
                "t_rel_sec": rel_ns / 1e9,
            }
            if header_ns is not None:
                row["header_stamp_ns"] = header_ns
                row["header_stamp_sec"] = header_ns / 1e9
            row.update(flatten_message(msg, max_sequence_length=args.max_sequence_length))
            rows_by_topic[topic].append(row)
            exported_counts[topic] += 1

        if not args.no_images and topic == args.image_topic:
            if args.max_images is not None and image_count >= args.max_images:
                continue
            if image_count % args.image_step != 0:
                image_count += 1
                continue
            image_dir.mkdir(parents=True, exist_ok=True)
            image_ns = header_ns if header_ns not in (None, 0) else int(bag_time_ns)
            suffix = "jpg" if args.image_format == "jpeg" else args.image_format
            out_path = image_dir / timestamp_filename(image_ns, suffix)
            write_image(out_path, msg, args.image_format)
            exported_counts[topic] += 1
            image_count += 1

    csv_paths = {}
    for topic, rows in rows_by_topic.items():
        path = csv_dir / f"{sanitize_topic(topic)}.csv"
        write_csv(path, rows)
        csv_paths[topic] = str(path)

    if args.plot:
        make_plots(output_dir, rows_by_topic, max_cols=args.max_plot_columns)

    manifest = {
        "bag": str(bag_path),
        "output_dir": str(output_dir),
        "data_topics": list(selected_data_topics),
        "image_topic": None if args.no_images else args.image_topic,
        "topic_types": {topic: topic_types.get(topic) for topic in sorted(topics_to_read)},
        "missing_topics": missing,
        "exported_counts": dict(exported_counts),
        "csv_paths": csv_paths,
        "image_dir": None if args.no_images else str(image_dir),
        "time_axis": "t_rel_sec is seconds from the first message timestamp in the bag read order.",
    }
    with (output_dir / "export_manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
    return manifest


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="ROS 2 bag directory, or basename accepted by rosbag2_py.")
    parser.add_argument("--output-dir", default=None, help="Directory for CSV, plots, images, and manifest.")
    parser.add_argument("--storage-id", default="sqlite3", help="rosbag2 storage id, usually sqlite3.")
    parser.add_argument(
        "--data-topic",
        action="append",
        default=None,
        help="Data topic to export to CSV. Can be repeated. Defaults to the VS/DVL topics.",
    )
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC, help="Image topic to export.")
    parser.add_argument("--no-images", action="store_true", help="Do not export image frames.")
    parser.add_argument("--image-format", choices=("png", "jpg", "jpeg"), default="png")
    parser.add_argument("--image-step", type=int, default=1, help="Export every Nth image message.")
    parser.add_argument("--max-images", type=int, default=None, help="Optional cap for exported images.")
    parser.add_argument("--start-sec", type=float, default=None, help="Optional start time in bag-relative seconds.")
    parser.add_argument("--end-sec", type=float, default=None, help="Optional end time in bag-relative seconds.")
    parser.add_argument("--max-sequence-length", type=int, default=80)
    parser.add_argument("--plot", action="store_true", help="Also create quick-look PNG plots from the exported rows.")
    parser.add_argument("--max-plot-columns", type=int, default=12)

    args = parser.parse_args(argv)
    if args.output_dir is None:
        bag_name = Path(args.bag).expanduser().name
        args.output_dir = f"{bag_name}_vs_export"
    if args.data_topic is None:
        args.data_topic = list(DEFAULT_DATA_TOPICS)
    if args.image_step < 1:
        parser.error("--image-step must be >= 1")
    if args.max_images is not None and args.max_images < 0:
        parser.error("--max-images must be >= 0")
    if args.start_sec is not None and args.start_sec < 0:
        parser.error("--start-sec must be >= 0")
    if args.end_sec is not None and args.end_sec < 0:
        parser.error("--end-sec must be >= 0")
    if args.start_sec is not None and args.end_sec is not None and args.end_sec < args.start_sec:
        parser.error("--end-sec must be >= --start-sec")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    manifest = export_bag(args)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
