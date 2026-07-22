#!/usr/bin/env python3
"""Fast image-only extractor for ROS 2 bags.

This intentionally reads only one sensor_msgs/Image topic, so it does not need
custom message packages such as marine_acoustic_msgs or state_estimation.

Example:
  source /opt/ros/humble/setup.bash
  python3 export_rosbag_images_fast.py /path/to/bag \
    --output-dir /path/to/images --image-format jpg --image-step 30 --max-images 20
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def import_ros_tools() -> tuple[Any, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
    except Exception as exc:
        raise RuntimeError(
            "ROS 2 Python modules are not importable. Run inside a ROS 2 "
            "environment after sourcing /opt/ros/<distro>/setup.bash."
        ) from exc
    return rosbag2_py, (deserialize_message, Image)


def stamp_to_ns(stamp: Any) -> int | None:
    sec = getattr(stamp, "sec", None)
    nsec = getattr(stamp, "nanosec", None)
    if sec is None or nsec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nsec)


def image_stamp_ns(msg: Any, bag_time_ns: int) -> int:
    header = getattr(msg, "header", None)
    stamp_ns = stamp_to_ns(getattr(header, "stamp", None))
    if stamp_ns not in (None, 0):
        return int(stamp_ns)
    return int(bag_time_ns)


def timestamp_filename(ns: int, suffix: str) -> str:
    sec = ns // 1_000_000_000
    nsec = ns % 1_000_000_000
    return f"{sec}_{nsec:09d}.{suffix}"


def image_msg_to_bgr(msg: Any) -> Any:
    try:
        from cv_bridge import CvBridge

        return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception:
        pass

    import numpy as np

    encoding = str(msg.encoding).lower()
    height = int(msg.height)
    width = int(msg.width)
    data = bytes(msg.data)

    if encoding == "rgb8":
        arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
        return arr[..., ::-1]
    if encoding == "bgr8":
        return np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
    if encoding == "rgba8":
        arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 4))
        return arr[..., [2, 1, 0]]
    if encoding == "bgra8":
        arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 4))
        return arr[..., :3]
    if encoding in {"mono8", "8uc1"}:
        return np.frombuffer(data, dtype=np.uint8).reshape((height, width))

    raise RuntimeError(f"Unsupported image encoding without cv_bridge: {msg.encoding!r}")


def write_image(path: Path, msg: Any, image_format: str) -> None:
    import cv2

    arr = image_msg_to_bgr(msg)
    fmt = image_format.lower()
    params: list[int] = []
    if fmt in {"jpg", "jpeg"}:
        params = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
    elif fmt == "png":
        params = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]

    ok = cv2.imwrite(str(path), arr, params)
    if not ok:
        raise RuntimeError(f"cv2.imwrite failed for {path}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="ROS 2 bag directory.")
    parser.add_argument("--output-dir", required=True, help="Directory for images and manifest.")
    parser.add_argument("--image-topic", default="/oak/rgb/image_raw")
    parser.add_argument("--image-format", choices=("jpg", "jpeg", "png"), default="jpg")
    parser.add_argument("--image-step", type=int, default=1, help="Save every Nth image.")
    parser.add_argument("--max-images", type=int, default=None, help="Stop after saving this many images.")
    parser.add_argument("--start-sec", type=float, default=None, help="Bag-relative start time.")
    parser.add_argument("--end-sec", type=float, default=None, help="Bag-relative end time.")
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("--progress-every", type=int, default=100, help="Print progress every N saved images.")
    args = parser.parse_args(argv)

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
    rosbag2_py, image_tools = import_ros_tools()
    deserialize_message, image_cls = image_tools

    bag_path = Path(args.bag).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=args.storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if args.image_topic not in topic_types:
        raise RuntimeError(
            f"Image topic {args.image_topic!r} not found. Available topics: {sorted(topic_types)}"
        )
    if topic_types[args.image_topic] != "sensor_msgs/msg/Image":
        raise RuntimeError(
            f"Topic {args.image_topic!r} has type {topic_types[args.image_topic]!r}, "
            "expected sensor_msgs/msg/Image."
        )

    start_ns = int(args.start_sec * 1_000_000_000) if args.start_sec is not None else None
    end_ns = int(args.end_sec * 1_000_000_000) if args.end_sec is not None else None
    suffix = "jpg" if args.image_format == "jpeg" else args.image_format

    first_bag_time_ns: int | None = None
    seen_images = 0
    saved_images = 0
    first_saved_rel_sec: float | None = None
    last_saved_rel_sec: float | None = None

    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        bag_time_ns = int(bag_time_ns)
        if first_bag_time_ns is None:
            first_bag_time_ns = bag_time_ns
        rel_ns = bag_time_ns - first_bag_time_ns

        if start_ns is not None and rel_ns < start_ns:
            continue
        if end_ns is not None and rel_ns > end_ns:
            break
        if topic != args.image_topic:
            continue

        if seen_images % args.image_step != 0:
            seen_images += 1
            continue

        msg = deserialize_message(serialized, image_cls)
        stamp_ns = image_stamp_ns(msg, bag_time_ns)
        out_path = image_dir / timestamp_filename(stamp_ns, suffix)
        write_image(out_path, msg, args.image_format)

        rel_sec = rel_ns / 1e9
        if first_saved_rel_sec is None:
            first_saved_rel_sec = rel_sec
        last_saved_rel_sec = rel_sec
        saved_images += 1
        seen_images += 1

        if args.progress_every > 0 and saved_images % args.progress_every == 0:
            print(f"saved {saved_images} images, t_rel={rel_sec:.3f}s", flush=True)

        if args.max_images is not None and saved_images >= args.max_images:
            break

    manifest = {
        "bag": str(bag_path),
        "image_topic": args.image_topic,
        "image_topic_type": topic_types.get(args.image_topic),
        "output_dir": str(output_dir),
        "image_dir": str(image_dir),
        "image_format": args.image_format,
        "image_step": args.image_step,
        "max_images": args.max_images,
        "seen_image_messages": seen_images,
        "saved_images": saved_images,
        "first_saved_rel_sec": first_saved_rel_sec,
        "last_saved_rel_sec": last_saved_rel_sec,
    }
    with (output_dir / "image_export_manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
