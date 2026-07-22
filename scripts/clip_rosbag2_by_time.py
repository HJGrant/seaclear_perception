#!/usr/bin/env python3
"""Clip a ROS 2 bag by bag-relative time without deserializing messages.

This keeps serialized CDR payloads unchanged, so custom message packages are
not required. It is intended for reducing large field rosbag2/sqlite3 bags
before image extraction or plotting.

Example:
  source /opt/ros/humble/setup.bash
  python3 clip_rosbag2_by_time.py /path/to/input_bag /path/to/output_bag \
    --start-sec 80 --end-sec 120
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def import_rosbag2() -> Any:
    try:
        import rosbag2_py
    except Exception as exc:
        raise RuntimeError(
            "rosbag2_py is not importable. Run inside a ROS 2 environment after "
            "sourcing /opt/ros/<distro>/setup.bash."
        ) from exc
    return rosbag2_py


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag", help="Input ROS 2 bag directory.")
    parser.add_argument("output_bag", help="Output ROS 2 bag directory. Must not already exist unless --overwrite.")
    parser.add_argument("--start-sec", type=float, required=True, help="Bag-relative start time in seconds.")
    parser.add_argument("--end-sec", type=float, default=None, help="Bag-relative end time in seconds. Default: until EOF.")
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument(
        "--topic",
        action="append",
        default=None,
        help="Optional topic to keep. Can be repeated. Default: keep all topics.",
    )
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--progress-every", type=int, default=10000)
    args = parser.parse_args()
    if args.start_sec < 0:
        parser.error("--start-sec must be >= 0")
    if args.end_sec is not None and args.end_sec <= args.start_sec:
        parser.error("--end-sec must be greater than --start-sec")
    return args


def main() -> int:
    args = parse_args()
    rosbag2_py = import_rosbag2()

    input_bag = Path(args.input_bag).expanduser().resolve()
    output_bag = Path(args.output_bag).expanduser().resolve()
    if output_bag.exists():
        if not args.overwrite:
            raise SystemExit(f"Output already exists: {output_bag}. Use --overwrite or choose another path.")
        import shutil

        shutil.rmtree(output_bag)

    storage_options = rosbag2_py.StorageOptions(uri=str(input_bag), storage_id=args.storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topics = reader.get_all_topics_and_types()
    topic_type_by_name = {topic.name: topic.type for topic in topics}
    topic_qos_by_name = {topic.name: getattr(topic, "offered_qos_profiles", "") for topic in topics}

    keep_topics = set(args.topic or topic_type_by_name.keys())
    missing = sorted(keep_topics - set(topic_type_by_name.keys()))
    if missing:
        raise SystemExit(f"Requested topic(s) not found: {missing}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id=args.storage_id),
        converter_options,
    )
    for name in sorted(keep_topics):
        metadata = rosbag2_py.TopicMetadata(
            name=name,
            type=topic_type_by_name[name],
            serialization_format="cdr",
            offered_qos_profiles=topic_qos_by_name.get(name, ""),
        )
        writer.create_topic(metadata)

    start_ns = int(args.start_sec * 1_000_000_000)
    end_ns = int(args.end_sec * 1_000_000_000) if args.end_sec is not None else None
    first_bag_time_ns: int | None = None
    read_count = 0
    written_count = 0
    first_written_rel_sec: float | None = None
    last_written_rel_sec: float | None = None

    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        stamp_ns = int(stamp_ns)
        if first_bag_time_ns is None:
            first_bag_time_ns = stamp_ns
        rel_ns = stamp_ns - first_bag_time_ns

        if end_ns is not None and rel_ns > end_ns:
            break

        read_count += 1
        if rel_ns < start_ns or topic not in keep_topics:
            continue

        writer.write(topic, serialized, stamp_ns)
        written_count += 1
        rel_sec = rel_ns / 1e9
        if first_written_rel_sec is None:
            first_written_rel_sec = rel_sec
        last_written_rel_sec = rel_sec
        if args.progress_every > 0 and written_count % args.progress_every == 0:
            print(f"written {written_count} messages, t_rel={rel_sec:.3f}s", flush=True)

    manifest = {
        "input_bag": str(input_bag),
        "output_bag": str(output_bag),
        "start_sec": args.start_sec,
        "end_sec": args.end_sec,
        "kept_topics": sorted(keep_topics),
        "read_messages": read_count,
        "written_messages": written_count,
        "first_written_rel_sec": first_written_rel_sec,
        "last_written_rel_sec": last_written_rel_sec,
    }
    output_bag.mkdir(parents=True, exist_ok=True)
    with (output_bag / "clip_manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
