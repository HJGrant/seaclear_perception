from __future__ import annotations

import csv
import math
import sqlite3
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


REPO_ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_ROOT = REPO_ROOT / "artifacts/20260423/wsl_only_dvl_and_tracker_validation"
FIG_DIR = ARTIFACT_ROOT / "figures"


TARGET_BAGS = [
    REPO_ROOT / "rosbag2_2026_04_22-15_22_39",
    REPO_ROOT / "rosbag2_2026_04_22-16_47_57-diving-noimage",
    REPO_ROOT / "rosbag2_2026_04_10-17_38_57",
    REPO_ROOT / "src/deployment_env/assets/rosbag2_2026_04_13-17_56_18",
]


@dataclass
class BagSummary:
    bag_path: str
    dvl_present: bool
    dvl_type: str
    dvl_count: int
    detections_present: bool
    detections_count: int
    best_detection_count: int
    duration_sec: float
    total_frames: int
    zero_frames: int
    one_frames: int
    two_plus_frames: int
    same_class_two_plus_frames: int
    longest_two_plus_run_frames: int
    longest_two_plus_run_sec: float
    longest_two_plus_start_sec: float
    longest_two_plus_end_sec: float
    near_center_competitor_frames: int
    near_size_competitor_frames: int


def load_metadata(bag_dir: Path) -> dict:
    data = yaml.safe_load((bag_dir / "metadata.yaml").read_text())
    info = data["rosbag2_bagfile_information"]
    if not isinstance(info, dict):
        raise TypeError(f"unexpected metadata structure for {bag_dir}")
    return info


def topics_from_metadata(info: dict) -> dict[str, tuple[str, int]]:
    result = {}
    for entry in info.get("topics_with_message_count", []):
        if not isinstance(entry, dict):
            continue
        md = entry.get("topic_metadata", {})
        if not isinstance(md, dict):
            continue
        result[str(md.get("name"))] = (str(md.get("type")), int(entry.get("message_count", 0)))
    return result


def open_db3(bag_dir: Path) -> sqlite3.Connection:
    db3_files = sorted(bag_dir.glob("*.db3"))
    if not db3_files:
        raise FileNotFoundError(f"no db3 file found in {bag_dir}")
    return sqlite3.connect(str(db3_files[0]))


def iter_topic_messages(conn: sqlite3.Connection, topic_name: str) -> Iterable[tuple[int, object]]:
    cur = conn.cursor()
    cur.execute("SELECT id, type FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if row is None:
        return
    topic_id, topic_type = row
    msg_type = get_message(topic_type)
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,))
    for ts, blob in cur.fetchall():
        yield int(ts), deserialize_message(blob, msg_type)


def bbox_center(det) -> tuple[float, float]:
    return float(det.bbox.center.position.x), float(det.bbox.center.position.y)


def bbox_size(det) -> tuple[float, float]:
    return float(det.bbox.size_x), float(det.bbox.size_y)


def bbox_distance(a, b) -> float:
    au, av = bbox_center(a)
    bu, bv = bbox_center(b)
    return math.hypot(au - bu, av - bv)


def size_similarity(a, b) -> float:
    aw, ah = bbox_size(a)
    bw, bh = bbox_size(b)
    if min(aw, ah, bw, bh) <= 0.0:
        return float("inf")
    return max(abs(aw - bw) / max(aw, bw), abs(ah - bh) / max(ah, bh))


def class_id_of(det) -> str:
    if not det.results:
        return ""
    hyp = det.results[0].hypothesis
    return str(hyp.class_id)


def analyze_detection_stream(bag_dir: Path) -> tuple[dict, list[int]]:
    conn = open_db3(bag_dir)
    counts: dict[str, float] = {
        "total_frames": 0,
        "zero_frames": 0,
        "one_frames": 0,
        "two_plus_frames": 0,
        "same_class_two_plus_frames": 0,
        "longest_two_plus_run_frames": 0,
        "longest_two_plus_run_sec": 0.0,
        "longest_two_plus_start_sec": -1.0,
        "longest_two_plus_end_sec": -1.0,
        "near_center_competitor_frames": 0,
        "near_size_competitor_frames": 0,
    }
    frame_counts: list[int] = []
    current_run_frames = 0
    current_run_start = None
    first_ts = None
    last_ts = None
    for ts, msg in iter_topic_messages(conn, "/detections_output"):
        if first_ts is None:
            first_ts = ts
        dets = list(msg.detections)
        n = len(dets)
        frame_counts.append(n)
        counts["total_frames"] += 1
        if n == 0:
            counts["zero_frames"] += 1
        elif n == 1:
            counts["one_frames"] += 1
        else:
            counts["two_plus_frames"] += 1
            by_class = defaultdict(int)
            for det in dets:
                by_class[class_id_of(det)] += 1
            if any(v >= 2 and k != "" for k, v in by_class.items()):
                counts["same_class_two_plus_frames"] += 1
            close_pair = False
            similar_pair = False
            for i in range(n):
                for j in range(i + 1, n):
                    if bbox_distance(dets[i], dets[j]) <= 80.0:
                        close_pair = True
                    if size_similarity(dets[i], dets[j]) <= 0.20:
                        similar_pair = True
            if close_pair:
                counts["near_center_competitor_frames"] += 1
            if similar_pair:
                counts["near_size_competitor_frames"] += 1

        if n >= 2:
            current_run_frames += 1
            if current_run_start is None:
                current_run_start = ts
            counts["longest_two_plus_run_frames"] = max(
                counts["longest_two_plus_run_frames"],
                current_run_frames,
            )
        else:
            if current_run_frames and current_run_start is not None and last_ts is not None:
                run_sec = max(0.0, (last_ts - current_run_start) * 1e-9)
                if run_sec >= counts["longest_two_plus_run_sec"]:
                    counts["longest_two_plus_run_sec"] = run_sec
                    counts["longest_two_plus_start_sec"] = (current_run_start - first_ts) * 1e-9
                    counts["longest_two_plus_end_sec"] = (last_ts - first_ts) * 1e-9
            current_run_frames = 0
            current_run_start = None
        last_ts = ts

    if current_run_frames and current_run_start is not None and last_ts is not None:
        run_sec = max(0.0, (last_ts - current_run_start) * 1e-9)
        if run_sec >= counts["longest_two_plus_run_sec"]:
            counts["longest_two_plus_run_sec"] = run_sec
            counts["longest_two_plus_start_sec"] = (current_run_start - first_ts) * 1e-9
            counts["longest_two_plus_end_sec"] = (last_ts - first_ts) * 1e-9
    conn.close()
    return counts, frame_counts


def summarize_bag(bag_dir: Path) -> tuple[BagSummary, list[int]]:
    info = load_metadata(bag_dir)
    topics = topics_from_metadata(info)
    duration_sec = float(info.get("duration", {}).get("nanoseconds", 0)) * 1e-9
    dvl_type, dvl_count = topics.get("/waterlinked_dvl_driver/velocity_report", ("", 0))
    det_type, det_count = topics.get("/detections_output", ("", 0))
    _best_type, best_count = topics.get("/best_detection", ("", 0))
    if det_count > 0:
        det_stats, frame_counts = analyze_detection_stream(bag_dir)
    else:
        det_stats = {
            "total_frames": 0,
            "zero_frames": 0,
            "one_frames": 0,
            "two_plus_frames": 0,
            "same_class_two_plus_frames": 0,
            "longest_two_plus_run_frames": 0,
            "longest_two_plus_run_sec": 0.0,
            "longest_two_plus_start_sec": -1.0,
            "longest_two_plus_end_sec": -1.0,
            "near_center_competitor_frames": 0,
            "near_size_competitor_frames": 0,
        }
        frame_counts = []

    summary = BagSummary(
        bag_path=str(bag_dir.relative_to(REPO_ROOT)),
        dvl_present=dvl_count > 0,
        dvl_type=dvl_type,
        dvl_count=dvl_count,
        detections_present=det_count > 0,
        detections_count=det_count,
        best_detection_count=best_count,
        duration_sec=duration_sec,
        total_frames=int(det_stats["total_frames"]),
        zero_frames=int(det_stats["zero_frames"]),
        one_frames=int(det_stats["one_frames"]),
        two_plus_frames=int(det_stats["two_plus_frames"]),
        same_class_two_plus_frames=int(det_stats["same_class_two_plus_frames"]),
        longest_two_plus_run_frames=int(det_stats["longest_two_plus_run_frames"]),
        longest_two_plus_run_sec=float(det_stats["longest_two_plus_run_sec"]),
        longest_two_plus_start_sec=float(det_stats["longest_two_plus_start_sec"]),
        longest_two_plus_end_sec=float(det_stats["longest_two_plus_end_sec"]),
        near_center_competitor_frames=int(det_stats["near_center_competitor_frames"]),
        near_size_competitor_frames=int(det_stats["near_size_competitor_frames"]),
    )
    return summary, frame_counts


def write_csv(summaries: list[BagSummary]) -> None:
    out_path = ARTIFACT_ROOT / "multi_object_bag_audit.csv"
    with out_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(list(BagSummary.__dataclass_fields__.keys()))
        for s in summaries:
            writer.writerow([getattr(s, name) for name in BagSummary.__dataclass_fields__])


def plot_detection_timeseries(bag_name: str, frame_counts: list[int]) -> None:
    if not frame_counts:
        return
    plt.figure(figsize=(10, 3))
    plt.plot(range(len(frame_counts)), frame_counts, linewidth=1.2)
    plt.xlabel("frame index")
    plt.ylabel("detections")
    plt.title(f"detection count timeline: {bag_name}")
    plt.tight_layout()
    plt.savefig(FIG_DIR / f"{bag_name.replace('/', '_')}_detection_counts.png", dpi=160)
    plt.close()


def plot_multi_object_ratio(summaries: list[BagSummary]) -> None:
    labels = [Path(s.bag_path).name for s in summaries if s.total_frames > 0]
    values = [
        (100.0 * s.two_plus_frames / s.total_frames) if s.total_frames else 0.0
        for s in summaries if s.total_frames > 0
    ]
    if not labels:
        return
    plt.figure(figsize=(10, 4))
    plt.bar(labels, values)
    plt.ylabel("multi-object frames [%]")
    plt.title("multi-object frame ratio by bag")
    plt.xticks(rotation=20, ha="right")
    plt.tight_layout()
    plt.savefig(FIG_DIR / "multi_object_frame_ratio.png", dpi=160)
    plt.close()


def write_bag_inventory(summaries: list[BagSummary]) -> None:
    lines = [
        "# Bag Inventory",
        "",
        "| bag | duration[s] | DVL topic | DVL count | detections_output | detection frames | two+ frames | longest two+ run[s] | longest run window[s] |",
        "|---|---:|---|---:|---|---:|---:|---:|---|",
    ]
    for s in summaries:
        lines.append(
            f"| `{s.bag_path}` | {s.duration_sec:.2f} | "
            f"{'yes' if s.dvl_present else 'no'} ({s.dvl_type or '-'}) | {s.dvl_count} | "
            f"{'yes' if s.detections_present else 'no'} | {s.total_frames} | {s.two_plus_frames} | "
            f"{s.longest_two_plus_run_sec:.3f} | "
            f"{s.longest_two_plus_start_sec:.3f} -> {s.longest_two_plus_end_sec:.3f} |"
        )
    (ARTIFACT_ROOT / "bag_inventory.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_recommendation(summaries: list[BagSummary]) -> None:
    ranked = [s for s in summaries if s.total_frames > 0]
    ranked.sort(key=lambda s: (s.two_plus_frames, s.longest_two_plus_run_sec, s.same_class_two_plus_frames), reverse=True)
    if ranked and ranked[0].two_plus_frames > 0:
        best = ranked[0]
        conclusion = "yes"
        body = [
            "# Bag Recommendation",
            "",
            f"結論: multi-object 検証に使える bag は **{conclusion}**。",
            "",
            f"最有力は `{best.bag_path}`。",
            "",
            "- 理由:",
            f"  - `two_plus_frames={best.two_plus_frames}`",
            f"  - `same_class_two_plus_frames={best.same_class_two_plus_frames}`",
            f"  - `longest_two_plus_run_sec={best.longest_two_plus_run_sec:.3f}`",
            f"  - `longest_run_window_sec={best.longest_two_plus_start_sec:.3f}->{best.longest_two_plus_end_sec:.3f}`",
            f"  - `near_center_competitor_frames={best.near_center_competitor_frames}`",
            f"  - `near_size_competitor_frames={best.near_size_competitor_frames}`",
            "",
            f"候補区間: `{best.longest_two_plus_start_sec:.3f}s` から `{best.longest_two_plus_end_sec:.3f}s`。",
            "",
            "補足: bag だけで stress を十分に掛けるには弱いので、synthetic validation を併用する。",
        ]
    else:
        conclusion = "no"
        body = [
            "# Bag Recommendation",
            "",
            f"結論: multi-object 検証に使える bag は **{conclusion}**。",
            "",
            "理由: `/detections_output` を持つ bag に 2 件以上 frame がほぼ無い。",
            "",
            "したがって synthetic validation を実施する。",
        ]
    (ARTIFACT_ROOT / "bag_recommendation.md").write_text("\n".join(body) + "\n", encoding="utf-8")


def main() -> None:
    ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)

    summaries: list[BagSummary] = []
    for bag in TARGET_BAGS:
        summary, frame_counts = summarize_bag(bag)
        summaries.append(summary)
        plot_detection_timeseries(summary.bag_path, frame_counts)

    write_csv(summaries)
    plot_multi_object_ratio(summaries)
    write_bag_inventory(summaries)
    write_recommendation(summaries)


if __name__ == "__main__":
    main()
