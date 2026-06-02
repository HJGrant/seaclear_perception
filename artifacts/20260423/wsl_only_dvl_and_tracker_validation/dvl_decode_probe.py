from __future__ import annotations

import signal
import sqlite3
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from marine_acoustic_msgs.msg import Dvl


REPO_ROOT = Path(__file__).resolve().parents[3]
TARGET_BAGS = [
    REPO_ROOT / "rosbag2_2026_04_22-15_22_39",
    REPO_ROOT / "rosbag2_2026_04_22-16_47_57-diving-noimage",
]


class DvlProbe(Node):
    def __init__(self) -> None:
        super().__init__("dvl_decode_probe")
        self.samples: list[tuple[float, float, float, int]] = []
        self.create_subscription(Dvl, "/waterlinked_dvl_driver/velocity_report", self._cb, 10)

    def _cb(self, msg: Dvl) -> None:
        self.samples.append(
            (
                float(msg.altitude),
                float(msg.speed_gnd),
                float(msg.course_gnd),
                int(msg.num_good_beams),
            )
        )


def first_db3(bag_dir: Path) -> Path:
    return sorted(bag_dir.glob("*.db3"))[0]


def decode_direct_from_db(bag_dir: Path) -> list[str]:
    conn = sqlite3.connect(str(first_db3(bag_dir)))
    cur = conn.cursor()
    cur.execute("SELECT id, type FROM topics WHERE name=?", ("/waterlinked_dvl_driver/velocity_report",))
    topic_id, topic_type = cur.fetchone()
    msg_type = get_message(topic_type)
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 3", (topic_id,))
    lines = [
        f"bag={bag_dir.relative_to(REPO_ROOT)}",
        f"topic_type={topic_type}",
    ]
    for ts, blob in cur.fetchall():
        msg = deserialize_message(blob, msg_type)
        lines.append(
            "db_decode "
            f"ts={int(ts)} altitude={float(msg.altitude):.6f} "
            f"speed_gnd={float(msg.speed_gnd):.6f} course_gnd={float(msg.course_gnd):.6f} "
            f"num_good_beams={int(msg.num_good_beams)}"
        )
    conn.close()
    return lines


def replay_subscribe(bag_dir: Path) -> list[str]:
    rclpy.init()
    node = DvlProbe()
    bag = subprocess.Popen(
        [
            "ros2",
            "bag",
            "play",
            str(bag_dir),
            "--topics",
            "/waterlinked_dvl_driver/velocity_report",
        ]
    )
    try:
        deadline = time.time() + 15.0
        while time.time() < deadline and len(node.samples) < 3 and bag.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        lines = [f"subscriber_count={len(node.samples)}"]
        for altitude, speed_gnd, course_gnd, beams in node.samples[:3]:
            lines.append(
                "subscriber_sample "
                f"altitude={altitude:.6f} speed_gnd={speed_gnd:.6f} "
                f"course_gnd={course_gnd:.6f} num_good_beams={beams}"
            )
        return lines
    finally:
        if bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try:
                bag.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                bag.kill()
                bag.wait(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    for bag in TARGET_BAGS:
        print("==", bag.relative_to(REPO_ROOT), "==")
        for line in decode_direct_from_db(bag):
            print(line)
        for line in replay_subscribe(bag):
            print(line)


if __name__ == "__main__":
    main()
