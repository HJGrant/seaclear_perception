#!/usr/bin/env python3
from __future__ import annotations

import csv
import importlib
import json
import math
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Callable, Optional


ROOT = Path(__file__).resolve().parents[1]
TARGET_COMMIT = "7cb44747d0c8e9d5f563d3d48c23e385aeee4211"
OUTPUT_ROOT = ROOT / "_artifacts" / "staged_pid_horizontal_compare"
EPS = 1e-12


CURRENT_PARAMETER_OVERRIDES: dict[str, object] = {}


class FakeDuration:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds


class FakeTime:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds

    def __sub__(self, other: "FakeTime") -> FakeDuration:
        return FakeDuration(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return SimpleNamespace(sec=self.nanoseconds // 1_000_000_000, nanosec=self.nanoseconds % 1_000_000_000)


class FakeClock:
    def __init__(self, node: "FakeNode") -> None:
        self._node = node

    def now(self) -> FakeTime:
        return FakeTime(self._node._fake_now_ns)


class FakeLogger:
    def info(self, *args, **kwargs) -> None:
        pass

    def warn(self, *args, **kwargs) -> None:
        pass

    def error(self, *args, **kwargs) -> None:
        pass


class FakeParameter:
    def __init__(self, value) -> None:
        self.value = value

    def get_parameter_value(self):
        return SimpleNamespace(bool_value=bool(self.value))


class FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages: list[object] = []
        self.last_msg = None

    def publish(self, msg) -> None:
        self.messages.append(msg)
        self.last_msg = msg


class FakeNode:
    def __init__(self, name: str) -> None:
        self._name = name
        self._params: dict[str, FakeParameter] = {}
        self._publishers: dict[str, FakePublisher] = {}
        self._fake_now_ns = 0
        self._logger = FakeLogger()

    def declare_parameter(self, name: str, default_value):
        value = CURRENT_PARAMETER_OVERRIDES.get(name, default_value)
        param = FakeParameter(value)
        self._params[name] = param
        return param

    def get_parameter(self, name: str) -> FakeParameter:
        return self._params[name]

    def create_publisher(self, _msg_type, topic: str, _queue_size: int) -> FakePublisher:
        pub = FakePublisher(topic)
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, *_args, **_kwargs):
        return None

    def create_service(self, *_args, **_kwargs):
        return None

    def create_timer(self, *_args, **_kwargs):
        return None

    def get_clock(self) -> FakeClock:
        return FakeClock(self)

    def get_logger(self) -> FakeLogger:
        return self._logger

    def destroy_node(self) -> None:
        pass


def register_fake_ros_modules() -> None:
    if "rclpy" in sys.modules:
        return

    def make_msg_type(name: str):
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.twist = SimpleNamespace(
                linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )
            if name == "PoseStamped":
                self.pose = SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0))
            elif name == "PointStamped":
                self.point = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            elif name == "Vector3":
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
            elif name == "Vector3Stamped":
                self.vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            elif name == "String":
                self.data = ""
            elif name == "Float64":
                self.data = 0.0
            elif name == "CameraInfo":
                self.k = [0.0] * 9
            elif name == "Detection2D":
                self.bbox = SimpleNamespace(
                    center=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
                    size_x=0.0,
                    size_y=0.0,
                )

        return type(name, (), {"__init__": __init__})

    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    for name in ["TwistStamped", "PoseStamped", "PointStamped", "Vector3", "Vector3Stamped"]:
        setattr(geometry_msgs_msg, name, make_msg_type(name))
    geometry_msgs.msg = geometry_msgs_msg

    vision_msgs = ModuleType("vision_msgs")
    vision_msgs_msg = ModuleType("vision_msgs.msg")
    vision_msgs_msg.Detection2D = make_msg_type("Detection2D")
    vision_msgs.msg = vision_msgs_msg

    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    for name in ["String", "Float64"]:
        setattr(std_msgs_msg, name, make_msg_type(name))
    std_msgs.msg = std_msgs_msg

    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs_msg = ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CameraInfo = make_msg_type("CameraInfo")
    sensor_msgs.msg = sensor_msgs_msg

    std_srvs = ModuleType("std_srvs")
    std_srvs_srv = ModuleType("std_srvs.srv")
    std_srvs_srv.SetBool = type("SetBool", (), {})
    std_srvs.srv = std_srvs_srv

    tf2_ros = ModuleType("tf2_ros")
    tf2_ros.Buffer = type("Buffer", (), {"lookup_transform": lambda self, *args, **kwargs: SimpleNamespace()})
    tf2_ros.TransformListener = type("TransformListener", (), {"__init__": lambda self, *args, **kwargs: None})
    tf2_ros.TransformException = Exception

    tf2_geometry_msgs = ModuleType("tf2_geometry_msgs")
    tf2_geometry_msgs.do_transform_vector3 = lambda msg, _transform: msg
    tf2_geometry_msgs.do_transform_point = lambda msg, _transform: msg

    rclpy = ModuleType("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.try_shutdown = lambda *args, **kwargs: None
    rclpy_time = ModuleType("rclpy.time")
    rclpy_time.Time = type("Time", (), {})
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy.time = rclpy_time
    rclpy.node = rclpy_node

    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg
    sys.modules["vision_msgs"] = vision_msgs
    sys.modules["vision_msgs.msg"] = vision_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs_srv
    sys.modules["tf2_ros"] = tf2_ros
    sys.modules["tf2_geometry_msgs"] = tf2_geometry_msgs
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.time"] = rclpy_time
    sys.modules["rclpy.node"] = rclpy_node


def git_show(path: str) -> str:
    result = subprocess.run(
        ["git", "show", f"{TARGET_COMMIT}:{path}"],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout


def build_package_tree(base_dir: Path, package_name: str, source_loader: Callable[[str], str]) -> Path:
    pkg_dir = base_dir / package_name
    ibvs_dir = pkg_dir / "ibvs"
    ibvs_dir.mkdir(parents=True, exist_ok=True)
    (pkg_dir / "__init__.py").write_text("", encoding="utf-8")
    (ibvs_dir / "__init__.py").write_text("", encoding="utf-8")
    files = {
        "visual_servo_controller_node_implementation.py": "src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py",
        "pid.py": "src/sc2_visual_servo_controller/sc2_visual_servo_controller/pid.py",
        "ibvs/controller_math.py": "src/sc2_visual_servo_controller/sc2_visual_servo_controller/ibvs/controller_math.py",
    }
    for relative_dst, repo_src in files.items():
        dst = pkg_dir / relative_dst
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_text(source_loader(repo_src), encoding="utf-8")
    return pkg_dir


def load_controller_module(package_dir: Path, package_name: str):
    sys.path.insert(0, str(package_dir.parent))
    try:
        module_name = f"{package_name}.visual_servo_controller_node_implementation"
        if module_name in sys.modules:
            del sys.modules[module_name]
        return importlib.import_module(module_name)
    finally:
        sys.path.pop(0)


def current_source_loader(repo_src: str) -> str:
    return (ROOT / repo_src).read_text(encoding="utf-8")


def old_source_loader(repo_src: str) -> str:
    return git_show(repo_src)


def make_detection(x: float, y: float, size_x: float = 80.0, size_y: float = 100.0):
    from vision_msgs.msg import Detection2D

    msg = Detection2D()
    msg.bbox.center.position.x = float(x)
    msg.bbox.center.position.y = float(y)
    msg.bbox.size_x = float(size_x)
    msg.bbox.size_y = float(size_y)
    return msg


def make_pose(z: float):
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.pose.position.z = float(z)
    return msg


def make_camera_info(fx: float = 600.0, fy: float = 600.0, cx: float = 320.0, cy: float = 320.0):
    from sensor_msgs.msg import CameraInfo

    msg = CameraInfo()
    msg.k = [0.0] * 9
    msg.k[0] = float(fx)
    msg.k[2] = float(cx)
    msg.k[4] = float(fy)
    msg.k[5] = float(cy)
    msg.k[8] = 1.0
    return msg


@dataclass
class Case:
    name: str
    duration_s: float
    detection_hz: float
    pose_hz: float
    detection_active: Callable[[float], bool]
    centroid: Callable[[float], tuple[float, float]]
    bbox_size: Callable[[float], tuple[float, float]]
    description: str


def case_definitions() -> list[Case]:
    return [
        Case(
            name="case_a_stable_tracking",
            duration_s=4.0,
            detection_hz=10.0,
            pose_hz=20.0,
            detection_active=lambda t: True,
            centroid=lambda t: (320.0 + 120.0 * math.exp(-1.2 * t), 320.0 - 60.0 * math.exp(-1.0 * t)),
            bbox_size=lambda _t: (80.0, 100.0),
            description="Stable detections converging toward target center.",
        ),
        Case(
            name="case_b_constant_offset",
            duration_s=3.0,
            detection_hz=10.0,
            pose_hz=20.0,
            detection_active=lambda t: True,
            centroid=lambda _t: (410.0, 260.0),
            bbox_size=lambda _t: (80.0, 100.0),
            description="Fixed horizontal offset under steady detections.",
        ),
        Case(
            name="case_c_dropout",
            duration_s=4.0,
            detection_hz=10.0,
            pose_hz=20.0,
            detection_active=lambda t: not (1.2 <= t < 2.2),
            centroid=lambda _t: (400.0, 290.0),
            bbox_size=lambda _t: (80.0, 100.0),
            description="Detection dropout causing stale then unlock.",
        ),
        Case(
            name="case_d_roi_jump",
            duration_s=3.5,
            detection_hz=10.0,
            pose_hz=20.0,
            detection_active=lambda t: True,
            centroid=lambda t: (620.0, 500.0) if 1.4 <= t < 1.8 else (350.0, 300.0),
            bbox_size=lambda _t: (80.0, 100.0),
            description="Large centroid jump causing ROI failure.",
        ),
        Case(
            name="case_e_reacquire",
            duration_s=5.0,
            detection_hz=10.0,
            pose_hz=20.0,
            detection_active=lambda t: not (1.0 <= t < 1.8),
            centroid=lambda t: (610.0, 480.0) if 1.8 <= t < 2.2 else (360.0 - 25.0 * max(0.0, t - 2.2), 305.0),
            bbox_size=lambda _t: (80.0, 100.0),
            description="Dropout followed by reacquire and convergence.",
        ),
        Case(
            name="case_f_horizontal_only",
            duration_s=4.0,
            detection_hz=10.0,
            pose_hz=20.0,
            detection_active=lambda t: True,
            centroid=lambda _t: (380.0, 420.0),
            bbox_size=lambda _t: (80.0, 100.0),
            description="Stable detections with persistent vertical misalignment to stay in HORIZONTAL.",
        ),
    ]


def default_parameter_overrides() -> dict[str, object]:
    return {
        "controller_mode": "staged_pid",
        "rate_hz": 60.0,
        "timeout_ms": 500,
        "target_point_timeout_ms": 500,
        "image_width": 640.0,
        "image_height": 640.0,
        "target_u": 320.0,
        "target_v": 320.0,
        "body_frame_id": "base_link",
        "mavros_base_link_frame_id": "base_link",
        "sc2_base_link_frame_id": "sc2_base_link",
        "mavros_depth": False,
        "staged_pid_enable_yaw": True,
        "staged_pid_z_source": "target_pose",
        "desired_vertical_distance_m": 0.6,
        "max_vx": 0.4,
        "max_vy": 0.3,
        "max_vz": 0.3,
        "max_yaw_rate": 0.4,
        "slew_vx": 100.0,
        "slew_vy": 100.0,
        "slew_vz": 100.0,
        "slew_yaw_rate": 100.0,
        "verbose": False,
    }


def latest_publisher_string(node, topic_attr: str) -> str:
    topic = getattr(node, topic_attr)
    publisher = node._publishers.get(topic)
    if publisher is None or publisher.last_msg is None:
        return ""
    msg = publisher.last_msg
    return getattr(msg, "data", "")


def run_case(module, case: Case) -> list[dict[str, object]]:
    global CURRENT_PARAMETER_OVERRIDES
    CURRENT_PARAMETER_OVERRIDES = default_parameter_overrides()
    node = module.VisualServoControllerNode()
    node.control_enabled = True
    node._fake_now_ns = 0
    node._camera_info_cb(make_camera_info())

    tick_hz = 60.0
    tick_dt = 1.0 / tick_hz
    det_dt = 1.0 / case.detection_hz
    pose_dt = 1.0 / case.pose_hz
    next_det_t = 0.0
    next_pose_t = 0.0

    rows: list[dict[str, object]] = []
    tick_count = int(case.duration_s * tick_hz)
    for step in range(tick_count):
        t = step * tick_dt
        node._fake_now_ns = int(round(t * 1_000_000_000))

        if t + 1e-12 >= next_pose_t:
            node._target_pose_cb(make_pose(0.6))
            next_pose_t += pose_dt

        while t + 1e-12 >= next_det_t:
            if case.detection_active(next_det_t):
                x, y = case.centroid(next_det_t)
                sx, sy = case.bbox_size(next_det_t)
                node._detection_cb(make_detection(x, y, sx, sy))
            next_det_t += det_dt

        node._tick()
        vx, vy, vz, yaw = map(float, node._last_cmd)
        rows.append(
            {
                "tick": step,
                "time_s": round(t, 12),
                "vx": vx,
                "vy": vy,
                "vz": vz,
                "yaw": yaw,
                "is_locked": latest_publisher_string(node, "_target_locked_topic"),
                "status": latest_publisher_string(node, "_status_topic"),
                "state": node.current_state.name,
                "phase": getattr(node, "_staged_pid_phase", ""),
                "staged_pid_state": getattr(getattr(node, "_staged_pid_state", None), "value", ""),
                "obs_fresh": str(node._latest_obs is not None and ((node.get_clock().now() - node._last_obs_time).nanoseconds * 1e-9) <= node._timeout_sec),
            }
        )

    return rows


def compare_rows(old_rows: list[dict[str, object]], new_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    diffs: list[dict[str, object]] = []
    for old, new in zip(old_rows, new_rows):
        row = {"tick": old["tick"], "time_s": old["time_s"]}
        exact = True
        tol = True
        for key in ["vx", "vy", "yaw"]:
            diff = float(new[key]) - float(old[key])
            row[f"{key}_old"] = old[key]
            row[f"{key}_new"] = new[key]
            row[f"{key}_diff"] = diff
            exact &= diff == 0.0
            tol &= abs(diff) <= EPS
        for key in ["is_locked", "status", "state", "phase", "obs_fresh"]:
            row[f"{key}_old"] = old[key]
            row[f"{key}_new"] = new[key]
            exact &= old[key] == new[key]
            tol &= old[key] == new[key]
        row["exact_match"] = exact
        row["tolerance_match"] = tol
        if not exact:
            diffs.append(row)
    return diffs


def summarize_case(old_rows: list[dict[str, object]], new_rows: list[dict[str, object]], diffs: list[dict[str, object]]) -> dict[str, object]:
    def first_signal_diff(signal: str) -> Optional[dict[str, object]]:
        for row in diffs:
            if abs(float(row[f"{signal}_diff"])) > EPS:
                return {"tick": row["tick"], "time_s": row["time_s"]}
        return None

    locked_samples = 0
    locked_mismatches = 0
    first_locked_signal_diff = None
    for old, new in zip(old_rows, new_rows):
        if old["is_locked"] == "True" and new["is_locked"] == "True":
            locked_samples += 1
            if any(abs(float(new[k]) - float(old[k])) > EPS for k in ["vx", "vy", "yaw"]):
                locked_mismatches += 1
                if first_locked_signal_diff is None:
                    first_locked_signal_diff = {"tick": old["tick"], "time_s": old["time_s"]}

    horizontal_samples = 0
    horizontal_signal_mismatches = 0
    first_horizontal_signal_diff = None
    for old, new in zip(old_rows, new_rows):
        if new["staged_pid_state"] == "HORIZONTAL":
            horizontal_samples += 1
            if any(abs(float(new[k]) - float(old[k])) > EPS for k in ["vx", "vy", "yaw"]):
                horizontal_signal_mismatches += 1
                if first_horizontal_signal_diff is None:
                    first_horizontal_signal_diff = {"tick": old["tick"], "time_s": old["time_s"]}

    return {
        "exact_match_all_fields": len(diffs) == 0,
        "vx_exact_match": first_signal_diff("vx") is None,
        "vy_exact_match": first_signal_diff("vy") is None,
        "yaw_exact_match": first_signal_diff("yaw") is None,
        "first_vx_diff": first_signal_diff("vx"),
        "first_vy_diff": first_signal_diff("vy"),
        "first_yaw_diff": first_signal_diff("yaw"),
        "locked_samples": locked_samples,
        "locked_signal_mismatch_count": locked_mismatches,
        "locked_signals_exact_match": locked_mismatches == 0,
        "first_locked_signal_diff": first_locked_signal_diff,
        "horizontal_state_samples": horizontal_samples,
        "horizontal_state_signals_exact_match": horizontal_signal_mismatches == 0,
        "horizontal_state_signal_mismatch_count": horizontal_signal_mismatches,
        "first_horizontal_state_signal_diff": first_horizontal_signal_diff,
        "lock_trace_exact_match": all(old["is_locked"] == new["is_locked"] for old, new in zip(old_rows, new_rows)),
        "status_exact_match": all(old["status"] == new["status"] for old, new in zip(old_rows, new_rows)),
        "phase_exact_match": all(old["phase"] == new["phase"] for old, new in zip(old_rows, new_rows)),
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_comparison() -> dict[str, object]:
    register_fake_ros_modules()
    output_dir = OUTPUT_ROOT / subprocess.run(
        ["date", "+%Y%m%d_%H%M%S"],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    ).stdout.strip()
    output_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="staged_pid_compare_") as tmp:
        tmp_dir = Path(tmp)
        old_pkg = build_package_tree(tmp_dir / "old", "sc2_visual_servo_controller_old", old_source_loader)
        new_pkg = build_package_tree(tmp_dir / "new", "sc2_visual_servo_controller_new", current_source_loader)
        old_module = load_controller_module(old_pkg, "sc2_visual_servo_controller_old")
        new_module = load_controller_module(new_pkg, "sc2_visual_servo_controller_new")

        summary_cases = []
        for case in case_definitions():
            old_rows = run_case(old_module, case)
            new_rows = run_case(new_module, case)
            diffs = compare_rows(old_rows, new_rows)
            case_summary = summarize_case(old_rows, new_rows, diffs)

            case_dir = output_dir / case.name
            case_dir.mkdir(parents=True, exist_ok=True)
            write_csv(case_dir / "old.csv", old_rows)
            write_csv(case_dir / "new.csv", new_rows)
            write_csv(case_dir / "diff.csv", diffs)

            summary_cases.append(
                {
                    "case": case.name,
                    "description": case.description,
                    **case_summary,
                    "diff_tick_count": len(diffs),
                    "first_diff_tick": diffs[0]["tick"] if diffs else None,
                    "first_diff_time_s": diffs[0]["time_s"] if diffs else None,
                }
            )

        summary = {
            "target_commit": TARGET_COMMIT,
            "current_tree": "working_tree",
            "epsilon": EPS,
            "cases": summary_cases,
        }
        (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return {"output_dir": str(output_dir), "summary": summary}


def main() -> None:
    result = run_comparison()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
