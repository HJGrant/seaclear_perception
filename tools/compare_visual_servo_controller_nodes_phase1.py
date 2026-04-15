#!/usr/bin/env python3
"""Fake-ROS comparison of the real BlueROV and grapple controller nodes.

This is the Phase 1 integration comparison. It imports the actual
`VisualServoControllerNode` implementation from:

- old: `bluerov_pid:src/sc2_visual_servo_controller/.../visual_servo_controller_node_implementation.py`
- new: the current `grapple_integration` worktree

Both nodes run under the same minimal fake ROS environment and are driven by the
same synthetic detections/ticks using the Phase 1 launch-equivalent parameters.
The comparison focuses on horizontal-only behavior. BlueROV raw status strings
can include vertical-source qualifiers from its fuller staged PID implementation;
the report records both raw status and normalized Phase 1 status.
"""

from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
from dataclasses import dataclass
from typing import Any, Callable, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_REL = (
    "src/sc2_visual_servo_controller/sc2_visual_servo_controller/"
    "visual_servo_controller_node_implementation.py"
)
PID_REL = "src/sc2_visual_servo_controller/sc2_visual_servo_controller/pid.py"
IBVS_REL = "src/sc2_visual_servo_controller/sc2_visual_servo_controller/ibvs/controller_math.py"

LAUNCH_PHASE1_PARAMS: dict[str, Any] = {
    "target_topic": "/sc2/visual_servo/target_obs",
    "controller_mode": "staged_pid",
    "cmd_topic": "/mavros/setpoint_velocity/cmd_vel",
    "cmd_topic_vec_6": "/sc2/velocity_controller_node/outer/console_signals",
    "status_topic": "/sc2/visual_servo/status",
    "dvl_altitude_topic": "/waterlinked_dvl_driver/velocity_report",
    "body_frame_id": "base_link",
    "rate_hz": 10.0,
    "timeout_ms": 500,
    "target_point_timeout_ms": 500,
    "max_vx": 0.20,
    "max_vy": 0.20,
    "max_vz": 0.0,
    "max_yaw_rate": 0.35,
    "image_width": 640.0,
    "image_height": 640.0,
    "target_u": 320.0,
    "target_v": 320.0,
    "desired_target_distance_m": 1.0,
    "staged_pid_enable_yaw": True,
    "tracked_enter_window_obs": 10,
    "tracked_enter_min_obs": 2,
    "tracked_exit_no_obs_sec": 0.6,
    "ibvs_lambda": 0.1,
    "alpha_1": 0.5,
    "decay_rate": 0.1,
    "depth_estimation_mode": "DVL",
}


class FakeROS:
    current_time_ns = 0
    parameter_overrides: dict[str, Any] = {}


def set_time(t_sec: float) -> None:
    FakeROS.current_time_ns = int(round(t_sec * 1_000_000_000))


class FakeDuration:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds


class FakeTime:
    def __init__(self, nanoseconds: Optional[int] = None) -> None:
        self.nanoseconds = FakeROS.current_time_ns if nanoseconds is None else int(nanoseconds)

    def __sub__(self, other: "FakeTime") -> FakeDuration:
        return FakeDuration(self.nanoseconds - other.nanoseconds)

    def to_msg(self) -> Any:
        sec = self.nanoseconds // 1_000_000_000
        nanosec = self.nanoseconds % 1_000_000_000
        return types.SimpleNamespace(sec=int(sec), nanosec=int(nanosec))


class FakeClock:
    def now(self) -> FakeTime:
        return FakeTime()


class FakeParameterValue:
    def __init__(self, value: Any) -> None:
        self.bool_value = bool(value)
        self.string_value = str(value)
        self.double_value = float(value) if isinstance(value, (int, float)) else 0.0
        self.integer_value = int(value) if isinstance(value, int) else 0


class FakeParameter:
    def __init__(self, value: Any) -> None:
        self.value = value

    def get_parameter_value(self) -> FakeParameterValue:
        return FakeParameterValue(self.value)


class FakeLogger:
    def info(self, *args: Any, **kwargs: Any) -> None:
        return None

    def warn(self, *args: Any, **kwargs: Any) -> None:
        return None

    def error(self, *args: Any, **kwargs: Any) -> None:
        return None


class FakePublisher:
    def __init__(self, node: "FakeNode", topic: str) -> None:
        self.node = node
        self.topic = topic

    def publish(self, msg: Any) -> None:
        self.node._published.setdefault(self.topic, []).append(msg)


class FakeNode:
    def __init__(self, name: str, *args: Any, **kwargs: Any) -> None:
        self.name = name
        self._parameters: dict[str, FakeParameter] = {}
        self._published: dict[str, list[Any]] = {}
        self._subscriptions: list[tuple[str, Callable[..., Any]]] = []

    def declare_parameter(self, name: str, default_value: Any = None, *args: Any, **kwargs: Any) -> FakeParameter:
        value = FakeROS.parameter_overrides.get(name, default_value)
        param = FakeParameter(value)
        self._parameters[name] = param
        return param

    def get_parameter(self, name: str) -> FakeParameter:
        return self._parameters[name]

    def create_publisher(self, msg_type: Any, topic: str, qos: int) -> FakePublisher:
        self._published.setdefault(topic, [])
        return FakePublisher(self, topic)

    def create_subscription(self, msg_type: Any, topic: str, callback: Callable[..., Any], qos: int) -> Any:
        self._subscriptions.append((topic, callback))
        return types.SimpleNamespace(topic=topic, callback=callback)

    def create_service(self, srv_type: Any, name: str, callback: Callable[..., Any]) -> Any:
        return types.SimpleNamespace(name=name, callback=callback)

    def create_timer(self, period_sec: float, callback: Callable[..., Any]) -> Any:
        return types.SimpleNamespace(period_sec=period_sec, callback=callback)

    def get_clock(self) -> FakeClock:
        return FakeClock()

    def get_logger(self) -> FakeLogger:
        return FakeLogger()

    def destroy_node(self) -> None:
        return None


class Header:
    def __init__(self) -> None:
        self.stamp = FakeTime().to_msg()
        self.frame_id = ""


class Vector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Twist:
    def __init__(self) -> None:
        self.linear = Vector3()
        self.angular = Vector3()


class TwistStamped:
    def __init__(self) -> None:
        self.header = Header()
        self.twist = Twist()


class Point:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Quaternion:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class Pose:
    def __init__(self) -> None:
        self.position = Point()
        self.orientation = Quaternion()


class PoseStamped:
    def __init__(self) -> None:
        self.header = Header()
        self.pose = Pose()


class PointStamped:
    def __init__(self) -> None:
        self.header = Header()
        self.point = Point()


class Vector3Stamped:
    def __init__(self) -> None:
        self.header = Header()
        self.vector = Vector3()


class String:
    def __init__(self) -> None:
        self.data = ""


class Float64:
    def __init__(self) -> None:
        self.data = 0.0


class CameraInfo:
    def __init__(self) -> None:
        self.header = Header()
        self.k = [600.0, 0.0, 320.0, 0.0, 600.0, 320.0, 0.0, 0.0, 1.0]


class BBoxCenter:
    def __init__(self) -> None:
        self.position = Point()


class BoundingBox2D:
    def __init__(self) -> None:
        self.center = BBoxCenter()
        self.size_x = 0.0
        self.size_y = 0.0


class Detection2D:
    def __init__(self) -> None:
        self.header = Header()
        self.bbox = BoundingBox2D()


class Vector6:
    def __init__(self) -> None:
        self.linear = Vector3()
        self.angular = Vector3()


class Dvl:
    def __init__(self) -> None:
        self.altitude = 0.0


def install_fake_ros_modules() -> None:
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.TwistStamped = TwistStamped
    geometry_msgs_msg.PoseStamped = PoseStamped
    geometry_msgs_msg.PointStamped = PointStamped
    geometry_msgs_msg.Vector3 = Vector3
    geometry_msgs_msg.Vector3Stamped = Vector3Stamped
    geometry_msgs.msg = geometry_msgs_msg

    vision_msgs = types.ModuleType("vision_msgs")
    vision_msgs_msg = types.ModuleType("vision_msgs.msg")
    vision_msgs_msg.Detection2D = Detection2D
    vision_msgs.msg = vision_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CameraInfo = CameraInfo
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = String
    std_msgs_msg.Float64 = Float64
    std_msgs.msg = std_msgs_msg

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.SetBool = type("SetBool", (), {})
    std_srvs.srv = std_srvs_srv

    state_estimation = types.ModuleType("state_estimation")
    state_estimation_msg = types.ModuleType("state_estimation.msg")
    state_estimation_msg.Vector6 = Vector6
    state_estimation.msg = state_estimation_msg

    marine_acoustic_msgs = types.ModuleType("marine_acoustic_msgs")
    marine_acoustic_msgs_msg = types.ModuleType("marine_acoustic_msgs.msg")
    marine_acoustic_msgs_msg.Dvl = Dvl
    marine_acoustic_msgs.msg = marine_acoustic_msgs_msg

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.try_shutdown = lambda *args, **kwargs: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_time = types.ModuleType("rclpy.time")
    rclpy_time.Time = FakeTime
    rclpy.node = rclpy_node
    rclpy.time = rclpy_time

    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.TransformException = type("TransformException", (Exception,), {})
    tf2_ros.Buffer = lambda *args, **kwargs: types.SimpleNamespace(
        lookup_transform=lambda *a, **kw: types.SimpleNamespace()
    )
    tf2_ros.TransformListener = lambda *args, **kwargs: types.SimpleNamespace()

    tf2_geometry_msgs = types.ModuleType("tf2_geometry_msgs")
    tf2_geometry_msgs.do_transform_vector3 = lambda msg, transform: msg
    tf2_geometry_msgs.do_transform_point = lambda msg, transform: msg

    sys.modules.update({
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "vision_msgs": vision_msgs,
        "vision_msgs.msg": vision_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
        "state_estimation": state_estimation,
        "state_estimation.msg": state_estimation_msg,
        "marine_acoustic_msgs": marine_acoustic_msgs,
        "marine_acoustic_msgs.msg": marine_acoustic_msgs_msg,
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.time": rclpy_time,
        "tf2_ros": tf2_ros,
        "tf2_geometry_msgs": tf2_geometry_msgs,
    })


def git_show(branch: str, rel_path: str) -> str:
    return subprocess.check_output(
        ["git", "show", f"{branch}:{rel_path}"],
        cwd=REPO_ROOT,
        text=True,
    )


def write_old_package(tmp: Path) -> str:
    package = tmp / "old_sc2_visual_servo_controller"
    (package / "ibvs").mkdir(parents=True)
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "ibvs" / "__init__.py").write_text("", encoding="utf-8")
    (package / "visual_servo_controller_node_implementation.py").write_text(
        git_show("bluerov_pid", CONTROLLER_REL),
        encoding="utf-8",
    )
    (package / "pid.py").write_text(git_show("bluerov_pid", PID_REL), encoding="utf-8")
    (package / "ibvs" / "controller_math.py").write_text(
        git_show("bluerov_pid", IBVS_REL),
        encoding="utf-8",
    )
    return "old_sc2_visual_servo_controller.visual_servo_controller_node_implementation"


def import_controller_modules(tmp: Path) -> tuple[Any, Any]:
    install_fake_ros_modules()
    old_module_name = write_old_package(tmp)
    sys.path.insert(0, str(tmp))
    sys.path.insert(0, str(REPO_ROOT / "src" / "sc2_visual_servo_controller"))
    old_module = importlib.import_module(old_module_name)
    new_module = importlib.import_module(
        "sc2_visual_servo_controller.visual_servo_controller_node_implementation"
    )
    return old_module, new_module


def make_detection(t: float, u: float, v: float, w: float = 80.0, h: float = 60.0) -> Detection2D:
    set_time(t)
    msg = Detection2D()
    msg.header.stamp = FakeTime().to_msg()
    msg.header.frame_id = "oak_rgb_camera_optical_frame"
    msg.bbox.center.position.x = float(u)
    msg.bbox.center.position.y = float(v)
    msg.bbox.size_x = float(w)
    msg.bbox.size_y = float(h)
    return msg


def make_camera_info(t: float) -> CameraInfo:
    set_time(t)
    msg = CameraInfo()
    msg.header.stamp = FakeTime().to_msg()
    msg.header.frame_id = "oak_rgb_camera_optical_frame"
    return msg


@dataclass(frozen=True)
class Step:
    t: float
    u: Optional[float]
    v: Optional[float]
    tick: bool = True


@dataclass(frozen=True)
class Case:
    name: str
    description: str
    steps: tuple[Step, ...]


CASES: tuple[Case, ...] = (
    Case(
        name="tracking_entry",
        description="Two ROI-consistent observations enter target_tracked.",
        steps=(
            Step(0.00, 380.0, 300.0),
            Step(0.10, 370.0, 304.0),
            Step(0.20, 360.0, 308.0),
        ),
    ),
    Case(
        name="stable_converge",
        description="Target moves toward center but remains horizontal-only.",
        steps=(
            Step(0.00, 390.0, 290.0),
            Step(0.10, 380.0, 296.0),
            Step(0.20, 370.0, 302.0),
            Step(0.30, 360.0, 308.0),
            Step(0.40, 350.0, 314.0),
        ),
    ),
    Case(
        name="steady_offset",
        description="Constant offset keeps horizontal PID active.",
        steps=(
            Step(0.00, 370.0, 300.0),
            Step(0.10, 370.0, 300.0),
            Step(0.20, 370.0, 300.0),
            Step(0.30, 370.0, 300.0),
        ),
    ),
    Case(
        name="short_stale",
        description="Detection drops after tracking and stale decay is used.",
        steps=(
            Step(0.00, 370.0, 300.0),
            Step(0.10, 365.0, 302.0),
            Step(0.20, 360.0, 304.0),
            Step(0.72, None, None),
        ),
    ),
    Case(
        name="long_horizontal_only",
        description="Longer run avoids vertical-ready alignment.",
        steps=(
            Step(0.00, 380.0, 294.0),
            Step(0.10, 374.0, 296.0),
            Step(0.20, 368.0, 298.0),
            Step(0.30, 362.0, 300.0),
            Step(0.40, 356.0, 302.0),
            Step(0.50, 350.0, 304.0),
            Step(0.60, 346.0, 306.0),
        ),
    ),
)


def instantiate(module: Any) -> Any:
    FakeROS.parameter_overrides = dict(LAUNCH_PHASE1_PARAMS)
    set_time(0.0)
    node = module.VisualServoControllerNode()
    node.control_enabled = True
    node._camera_info_cb(make_camera_info(0.0))
    return node


def latest_msg(node: Any, topic: str) -> Any:
    msgs = node._published.get(topic, [])
    return msgs[-1] if msgs else None


def latest_status(node: Any) -> str:
    msg = latest_msg(node, LAUNCH_PHASE1_PARAMS["status_topic"])
    return "" if msg is None else str(msg.data)


def latest_cmd(node: Any) -> tuple[float, float, float]:
    msg = latest_msg(node, LAUNCH_PHASE1_PARAMS["cmd_topic"])
    if msg is None:
        return 0.0, 0.0, 0.0
    return (
        float(msg.twist.linear.x),
        float(msg.twist.linear.y),
        float(msg.twist.angular.z),
    )


def latest_vec6(node: Any) -> Optional[tuple[float, float, float]]:
    msg = latest_msg(node, LAUNCH_PHASE1_PARAMS["cmd_topic_vec_6"])
    if msg is None:
        return None
    return float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)


def normalize_phase1_status(status: str) -> str:
    normalized = status
    for suffix in ("_TARGET_POSE", "_MAVROS_Z"):
        if normalized.endswith(suffix):
            normalized = normalized[: -len(suffix)]
    if normalized in {
        "TRACKING_STAGED_PID_HORIZONTAL_Z_STALE",
        "TRACKING_STAGED_PID_HORIZONTAL_READY",
    }:
        return "TRACKING_STAGED_PID_HORIZONTAL"
    if normalized.startswith("TRACKING_STAGED_PID_SEARCH_"):
        return "TRACKING_STAGED_PID_NO_TARGET"
    return normalized


def snapshot(node: Any) -> dict[str, Any]:
    vx, vy, yaw = latest_cmd(node)
    raw_status = latest_status(node)
    return {
        "vx": vx,
        "vy": vy,
        "yaw": yaw,
        "target_tracked": bool(getattr(node, "_target_tracked", False)),
        "status_raw": raw_status,
        "status_phase1": normalize_phase1_status(raw_status),
        "current_state": getattr(getattr(node, "current_state", None), "name", None),
        "staged_state": getattr(getattr(node, "_staged_pid_state", None), "value", None),
        "phase": getattr(node, "_staged_pid_phase", "horizontal_only"),
        "vec6": latest_vec6(node),
    }


def horizontal_scope(row: dict[str, Any]) -> bool:
    old = row["old"]
    new = row["new"]
    status = old["status_phase1"]
    return (
        old["target_tracked"]
        and new["target_tracked"]
        and status in {
            "TRACKING_STAGED_PID_HORIZONTAL",
            "TRACKING_STAGED_PID_HORIZONTAL_STALE",
        }
    )


def compare_case(case: Case, old_module: Any, new_module: Any) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    old = instantiate(old_module)
    new = instantiate(new_module)
    rows: list[dict[str, Any]] = []
    mismatches: list[dict[str, Any]] = []

    for idx, step in enumerate(case.steps):
        if step.u is not None and step.v is not None:
            det = make_detection(step.t, step.u, step.v)
            old._detection_cb(det)
            new._detection_cb(det)
        set_time(step.t)
        if step.tick:
            old._tick()
            new._tick()

        row = {
            "case": case.name,
            "tick": idx,
            "input": {"t": step.t, "u": step.u, "v": step.v},
            "old": snapshot(old),
            "new": snapshot(new),
        }
        row["horizontal_scope"] = horizontal_scope(row)
        rows.append(row)

        if not row["horizontal_scope"]:
            continue

        for field in ("vx", "vy", "yaw"):
            if abs(row["old"][field] - row["new"][field]) > 1e-12:
                mismatches.append({
                    "case": case.name,
                    "tick": idx,
                    "field": field,
                    "old": row["old"][field],
                    "new": row["new"][field],
                    "input": row["input"],
                    "old_branch": row["old"]["status_raw"],
                    "new_branch": row["new"]["status_raw"],
                })
        for field in ("target_tracked", "status_phase1"):
            if row["old"][field] != row["new"][field]:
                mismatches.append({
                    "case": case.name,
                    "tick": idx,
                    "field": field,
                    "old": row["old"][field],
                    "new": row["new"][field],
                    "input": row["input"],
                    "old_branch": row["old"]["status_raw"],
                    "new_branch": row["new"]["status_raw"],
                })

    return rows, mismatches


def compare_all() -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    with tempfile.TemporaryDirectory() as tmp_s:
        tmp = Path(tmp_s)
        old_module, new_module = import_controller_modules(tmp)
        all_rows: list[dict[str, Any]] = []
        all_mismatches: list[dict[str, Any]] = []
        for case in CASES:
            rows, mismatches = compare_case(case, old_module, new_module)
            all_rows.extend(rows)
            all_mismatches.extend(mismatches)
        return all_rows, all_mismatches


def write_reports(rows: list[dict[str, Any]], mismatches: list[dict[str, Any]], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "phase1_controller_node_compare.json").write_text(
        json.dumps(
            {
                "comparison_type": "fake_ros_real_controller_node",
                "launch_phase1_params": LAUNCH_PHASE1_PARAMS,
                "cases": [case.__dict__ for case in CASES],
                "rows": rows,
                "mismatches": mismatches,
            },
            indent=2,
            default=lambda obj: obj.__dict__,
        ),
        encoding="utf-8",
    )

    status = "PASS" if not mismatches else "FAIL"
    case_lines = []
    for case in CASES:
        scoped = [r for r in rows if r["case"] == case.name and r["horizontal_scope"]]
        case_mismatches = [m for m in mismatches if m["case"] == case.name]
        case_lines.append(
            f"| {case.name} | {case.description} | {len(scoped)} | "
            f"{'PASS' if not case_mismatches else 'FAIL'} |"
        )

    md = [
        "# Phase 1 Real Controller Compare Result",
        "",
        f"Status: {status}",
        "",
        "Comparison type: fake ROS import/drive of the real `VisualServoControllerNode` implementations.",
        "",
        "Old implementation: `bluerov_pid` branch.",
        "New implementation: current `grapple_integration` worktree.",
        "",
        "The launch-equivalent parameters used by the harness are:",
        "",
        "| parameter | value |",
        "| --- | --- |",
    ]
    for key, value in LAUNCH_PHASE1_PARAMS.items():
        md.append(f"| `{key}` | `{value}` |")

    md.extend([
        "",
        "Case summary:",
        "",
        "| case | description | horizontal-scope ticks | result |",
        "| --- | --- | ---: | --- |",
        *case_lines,
        "",
        "Rows are compared only when both implementations are in the Phase 1 horizontal-only scope. "
        "Pre-tracking/search rows are recorded in JSON but not counted as Phase 1 horizontal mismatches.",
        "",
        "BlueROV raw statuses may include vertical-source qualifiers such as `_TARGET_POSE` or "
        "`_Z_STALE`; these are recorded as `status_raw` and normalized to `status_phase1` for "
        "the horizontal-only comparison.",
    ])

    if mismatches:
        md.extend(["", "## Mismatches", ""])
        for mismatch in mismatches:
            md.append(
                "- case={case}, tick={tick}, field={field}, old={old}, new={new}, "
                "input={input}, old_branch={old_branch}, new_branch={new_branch}".format(**mismatch)
            )
    else:
        md.extend([
            "",
            "## Result",
            "",
            "All horizontal-scope ticks match for `vx`, `vy`, `yaw`, `target_tracked`, and "
            "Phase 1 normalized status.",
        ])

    (out_dir / "PHASE1_REAL_CONTROLLER_COMPARE_RESULT.md").write_text(
        "\n".join(md) + "\n",
        encoding="utf-8",
    )

    summary = [
        "# Phase 1 Horizontal Compare Result",
        "",
        f"Status: {status}",
        "",
        "This is the Phase 1 summary report. The acceptance comparison is the fake-ROS real controller comparison.",
        "",
        "## Compare Roles",
        "",
        "| script | role | output |",
        "| --- | --- | --- |",
        "| `tools/compare_staged_pid_horizontal_behavior.py` | Independent horizontal model / PID math check | `PHASE1_INDEPENDENT_HORIZONTAL_COMPARE_RESULT.md` |",
        "| `tools/compare_visual_servo_controller_nodes_phase1.py` | Real `VisualServoControllerNode` old/new comparison under fake ROS | `PHASE1_REAL_CONTROLLER_COMPARE_RESULT.md` |",
        "",
        "## Launch Alignment",
        "",
        "The real controller harness uses the Phase 1 launch-equivalent parameters below.",
        "",
        "| parameter | value |",
        "| --- | --- |",
    ]
    for key, value in LAUNCH_PHASE1_PARAMS.items():
        summary.append(f"| `{key}` | `{value}` |")

    summary.extend([
        "",
        "## Case Results",
        "",
        "| case | horizontal-scope ticks | result |",
        "| --- | ---: | --- |",
        *[
            f"| {case.name} | {len([r for r in rows if r['case'] == case.name and r['horizontal_scope']])} | "
            f"{'PASS' if not [m for m in mismatches if m['case'] == case.name] else 'FAIL'} |"
            for case in CASES
        ],
        "",
        "## Result",
        "",
    ])
    if mismatches:
        summary.append("Real controller comparison found horizontal-scope mismatches. See `PHASE1_REAL_CONTROLLER_COMPARE_RESULT.md`.")
    else:
        summary.append(
            "Real controller comparison passed: `vx`, `vy`, `yaw`, `target_tracked`, "
            "and Phase 1 normalized status match on all horizontal-scope ticks."
        )
    summary.extend([
        "",
        "BlueROV raw status includes vertical/source qualifiers in some rows because the source branch already contains Phase 2 logic. "
        "The harness records raw status and compares the normalized Phase 1 horizontal status.",
        "",
        "Phase 1 can proceed to review/commit from the comparison standpoint. Phase 2 should still decide the grapple vertical source and search/hold behavior before adding those paths.",
    ])

    (out_dir / "PHASE1_HORIZONTAL_COMPARE_RESULT.md").write_text(
        "\n".join(summary) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out-dir",
        default=str(REPO_ROOT / "artifacts" / "20260415" / "IF-pid"),
    )
    args = parser.parse_args()

    rows, mismatches = compare_all()
    write_reports(rows, mismatches, Path(args.out_dir))
    if mismatches:
        print(f"FAIL: {len(mismatches)} horizontal-scope mismatches")
        return 1
    print("PASS: real controller horizontal-scope outputs match")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
