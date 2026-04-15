#!/usr/bin/env python3
"""Phase 2 fake-ROS comparison of real BlueROV and grapple controller nodes.

This harness imports the actual `VisualServoControllerNode` implementation from
the local `bluerov_pid` branch and the current worktree. It is not a PID unit
test or independent model comparison. The focus is staged PID Phase 2 behavior:
horizontal regression, vertical gate entry/stay/exit, z stale handling,
target_point primary / DVL fallback behavior, and known design differences.
"""

from __future__ import annotations

import argparse
import importlib
import json
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import compare_visual_servo_controller_nodes_phase1 as phase1


REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "artifacts" / "20260415" / "IF-pid" / "phase2_vertical_compare"

LAUNCH_PHASE2_PARAMS: dict[str, Any] = {
    "target_topic": "/sc2/visual_servo/target_obs",
    "target_point_topic": "/sc2/visual_servo/target_point",
    "controller_mode": "staged_pid",
    "cmd_topic": "/mavros/setpoint_velocity/cmd_vel",
    "cmd_topic_vec_6": "/sc2/velocity_controller_node/outer/console_signals",
    "status_topic": "/sc2/visual_servo/status",
    "dvl_altitude_topic": "/waterlinked_dvl_driver/velocity_report",
    "mavros_depth_topic": "/mavros/global_position/rel_alt",
    "body_frame_id": "base_link",
    "rate_hz": 10.0,
    "timeout_ms": 500,
    "target_point_timeout_ms": 500,
    "max_vx": 0.20,
    "max_vy": 0.20,
    "max_vz": 0.20,
    "max_yaw_rate": 0.35,
    "image_width": 640.0,
    "image_height": 640.0,
    "target_u": 320.0,
    "target_v": 320.0,
    "desired_target_distance_m": 1.0,
    "desired_vertical_distance_m": 1.0,
    "staged_pid_z_source": "target_point_primary_dvl_fallback",
    "staged_pid_z_kp": 0.8,
    "staged_pid_z_ki": 0.0,
    "staged_pid_z_kd": 0.10,
    "staged_pid_z_integral_limit": 1.0,
    "staged_pid_enable_yaw": True,
    "staged_pid_horizontal_tolerance_px": 150.0,
    "staged_pid_horizontal_tolerance_x_px": 150.0,
    "staged_pid_horizontal_tolerance_y_px": 150.0,
    "staged_pid_vertical_tolerance_px": 150.0,
    "staged_pid_distance_tolerance_m": 0.08,
    "staged_pid_vertical_gate_window_ticks": 10,
    "staged_pid_vertical_enter_ratio": 0.8,
    "staged_pid_vertical_exit_ratio": 0.4,
    "staged_pid_vertical_enter_hold_ticks": 2,
    "staged_pid_vertical_exit_hold_ticks": 2,
    "staged_pid_hold_zero_distance_m": 0.5,
    "tracked_enter_window_obs": 10,
    "tracked_enter_min_obs": 2,
    "tracked_exit_no_obs_sec": 0.6,
    "ibvs_lambda": 0.1,
    "alpha_1": 0.5,
    "decay_rate": 0.1,
    "depth_estimation_mode": "DVL",
    "mavros_depth": False,
}


@dataclass(frozen=True)
class Step:
    t: float
    u: Optional[float]
    v: Optional[float]
    pose_z: Optional[float] = None
    dvl_altitude: Optional[float] = None
    tick: bool = True


@dataclass(frozen=True)
class Case:
    name: str
    description: str
    mode: str
    steps: tuple[Step, ...]


def aligned_steps(count: int, z: float, start: float = 0.0) -> tuple[Step, ...]:
    return tuple(Step(round(start + i * 0.10, 2), 322.0, 318.0, z) for i in range(count))


CASES: tuple[Case, ...] = (
    Case(
        name="case_a_horizontal_only_regression",
        description="Fresh target_pose is present, but the target never enters the horizontal alignment gate.",
        mode="exact",
        steps=(
            Step(0.00, 500.0, 300.0, 1.40),
            Step(0.10, 500.0, 300.0, 1.40),
            Step(0.20, 500.0, 300.0, 1.40),
            Step(0.30, 500.0, 300.0, 1.40),
            Step(0.82, None, None, None),
        ),
    ),
    Case(
        name="case_b_vertical_entry",
        description="Stable centered target enables vertical gate and enters HORIZONTAL_AND_VERTICAL.",
        mode="exact",
        steps=aligned_steps(14, 1.40),
    ),
    Case(
        name="case_c_z_stale",
        description="target_pose becomes stale after vertical is enabled; horizontal remains fresh.",
        mode="exact",
        steps=aligned_steps(11, 1.40) + (Step(1.60, 322.0, 318.0, None),),
    ),
    Case(
        name="case_d_vertical_stay",
        description="Target remains aligned with fresh target_pose for several ticks after vertical entry.",
        mode="exact",
        steps=aligned_steps(20, 1.30),
    ),
    Case(
        name="case_e_vertical_exit_to_horizontal",
        description="After vertical entry, horizontal alignment is broken but tracking continuity remains valid.",
        mode="exact",
        steps=aligned_steps(12, 1.40) + tuple(
            Step(round(1.20 + i * 0.10, 2), 500.0, 318.0, 1.40)
            for i in range(10)
        ),
    ),
    Case(
        name="case_f_target_distance_threshold_hook",
        description="z_meas crosses the hold-zero distance threshold.",
        mode="expected_difference",
        steps=aligned_steps(12, 1.40) + (Step(1.20, 322.0, 318.0, 0.40),),
    ),
    Case(
        name="case_g_target_point_stale_dvl_fallback",
        description="target_point becomes stale but fresh DVL altitude keeps vertical allowed.",
        mode="expected_difference",
        steps=aligned_steps(11, 1.40) + (
            Step(1.60, 322.0, 318.0, None, 1.35),
        ),
    ),
    Case(
        name="case_h_target_point_recovers_after_fallback",
        description="After one DVL fallback tick, fresh target_point returns and becomes primary again.",
        mode="expected_difference",
        steps=aligned_steps(11, 1.40) + (
            Step(1.60, 322.0, 318.0, None, 1.35),
            Step(1.70, 322.0, 318.0, 1.25, 1.35),
        ),
    ),
    Case(
        name="case_i_v_axis_over_150_vertical_off",
        description="After vertical entry, v alone exceeds 150 px and immediately gates off z.",
        mode="exact",
        steps=aligned_steps(12, 1.40) + (
            Step(1.20, 322.0, 471.0, 1.40),
        ),
    ),
    Case(
        name="case_j_u_axis_over_150_vertical_off",
        description="After vertical entry, u alone exceeds 150 px and immediately gates off z.",
        mode="exact",
        steps=aligned_steps(12, 1.40) + (
            Step(1.20, 471.0, 318.0, 1.40),
        ),
    ),
)


def git_text(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO_ROOT, text=True).strip()


def import_controller_modules(tmp: Path) -> tuple[Any, Any]:
    phase1.install_fake_ros_modules()
    old_module_name = phase1.write_old_package(tmp)
    sys.path.insert(0, str(tmp))
    sys.path.insert(0, str(REPO_ROOT / "src" / "sc2_visual_servo_controller"))
    old_module = importlib.import_module(old_module_name)
    new_module = importlib.import_module(
        "sc2_visual_servo_controller.visual_servo_controller_node_implementation"
    )
    return old_module, new_module


def make_target_pose(t: float, z: float) -> phase1.PoseStamped:
    phase1.set_time(t)
    msg = phase1.PoseStamped()
    msg.header.stamp = phase1.FakeTime().to_msg()
    msg.header.frame_id = "oak_rgb_camera_optical_frame"
    msg.pose.position.z = float(z)
    return msg


def make_dvl(t: float, altitude: float) -> phase1.Dvl:
    phase1.set_time(t)
    msg = phase1.Dvl()
    msg.altitude = float(altitude)
    return msg


def wrap_compute_for_logging(node: Any) -> None:
    if not hasattr(node, "_compute_staged_pid_command"):
        return
    original = node._compute_staged_pid_command

    def wrapped(*args: Any, **kwargs: Any) -> Any:
        result = original(*args, **kwargs)
        if len(result) >= 6:
            node._compare_last_allow_z = bool(result[5])
        if len(result) >= 7:
            node._compare_last_z_meas = result[6]
        if len(result) >= 8:
            node._compare_last_z_fresh = bool(result[7])
        else:
            node._compare_last_z_fresh = resolve_z_fresh(node)
        if len(result) >= 9:
            node._compare_last_z_source_raw = result[8]
        else:
            node._compare_last_z_source_raw = resolve_z_source_raw(node)
        return result

    node._compute_staged_pid_command = wrapped


def instantiate(module: Any) -> Any:
    phase1.FakeROS.parameter_overrides = dict(LAUNCH_PHASE2_PARAMS)
    phase1.set_time(0.0)
    node = module.VisualServoControllerNode()
    node.control_enabled = True
    wrap_compute_for_logging(node)
    node._camera_info_cb(phase1.make_camera_info(0.0))
    return node


def latest_msg(node: Any, topic: str) -> Any:
    msgs = node._published.get(topic, [])
    return msgs[-1] if msgs else None


def latest_status(node: Any) -> str:
    msg = latest_msg(node, LAUNCH_PHASE2_PARAMS["status_topic"])
    return "" if msg is None else str(msg.data)


def latest_cmd(node: Any) -> tuple[float, float, float, float]:
    msg = latest_msg(node, LAUNCH_PHASE2_PARAMS["cmd_topic"])
    if msg is None:
        return 0.0, 0.0, 0.0, 0.0
    return (
        float(msg.twist.linear.x),
        float(msg.twist.linear.y),
        float(msg.twist.linear.z),
        float(msg.twist.angular.z),
    )


def normalize_status(status: str) -> str:
    normalized = status
    changed = True
    while changed:
        changed = False
        for suffix in ("_TARGET_POSE", "_TARGET_POINT", "_DVL_FALLBACK", "_MAVROS_Z", "_DVL"):
            if normalized.endswith(suffix):
                normalized = normalized[: -len(suffix)]
                changed = True
    return normalized


def resolve_z_fresh(node: Any) -> bool:
    if hasattr(node, "_resolve_vertical_measurement"):
        return bool(node._resolve_vertical_measurement()[2])
    if hasattr(node, "_resolve_staged_pid_vertical_measurement"):
        result = node._resolve_staged_pid_vertical_measurement()
        return bool(result[2] if len(result) >= 3 else result[1])
    return bool(getattr(node, "_staged_pid_last_z_fresh", False))


def resolve_z_meas(node: Any) -> Optional[float]:
    value = getattr(node, "_compare_last_z_meas", None)
    if value is not None:
        return float(value)
    if hasattr(node, "_resolve_vertical_measurement"):
        z_meas = node._resolve_vertical_measurement()[0]
        return None if z_meas is None else float(z_meas)
    if hasattr(node, "_resolve_staged_pid_vertical_measurement"):
        z_meas = node._resolve_staged_pid_vertical_measurement()[0]
        return None if z_meas is None else float(z_meas)
    return getattr(node, "_staged_pid_last_z_meas", None)


def normalize_z_source(source: Any) -> str:
    if source is None:
        return "none"
    value = str(source).strip().lower()
    if value in {"target_pose", "target_point"}:
        return "primary"
    if value in {"dvl_fallback", "dvl"}:
        return "fallback"
    if value in {"mavros_depth", "mavros_z"}:
        return "fallback"
    return value or "none"


def resolve_z_source_raw(node: Any) -> str:
    value = getattr(node, "_compare_last_z_source_raw", None)
    if value is not None:
        return str(value)
    if hasattr(node, "_resolve_vertical_measurement"):
        return str(node._resolve_vertical_measurement()[1] or "none")
    if hasattr(node, "_resolve_staged_pid_vertical_measurement"):
        return str(node._resolve_staged_pid_vertical_measurement()[1] or "none")
    return str(getattr(node, "_staged_pid_last_z_source", "none"))


def snapshot(node: Any) -> dict[str, Any]:
    vx, vy, vz, yaw = latest_cmd(node)
    raw_status = latest_status(node)
    allow_z = getattr(node, "_compare_last_allow_z", None)
    if allow_z is None:
        allow_z = bool(getattr(node, "_staged_pid_last_allow_z", False))
    z_fresh = getattr(node, "_compare_last_z_fresh", None)
    if z_fresh is None:
        z_fresh = resolve_z_fresh(node)
    z_source_raw = resolve_z_source_raw(node)
    z_source = normalize_z_source(z_source_raw) if z_fresh else "none"
    return {
        "vx": vx,
        "vy": vy,
        "vz": vz,
        "yaw": yaw,
        "target_tracked": bool(getattr(node, "_target_tracked", False)),
        "status_raw": raw_status,
        "status_normalized": normalize_status(raw_status),
        "current_state": getattr(getattr(node, "current_state", None), "name", None),
        "staged_state": getattr(getattr(node, "_staged_pid_state", None), "value", None),
        "phase": getattr(node, "_staged_pid_phase", "horizontal_only"),
        "horizontal_aligned_now": bool(getattr(node, "_staged_pid_last_horizontal_aligned", False)),
        "vertical_enabled": bool(getattr(node, "_staged_pid_vertical_enabled", False)),
        "allow_z": bool(allow_z),
        "z_fresh": bool(z_fresh),
        "z_source_raw": z_source_raw,
        "z_source": z_source,
        "z_meas": resolve_z_meas(node),
        "horizontal_ok_ratio": float(getattr(node, "_staged_pid_horizontal_ok_ratio", 0.0)),
        "vertical_scale": float(getattr(node, "_staged_pid_vertical_scale", 0.0)),
    }


def reset_tick_logging(node: Any) -> None:
    node._compare_last_allow_z = None
    node._compare_last_z_meas = None
    node._compare_last_z_fresh = None
    node._compare_last_z_source_raw = None


def drive_case(case: Case, old_module: Any, new_module: Any) -> list[dict[str, Any]]:
    old = instantiate(old_module)
    new = instantiate(new_module)
    rows: list[dict[str, Any]] = []

    for idx, step in enumerate(case.steps):
        reset_tick_logging(old)
        reset_tick_logging(new)
        if step.u is not None and step.v is not None:
            detection = phase1.make_detection(step.t, step.u, step.v)
            old._detection_cb(detection)
            new._detection_cb(detection)
        if step.pose_z is not None:
            pose = make_target_pose(step.t, step.pose_z)
            old._target_pose_cb(pose)
            new._target_pose_cb(pose)
        if step.dvl_altitude is not None:
            dvl = make_dvl(step.t, step.dvl_altitude)
            if hasattr(old, "_dvl_altitude_cb"):
                old._dvl_altitude_cb(dvl)
            elif hasattr(old, "_depth_measurement_cb"):
                mavros_depth = phase1.Float64()
                mavros_depth.data = float(step.dvl_altitude)
                old._depth_measurement_cb(mavros_depth)
            if hasattr(new, "_dvl_altitude_cb"):
                new._dvl_altitude_cb(dvl)
        phase1.set_time(step.t)
        if step.tick:
            old._tick()
            new._tick()

        rows.append({
            "case": case.name,
            "tick": idx,
            "input": {
                "t": step.t,
                "u": step.u,
                "v": step.v,
                "pose_z": step.pose_z,
                "dvl_altitude": step.dvl_altitude,
            },
            "old": snapshot(old),
            "new": snapshot(new),
        })

    return rows


def is_comparison_scope(row: dict[str, Any]) -> bool:
    return row["old"]["target_tracked"] and row["new"]["target_tracked"]


def compare_value(old: Any, new: Any, tol: float = 1e-12) -> bool:
    if isinstance(old, float) or isinstance(new, float):
        if old is None or new is None:
            return old is new
        return abs(float(old) - float(new)) <= tol
    return old == new


def compare_exact_case(case: Case, rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    failures: list[dict[str, Any]] = []
    fields = (
        "vx",
        "vy",
        "vz",
        "yaw",
        "target_tracked",
        "status_normalized",
        "staged_state",
        "horizontal_aligned_now",
        "vertical_enabled",
        "allow_z",
        "z_fresh",
        "z_source",
        "z_meas",
        "current_state",
    )
    for row in rows:
        if not is_comparison_scope(row):
            continue
        for field in fields:
            if not compare_value(row["old"][field], row["new"][field]):
                failures.append({
                    "case": case.name,
                    "tick": row["tick"],
                    "field": field,
                    "old": row["old"][field],
                    "new": row["new"][field],
                    "input": row["input"],
                    "old_status_raw": row["old"]["status_raw"],
                    "new_status_raw": row["new"]["status_raw"],
                    "classification": "failure",
                })
    return failures


def transition_ticks(rows: list[dict[str, Any]], side: str, predicate: str) -> list[int]:
    if predicate == "vertical_enabled":
        return [row["tick"] for row in rows if row[side]["vertical_enabled"]]
    if predicate == "allow_z":
        return [row["tick"] for row in rows if row[side]["allow_z"]]
    if predicate == "nonzero_vz":
        return [row["tick"] for row in rows if abs(row[side]["vz"]) > 1e-12]
    if predicate == "z_stale":
        return [row["tick"] for row in rows if row[side]["target_tracked"] and not row[side]["z_fresh"]]
    if predicate == "horizontal_and_vertical":
        return [row["tick"] for row in rows if row[side]["staged_state"] == "HORIZONTAL_AND_VERTICAL"]
    if predicate == "primary_source":
        return [row["tick"] for row in rows if row[side]["z_source"] == "primary"]
    if predicate == "fallback_source":
        return [row["tick"] for row in rows if row[side]["z_source"] == "fallback"]
    return []


def compare_transition_ticks(case: Case, rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    failures: list[dict[str, Any]] = []
    for predicate in (
        "vertical_enabled",
        "allow_z",
        "nonzero_vz",
        "z_stale",
        "horizontal_and_vertical",
        "primary_source",
        "fallback_source",
    ):
        old_ticks = transition_ticks(rows, "old", predicate)
        new_ticks = transition_ticks(rows, "new", predicate)
        if old_ticks != new_ticks:
            failures.append({
                "case": case.name,
                "tick": None,
                "field": f"{predicate}_ticks",
                "old": old_ticks,
                "new": new_ticks,
                "input": None,
                "old_status_raw": None,
                "new_status_raw": None,
                "classification": "failure",
            })
    return failures


def validate_axis_over_150_gate_off(case: Case, rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if case.name not in {
        "case_i_v_axis_over_150_vertical_off",
        "case_j_u_axis_over_150_vertical_off",
    }:
        return []
    last = rows[-1]
    failures: list[dict[str, Any]] = []
    expected = {
        "horizontal_aligned_now": False,
        "allow_z": False,
        "vz": 0.0,
        "staged_state": "HORIZONTAL",
    }
    for side in ("old", "new"):
        for field, value in expected.items():
            if not compare_value(last[side][field], value):
                failures.append({
                    "case": case.name,
                    "tick": last["tick"],
                    "field": f"{side}_{field}",
                    "old": value,
                    "new": last[side][field],
                    "input": last["input"],
                    "old_status_raw": last["old"]["status_raw"],
                    "new_status_raw": last["new"]["status_raw"],
                    "classification": "failure",
                })
    return failures


def validate_expected_difference(case: Case, rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    last = rows[-1]
    if case.name == "case_g_target_point_stale_dvl_fallback":
        if (
            last["new"]["z_source"] == "fallback" and
            last["new"]["allow_z"] and
            abs(last["new"]["vz"]) > 1e-12 and
            last["old"]["z_source"] != last["new"]["z_source"]
        ):
            return [{
                "case": case.name,
                "tick": last["tick"],
                "field": "fallback_behavior",
                "old": {
                    "z_source": last["old"]["z_source"],
                    "allow_z": last["old"]["allow_z"],
                    "vz": last["old"]["vz"],
                    "status_normalized": last["old"]["status_normalized"],
                },
                "new": {
                    "z_source": last["new"]["z_source"],
                    "z_source_raw": last["new"]["z_source_raw"],
                    "allow_z": last["new"]["allow_z"],
                    "vz": last["new"]["vz"],
                    "status_normalized": last["new"]["status_normalized"],
                },
                "input": last["input"],
                "old_status_raw": last["old"]["status_raw"],
                "new_status_raw": last["new"]["status_raw"],
                "classification": "expected_difference",
            }]
    if case.name == "case_h_target_point_recovers_after_fallback":
        fallback_rows = [row for row in rows if row["new"]["z_source"] == "fallback"]
        if (
            fallback_rows and
            last["new"]["z_source"] == "primary" and
            last["new"]["allow_z"] and
            abs(last["new"]["vz"]) > 1e-12
        ):
            return [{
                "case": case.name,
                "tick": last["tick"],
                "field": "primary_recovery_after_fallback",
                "old": {
                    "fallback_ticks": [row["tick"] for row in rows if row["old"]["z_source"] == "fallback"],
                    "last_z_source": last["old"]["z_source"],
                    "last_allow_z": last["old"]["allow_z"],
                },
                "new": {
                    "fallback_ticks": [row["tick"] for row in fallback_rows],
                    "last_z_source": last["new"]["z_source"],
                    "last_z_source_raw": last["new"]["z_source_raw"],
                    "last_allow_z": last["new"]["allow_z"],
                    "last_vz": last["new"]["vz"],
                },
                "input": last["input"],
                "old_status_raw": last["old"]["status_raw"],
                "new_status_raw": last["new"]["status_raw"],
                "classification": "expected_difference",
            }]
    expected = []
    if last["old"]["staged_state"] != "HOLD_ZERO_VELOCITY":
        expected.append({
            "case": case.name,
            "tick": last["tick"],
            "field": "old_hold_zero_state",
            "old": last["old"]["staged_state"],
            "new": last["new"]["staged_state"],
            "input": last["input"],
            "old_status_raw": last["old"]["status_raw"],
            "new_status_raw": last["new"]["status_raw"],
            "classification": "failure",
        })
    if last["new"]["status_normalized"] != "TRACKING_STAGED_PID_TARGET_DISTANCE_REACHED":
        expected.append({
            "case": case.name,
            "tick": last["tick"],
            "field": "new_status_hook",
            "old": last["old"]["status_normalized"],
            "new": last["new"]["status_normalized"],
            "input": last["input"],
            "old_status_raw": last["old"]["status_raw"],
            "new_status_raw": last["new"]["status_raw"],
            "classification": "failure",
        })
    if expected:
        return expected
    return [{
        "case": case.name,
        "tick": last["tick"],
        "field": "terminal_behavior",
        "old": {
            "staged_state": last["old"]["staged_state"],
            "status_normalized": last["old"]["status_normalized"],
            "cmd": [last["old"]["vx"], last["old"]["vy"], last["old"]["vz"], last["old"]["yaw"]],
        },
        "new": {
            "staged_state": last["new"]["staged_state"],
            "status_normalized": last["new"]["status_normalized"],
            "cmd": [last["new"]["vx"], last["new"]["vy"], last["new"]["vz"], last["new"]["yaw"]],
        },
        "input": last["input"],
        "old_status_raw": last["old"]["status_raw"],
        "new_status_raw": last["new"]["status_raw"],
        "classification": "expected_difference",
    }]


def transition_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        key: {
            "old": transition_ticks(rows, "old", key),
            "new": transition_ticks(rows, "new", key),
        }
        for key in (
            "vertical_enabled",
            "allow_z",
            "nonzero_vz",
            "z_stale",
            "horizontal_and_vertical",
            "primary_source",
            "fallback_source",
        )
    }


def compare_all() -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    with tempfile.TemporaryDirectory() as tmp_s:
        old_module, new_module = import_controller_modules(Path(tmp_s))
        rows: list[dict[str, Any]] = []
        findings: list[dict[str, Any]] = []
        summaries: dict[str, Any] = {}
        for case in CASES:
            case_rows = drive_case(case, old_module, new_module)
            rows.extend(case_rows)
            summaries[case.name] = transition_summary(case_rows)
            if case.mode == "exact":
                findings.extend(compare_exact_case(case, case_rows))
                findings.extend(compare_transition_ticks(case, case_rows))
                findings.extend(validate_axis_over_150_gate_off(case, case_rows))
            else:
                findings.extend(validate_expected_difference(case, case_rows))
        return rows, findings, summaries


def case_result(case: Case, findings: list[dict[str, Any]]) -> str:
    case_findings = [f for f in findings if f["case"] == case.name]
    failures = [f for f in case_findings if f["classification"] == "failure"]
    expected = [f for f in case_findings if f["classification"] == "expected_difference"]
    if failures:
        return "FAIL"
    if expected:
        return "EXPECTED_DIFFERENCE"
    return "PASS"


def write_case_logs(rows: list[dict[str, Any]], out_dir: Path) -> None:
    case_dir = out_dir / "cases"
    case_dir.mkdir(parents=True, exist_ok=True)
    for case in CASES:
        case_rows = [row for row in rows if row["case"] == case.name]
        (case_dir / f"{case.name}.json").write_text(
            json.dumps(case_rows, indent=2),
            encoding="utf-8",
        )


def write_reports(
    rows: list[dict[str, Any]],
    findings: list[dict[str, Any]],
    summaries: dict[str, Any],
    out_dir: Path,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    write_case_logs(rows, out_dir)
    old_commit = git_text("rev-parse", "bluerov_pid")
    new_branch = git_text("rev-parse", "--abbrev-ref", "HEAD")
    new_head = git_text("rev-parse", "HEAD")
    dirty = bool(git_text("status", "--short"))
    failures = [f for f in findings if f["classification"] == "failure"]
    expected = [f for f in findings if f["classification"] == "expected_difference"]
    status = "PASS" if not failures else "FAIL"

    payload = {
        "comparison_type": "fake_ros_real_controller_node_bluerov_vs_grapple_phase2",
        "status": status,
        "old": {"branch": "bluerov_pid", "commit": old_commit},
        "new": {"branch": new_branch, "head": new_head, "worktree_dirty": dirty},
        "launch_phase2_params": LAUNCH_PHASE2_PARAMS,
        "cases": [case.__dict__ for case in CASES],
        "transition_summaries": summaries,
        "rows": rows,
        "findings": findings,
    }
    (out_dir / "phase2_real_controller_compare.json").write_text(
        json.dumps(payload, indent=2, default=lambda obj: obj.__dict__),
        encoding="utf-8",
    )

    case_lines = []
    for case in CASES:
        scoped_ticks = len([row for row in rows if row["case"] == case.name and is_comparison_scope(row)])
        result = case_result(case, findings)
        summary = summaries[case.name]
        vertical_ticks = (
            f"old {summary['vertical_enabled']['old']} / "
            f"new {summary['vertical_enabled']['new']}"
        )
        allow_ticks = (
            f"old {summary['allow_z']['old']} / "
            f"new {summary['allow_z']['new']}"
        )
        vz_ticks = (
            f"old {summary['nonzero_vz']['old']} / "
            f"new {summary['nonzero_vz']['new']}"
        )
        primary_ticks = (
            f"old {summary['primary_source']['old']} / "
            f"new {summary['primary_source']['new']}"
        )
        fallback_ticks = (
            f"old {summary['fallback_source']['old']} / "
            f"new {summary['fallback_source']['new']}"
        )
        case_lines.append(
            f"| {case.name} | {case.description} | {case.mode} | {scoped_ticks} | "
            f"{vertical_ticks} | {allow_ticks} | {vz_ticks} | "
            f"{primary_ticks} | {fallback_ticks} | {result} |"
        )

    report = [
        "# Phase 2 Real Controller Compare Result",
        "",
        f"Status: {status}",
        "",
        "This is a fake-ROS comparison of the real `VisualServoControllerNode` implementations.",
        "It imports the actual controller class from `bluerov_pid` and the current `grapple_integration` worktree.",
        "",
        "## Compared Revisions",
        "",
        f"- old: `bluerov_pid` `{old_commit}`",
        f"- new: `{new_branch}` `{new_head}`",
        f"- new worktree dirty: `{dirty}`",
        "",
        "## Source Of Truth",
        "",
        "Default experiment command:",
        "",
        "```bash",
        "ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_ibvs:=true launch_foxglove:=false",
        "```",
        "",
        "Current grapple Phase 2 default vertical source behavior is `target_point` primary plus DVL fallback.",
        "The controller uses `/sc2/visual_servo/target_point` while it is fresh, uses DVL altitude only while target_point is stale/unavailable, and returns to target_point when it recovers.",
        "The latest `bluerov_pid` reference does not implement this combined primary/fallback selector; it supports target_pose or MAVROS-depth source modes. Cases G/H therefore record the new fallback behavior as expected differences.",
        "",
        "## Compare Semantics",
        "",
        "- `status_raw` is logged exactly as published.",
        "- `status_normalized` strips transport/source suffixes such as `_TARGET_POSE` and `_MAVROS_Z`.",
        "- Cases A-E require exact numeric/controller-state/source match with normalized status.",
        "- Cases I/J additionally verify that either axis exceeding 150 px gates z off immediately.",
        "- Case F is an expected design difference: BlueROV enters `HOLD_ZERO_VELOCITY`, while current grapple Phase 2 exposes only a status hook.",
        "- Cases G/H are expected differences because current grapple adds DVL fallback and target_point recovery on top of the BlueROV target_pose-only mode.",
        "",
        "## Launch Parameter Alignment",
        "",
        "The fixed harness parameters mirror `object_detection_ibvs.launch.py` for Phase 2 staged PID. Extra `bluerov_pid` compatibility parameters are included where needed. `depth_estimation_mode=DVL` selects the fallback subscriber for current grapple staged PID; it does not override target_point primary.",
        "",
        "| parameter | value |",
        "| --- | --- |",
    ]
    for key, value in LAUNCH_PHASE2_PARAMS.items():
        report.append(f"| `{key}` | `{value}` |")

    report.extend([
        "",
        "## Case Results",
        "",
        "| case | description | validation | compared ticks | vertical_enabled ticks | allow_z ticks | nonzero_vz ticks | primary source ticks | fallback source ticks | result |",
        "| --- | --- | --- | ---: | --- | --- | --- | --- | --- | --- |",
        *case_lines,
        "",
        "## Findings",
        "",
    ])
    if not findings:
        report.append("No failures or expected differences were recorded.")
    else:
        for finding in findings:
            report.append(
                "- {classification}: case={case}, tick={tick}, field={field}, old={old}, "
                "new={new}, input={input}, old_status_raw={old_status_raw}, "
                "new_status_raw={new_status_raw}".format(**finding)
            )

    report.extend([
        "",
        "## Result Interpretation",
        "",
        "Cases A-E pass when `vx/vy/vz/yaw`, `target_tracked`, normalized status, staged state, "
        "`horizontal_aligned_now`, `vertical_enabled`, `allow_z`, `z_fresh`, normalized `z_source`, and `z_meas` match on compared ticks.",
        "Raw status suffix differences are non-semantic and are preserved in JSON.",
        "",
        "Case F documents the intentionally incomplete terminal behavior in current grapple Phase 2. "
        "BlueROV has an absorbing hold-zero state; grapple Phase 2 only reports `TRACKING_STAGED_PID_TARGET_DISTANCE_REACHED` and keeps terminal command policy out of scope.",
        "",
        "Case G documents DVL fallback: when target_point is stale and DVL is fresh, current grapple uses `z_source=fallback`, keeps `allow_z=True`, and continues vertical command.",
        "",
        "Case H documents recovery: after a DVL fallback tick, a fresh target_point switches current grapple back to `z_source=primary`.",
        "",
        "Cases I/J document the updated vertical gate tolerance: `u` and `v` are both checked, and an error greater than 150 px on either axis sets `horizontal_aligned_now=False`, `allow_z=False`, `vz=0`, and `HORIZONTAL` for that tick.",
        "",
        "## Residual Risks",
        "",
        "- Verify `target_pose.pose.position.z` sign and frame convention before hardware tests.",
        "- Define DVL fallback/safety policy separately.",
        "- Search/recovery and final hold-zero behavior remain Phase 3+ work.",
    ])
    (out_dir / "PHASE2_REAL_CONTROLLER_COMPARE_RESULT.md").write_text(
        "\n".join(report) + "\n",
        encoding="utf-8",
    )

    readme = [
        "# Phase 2 Vertical Compare Artifacts",
        "",
        "Primary report: `PHASE2_REAL_CONTROLLER_COMPARE_RESULT.md`.",
        "",
        "Files:",
        "",
        "- `PHASE2_REAL_CONTROLLER_COMPARE_RESULT.md`: human-readable result and interpretation.",
        "- `phase2_real_controller_compare.json`: full row-by-row log.",
        "- `cases/*.json`: per-case row logs.",
        "",
        "Default source behavior under the standard launch command:",
        "",
        "- primary: `/sc2/visual_servo/target_point`",
        "- fallback: DVL altitude while target_point is stale/unavailable",
        "- recovery: target_point becomes primary again as soon as it is fresh",
        "",
        "Run:",
        "",
        "```bash",
        "uv run python tools/compare_visual_servo_controller_nodes_phase2_bluerov.py",
        "```",
    ]
    (out_dir / "README.md").write_text("\n".join(readme) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default=str(OUT_DIR))
    args = parser.parse_args()
    rows, findings, summaries = compare_all()
    write_reports(rows, findings, summaries, Path(args.out_dir))
    failures = [f for f in findings if f["classification"] == "failure"]
    expected = [f for f in findings if f["classification"] == "expected_difference"]
    if failures:
        print(f"FAIL: {len(failures)} failures, {len(expected)} expected differences")
        return 1
    print(f"PASS: Phase 2 real controller comparison passed with {len(expected)} expected difference(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
