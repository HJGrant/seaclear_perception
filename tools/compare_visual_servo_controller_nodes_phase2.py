#!/usr/bin/env python3
"""Fake-ROS comparison for Phase 2 staged PID vertical integration.

This harness compares:

- baseline: the saved Phase 1 `VisualServoControllerNode` implementation in
  `tools/baselines/phase1_sc2_visual_servo_controller`
- new: the current worktree Phase 2 `VisualServoControllerNode`

Case A is an exact Phase 1 horizontal-only regression check. Cases B/C validate
the new target_pose-sourced vertical behavior without treating the Phase 1
baseline as a vertical reference.
"""

from __future__ import annotations

import argparse
import importlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import compare_visual_servo_controller_nodes_phase1 as phase1


REPO_ROOT = Path(__file__).resolve().parents[1]

LAUNCH_PHASE2_PARAMS: dict[str, Any] = {
    "target_topic": "/sc2/visual_servo/target_obs",
    "target_point_topic": "/sc2/visual_servo/target_point",
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
    "max_vz": 0.20,
    "max_yaw_rate": 0.35,
    "image_width": 640.0,
    "image_height": 640.0,
    "target_u": 320.0,
    "target_v": 320.0,
    "desired_target_distance_m": 1.0,
    "desired_vertical_distance_m": 1.0,
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
}


@dataclass(frozen=True)
class Step:
    t: float
    u: Optional[float]
    v: Optional[float]
    pose_z: Optional[float] = None
    tick: bool = True


@dataclass(frozen=True)
class Case:
    name: str
    description: str
    mode: str
    steps: tuple[Step, ...]


CASES: tuple[Case, ...] = (
    Case(
        name="case_a_horizontal_only_exact",
        description="Never enters the horizontal alignment gate; includes a short detection stale tick.",
        mode="horizontal_exact",
        steps=(
            Step(0.00, 370.0, 300.0),
            Step(0.10, 500.0, 300.0),
            Step(0.20, 500.0, 300.0),
            Step(0.30, 500.0, 300.0),
            Step(0.82, None, None),
        ),
    ),
    Case(
        name="case_b_vertical_entry",
        description="Aligned observations plus fresh target_pose enable z control.",
        mode="vertical_entry",
        steps=tuple(
            Step(round(i * 0.10, 2), 322.0, 318.0, 1.40)
            for i in range(14)
        ),
    ),
    Case(
        name="case_c_z_stale",
        description="Vertical gate is enabled, then target_pose becomes stale while detections stay fresh.",
        mode="z_stale",
        steps=tuple(
            [Step(round(i * 0.10, 2), 322.0, 318.0, 1.40) for i in range(11)] +
            [Step(1.60, 322.0, 318.0, None)]
        ),
    ),
    Case(
        name="case_d_v_axis_over_150_vertical_off",
        description="After vertical entry, v alone exceeds 150 px and immediately forces allow_z false.",
        mode="axis_gate_off",
        steps=tuple(
            [Step(round(i * 0.10, 2), 322.0, 318.0, 1.40) for i in range(12)] +
            [Step(1.20, 322.0, 471.0, 1.40)]
        ),
    ),
    Case(
        name="case_e_u_axis_over_150_vertical_off",
        description="After vertical entry, u alone exceeds 150 px and immediately forces allow_z false.",
        mode="axis_gate_off",
        steps=tuple(
            [Step(round(i * 0.10, 2), 322.0, 318.0, 1.40) for i in range(12)] +
            [Step(1.20, 471.0, 318.0, 1.40)]
        ),
    ),
)


def import_controller_modules() -> tuple[Any, Any]:
    phase1.install_fake_ros_modules()
    baseline_root = REPO_ROOT / "tools" / "baselines"
    src_root = REPO_ROOT / "src" / "sc2_visual_servo_controller"
    for path in (str(baseline_root), str(src_root)):
        if path not in phase1.sys.path:
            phase1.sys.path.insert(0, path)

    baseline_module = importlib.import_module(
        "phase1_sc2_visual_servo_controller.visual_servo_controller_node_implementation"
    )
    new_module = importlib.import_module(
        "sc2_visual_servo_controller.visual_servo_controller_node_implementation"
    )
    return baseline_module, new_module


def make_target_pose(t: float, z: float) -> phase1.PoseStamped:
    phase1.set_time(t)
    msg = phase1.PoseStamped()
    msg.header.stamp = phase1.FakeTime().to_msg()
    msg.header.frame_id = "oak_rgb_camera_optical_frame"
    msg.pose.position.z = float(z)
    return msg


def instantiate(module: Any) -> Any:
    phase1.FakeROS.parameter_overrides = dict(LAUNCH_PHASE2_PARAMS)
    phase1.set_time(0.0)
    node = module.VisualServoControllerNode()
    node.control_enabled = True
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


def latest_vec6(node: Any) -> Optional[tuple[float, float, float, float]]:
    msg = latest_msg(node, LAUNCH_PHASE2_PARAMS["cmd_topic_vec_6"])
    if msg is None:
        return None
    return (
        float(msg.linear.x),
        float(msg.linear.y),
        float(msg.linear.z),
        float(msg.angular.z),
    )


def snapshot(node: Any) -> dict[str, Any]:
    vx, vy, vz, yaw = latest_cmd(node)
    return {
        "vx": vx,
        "vy": vy,
        "vz": vz,
        "yaw": yaw,
        "target_tracked": bool(getattr(node, "_target_tracked", False)),
        "status": latest_status(node),
        "current_state": getattr(getattr(node, "current_state", None), "name", None),
        "staged_state": getattr(getattr(node, "_staged_pid_state", None), "value", None),
        "phase": getattr(node, "_staged_pid_phase", "horizontal_only"),
        "horizontal_aligned_now": bool(getattr(node, "_staged_pid_last_horizontal_aligned", False)),
        "vertical_enabled": bool(getattr(node, "_staged_pid_vertical_enabled", False)),
        "allow_z": bool(getattr(node, "_staged_pid_last_allow_z", False)),
        "z_fresh": bool(getattr(node, "_staged_pid_last_z_fresh", False)),
        "z_meas": getattr(node, "_staged_pid_last_z_meas", None),
        "vec6": latest_vec6(node),
    }


def drive_case(case: Case, baseline_module: Any, new_module: Any) -> list[dict[str, Any]]:
    baseline = instantiate(baseline_module)
    new = instantiate(new_module)
    rows: list[dict[str, Any]] = []

    for idx, step in enumerate(case.steps):
        if step.u is not None and step.v is not None:
            det = phase1.make_detection(step.t, step.u, step.v)
            baseline._detection_cb(det)
            new._detection_cb(det)
        if step.pose_z is not None:
            new._target_pose_cb(make_target_pose(step.t, step.pose_z))
        phase1.set_time(step.t)
        if step.tick:
            baseline._tick()
            new._tick()

        rows.append({
            "case": case.name,
            "tick": idx,
            "input": {"t": step.t, "u": step.u, "v": step.v, "pose_z": step.pose_z},
            "baseline": snapshot(baseline),
            "new": snapshot(new),
        })

    return rows


def horizontal_scope(row: dict[str, Any]) -> bool:
    return (
        row["baseline"]["target_tracked"] and
        row["new"]["target_tracked"] and
        row["baseline"]["status"] in {
            "TRACKING_STAGED_PID_HORIZONTAL",
            "TRACKING_STAGED_PID_HORIZONTAL_STALE",
        }
    )


def validate_horizontal_exact(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    mismatches: list[dict[str, Any]] = []
    for row in rows:
        if not horizontal_scope(row):
            continue
        for field in ("vx", "vy", "vz", "yaw", "target_tracked", "status"):
            if row["baseline"][field] != row["new"][field]:
                mismatches.append({
                    "case": row["case"],
                    "tick": row["tick"],
                    "field": field,
                    "baseline": row["baseline"][field],
                    "new": row["new"][field],
                    "input": row["input"],
                    "new_branch": row["new"]["status"],
                })
    return mismatches


def validate_vertical_entry(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    passing_rows = [
        row for row in rows
        if row["new"]["allow_z"] and
        row["new"]["staged_state"] == "HORIZONTAL_AND_VERTICAL" and
        abs(row["new"]["vz"]) > 1e-12
    ]
    if passing_rows:
        return []
    return [{
        "case": rows[-1]["case"],
        "tick": rows[-1]["tick"],
        "field": "allow_z/vz",
        "baseline": None,
        "new": rows[-1]["new"],
        "input": rows[-1]["input"],
        "new_branch": rows[-1]["new"]["status"],
    }]


def validate_z_stale(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    last = rows[-1]
    errors: list[dict[str, Any]] = []
    expected = {
        "target_tracked": True,
        "vertical_enabled": True,
        "allow_z": False,
        "z_fresh": False,
        "vz": 0.0,
    }
    for field, value in expected.items():
        actual = last["new"][field]
        if actual != value:
            errors.append({
                "case": last["case"],
                "tick": last["tick"],
                "field": field,
                "baseline": None,
                "new": actual,
                "input": last["input"],
                "new_branch": last["new"]["status"],
            })
    for field in ("vx", "vy", "yaw"):
        if last["baseline"][field] != last["new"][field]:
            errors.append({
                "case": last["case"],
                "tick": last["tick"],
                "field": f"horizontal_{field}",
                "baseline": last["baseline"][field],
                "new": last["new"][field],
                "input": last["input"],
                "new_branch": last["new"]["status"],
            })
    return errors


def validate_axis_gate_off(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    last = rows[-1]
    errors: list[dict[str, Any]] = []
    expected = {
        "target_tracked": True,
        "horizontal_aligned_now": False,
        "allow_z": False,
        "vz": 0.0,
        "staged_state": "HORIZONTAL",
    }
    for field, value in expected.items():
        actual = last["new"][field]
        if actual != value:
            errors.append({
                "case": last["case"],
                "tick": last["tick"],
                "field": field,
                "baseline": None,
                "new": actual,
                "input": last["input"],
                "new_branch": last["new"]["status"],
            })
    return errors


def compare_all() -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    baseline_module, new_module = import_controller_modules()
    rows: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []

    for case in CASES:
        case_rows = drive_case(case, baseline_module, new_module)
        rows.extend(case_rows)
        if case.mode == "horizontal_exact":
            failures.extend(validate_horizontal_exact(case_rows))
        elif case.mode == "vertical_entry":
            failures.extend(validate_vertical_entry(case_rows))
        elif case.mode == "z_stale":
            failures.extend(validate_z_stale(case_rows))
        elif case.mode == "axis_gate_off":
            failures.extend(validate_axis_gate_off(case_rows))

    return rows, failures


def write_reports(rows: list[dict[str, Any]], failures: list[dict[str, Any]], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "phase2_controller_node_compare.json"
    json_path.write_text(
        json.dumps(
            {
                "comparison_type": "fake_ros_phase1_baseline_vs_phase2_controller_node",
                "launch_phase2_params": LAUNCH_PHASE2_PARAMS,
                "cases": [case.__dict__ for case in CASES],
                "rows": rows,
                "failures": failures,
            },
            indent=2,
            default=lambda obj: obj.__dict__,
        ),
        encoding="utf-8",
    )

    status = "PASS" if not failures else "FAIL"
    case_lines = []
    for case in CASES:
        scoped = [r for r in rows if r["case"] == case.name and horizontal_scope(r)]
        case_failures = [f for f in failures if f["case"] == case.name]
        case_lines.append(
            f"| {case.name} | {case.description} | {case.mode} | {len(scoped)} | "
            f"{'PASS' if not case_failures else 'FAIL'} |"
        )

    md = [
        "# Phase 2 Vertical Compare Result",
        "",
        f"Status: {status}",
        "",
        "Comparison type: fake ROS import/drive of real `VisualServoControllerNode` implementations.",
        "",
        "Baseline implementation: saved Phase 1 snapshot in `tools/baselines/phase1_sc2_visual_servo_controller`.",
        "New implementation: current Phase 2 worktree.",
        "",
        "Phase 2 vertical source behavior: `target_point_topic` / `PoseStamped.pose.position.z` is primary.",
        "DVL altitude is used only as fallback while target_point is stale or unavailable.",
        "",
        "## Launch Alignment",
        "",
        "The harness uses the Phase 2 launch-equivalent parameters below.",
        "",
        "| parameter | value |",
        "| --- | --- |",
    ]
    for key, value in LAUNCH_PHASE2_PARAMS.items():
        md.append(f"| `{key}` | `{value}` |")

    md.extend([
        "",
        "## Case Results",
        "",
        "| case | description | validation | horizontal-scope ticks | result |",
        "| --- | --- | --- | ---: | --- |",
        *case_lines,
        "",
        "## Result",
        "",
    ])
    if failures:
        md.append("Failures were found:")
        for failure in failures:
            md.append(
                "- case={case}, tick={tick}, field={field}, baseline={baseline}, "
                "new={new}, input={input}, new_branch={new_branch}".format(**failure)
            )
    else:
        md.extend([
            "Case A passed exact horizontal-only regression for `vx`, `vy`, `vz`, "
            "`yaw`, `target_tracked`, and status on horizontal-scope ticks.",
            "",
            "Case B passed: `allow_z=True`, `HORIZONTAL_AND_VERTICAL`, and non-zero "
            "`vz` were observed with fresh target_pose after the horizontal gate.",
            "",
            "Case C passed: stale target_pose forced `allow_z=False` and `vz=0` while "
            "horizontal `vx/vy/yaw` matched the Phase 1 baseline on the stale tick.",
            "",
            "Cases D/E passed: a single-axis error greater than 150 px forced "
            "`horizontal_aligned_now=False`, `allow_z=False`, `vz=0`, and `HORIZONTAL`.",
        ])

    (out_dir / "PHASE2_VERTICAL_COMPARE_RESULT.md").write_text(
        "\n".join(md) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out-dir",
        default=str(REPO_ROOT / "artifacts" / "20260415" / "IF-pid"),
    )
    args = parser.parse_args()

    rows, failures = compare_all()
    write_reports(rows, failures, Path(args.out_dir))
    if failures:
        print(f"FAIL: {len(failures)} Phase 2 comparison failures")
        return 1
    print("PASS: Phase 2 vertical behavior and horizontal regression checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
