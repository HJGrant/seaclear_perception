#!/usr/bin/env python3
"""Independent-model check for staged-PID horizontal math.

This is intentionally not a `VisualServoControllerNode` integration comparison.
It keeps a small independent reference model for quick PID math audits. Use
`compare_visual_servo_controller_nodes_phase1.py` for the stronger fake-ROS
comparison of the BlueROV and grapple controller implementations.

The fixed parameters below mirror the Phase 1 launch values in
`object_detection_ibvs.launch.py` for horizontal behavior.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "sc2_visual_servo_controller"))

from sc2_visual_servo_controller.pid import PIDController, PIDGains  # noqa: E402


LAUNCH_PHASE1_PARAMS = {
    "controller_mode": "staged_pid",
    "rate_hz": 10.0,
    "timeout_ms": 500,
    "target_u": 320.0,
    "target_v": 320.0,
    "image_width": 640.0,
    "image_height": 640.0,
    "max_vx": 0.20,
    "max_vy": 0.20,
    "max_yaw_rate": 0.35,
    "staged_pid_enable_yaw": True,
    "tracked_enter_window_obs": 10,
    "tracked_enter_min_obs": 2,
    "tracked_exit_no_obs_sec": 0.6,
    "decay_rate": 0.1,
}


def clamp_abs(value: float, limit: Optional[float]) -> float:
    if limit is None or limit <= 0.0:
        return value
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


class ReferencePID:
    """Independent copy of the BlueROV PID formula for comparison."""

    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float, output_limit: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None

    def update(self, error: float, dt: float) -> float:
        dt = max(float(dt), 1e-6)
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.integral = clamp_abs(self.integral + error * dt, self.integral_limit)
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return clamp_abs(output, self.output_limit)


@dataclass(frozen=True)
class TickInput:
    t: float
    u: Optional[float]
    v: Optional[float]


@dataclass
class Output:
    vx: float
    vy: float
    yaw: float
    target_tracked: bool
    status: str


class HorizontalModel:
    def __init__(self, *, use_grapple_pid: bool) -> None:
        self.image_width = LAUNCH_PHASE1_PARAMS["image_width"]
        self.target_u = LAUNCH_PHASE1_PARAMS["target_u"]
        self.target_v = LAUNCH_PHASE1_PARAMS["target_v"]
        self.max_vx = LAUNCH_PHASE1_PARAMS["max_vx"]
        self.max_vy = LAUNCH_PHASE1_PARAMS["max_vy"]
        self.max_yaw = LAUNCH_PHASE1_PARAMS["max_yaw_rate"]
        self.decay_rate = LAUNCH_PHASE1_PARAMS["decay_rate"]
        self.timeout_sec = LAUNCH_PHASE1_PARAMS["timeout_ms"] * 1e-3
        self.tracked_exit_no_obs_sec = LAUNCH_PHASE1_PARAMS["tracked_exit_no_obs_sec"]
        self.tracked_enter_min_obs = LAUNCH_PHASE1_PARAMS["tracked_enter_min_obs"]
        self.roi_w = 50.0
        self.enable_yaw = bool(LAUNCH_PHASE1_PARAMS["staged_pid_enable_yaw"])

        if use_grapple_pid:
            gains_x = PIDGains(0.0025, 0.0, 0.0008, integral_limit=200.0, output_limit=self.max_vx)
            gains_y = PIDGains(0.0025, 0.0, 0.0008, integral_limit=200.0, output_limit=self.max_vy)
            self.pid_x = PIDController(gains_x)
            self.pid_y = PIDController(gains_y)
        else:
            self.pid_x = ReferencePID(0.0025, 0.0, 0.0008, 200.0, self.max_vx)
            self.pid_y = ReferencePID(0.0025, 0.0, 0.0008, 200.0, self.max_vy)

        self.target_tracked = False
        self.last_detection_time: Optional[float] = None
        self.last_valid_target_observation_time: Optional[float] = None
        self.last_accepted_centroid: Optional[tuple[float, float]] = None
        self.valid_history: list[bool] = []
        self.latest_centroid: Optional[tuple[float, float]] = None
        self.last_cmd = (0.0, 0.0, 0.0)

    def _reset_pid(self) -> None:
        self.pid_x.reset()
        self.pid_y.reset()

    def _is_fresh_time(self, stamp: Optional[float], now: float, timeout: float) -> bool:
        return stamp is not None and (now - stamp) <= timeout

    def _in_roi(self, u: float, v: float, center: tuple[float, float]) -> bool:
        return (
            center[0] - self.roi_w <= u <= center[0] + self.roi_w
            and center[1] - self.roi_w <= v <= center[1] + self.roi_w
        )

    def _set_target_tracked(self, tracked: bool) -> None:
        if tracked == self.target_tracked:
            return
        self.target_tracked = tracked
        if not tracked:
            self.valid_history.clear()
            self.last_accepted_centroid = None
            self._reset_pid()

    def _update_tracking_from_detection(self, now: float, u: float, v: float) -> None:
        self.latest_centroid = (u, v)
        self.last_detection_time = now
        roi_consistent = (
            self.last_accepted_centroid is None
            or self._in_roi(u, v, self.last_accepted_centroid)
        )
        self.valid_history.append(bool(roi_consistent))
        self.valid_history = self.valid_history[-10:]
        if roi_consistent:
            self.last_accepted_centroid = (u, v)
            self.last_valid_target_observation_time = now
        if not self.target_tracked and sum(self.valid_history) >= self.tracked_enter_min_obs:
            self._set_target_tracked(True)

    def _update_tracking_from_tick(self, now: float) -> None:
        if (
            self.target_tracked
            and not self._is_fresh_time(
                self.last_valid_target_observation_time,
                now,
                self.tracked_exit_no_obs_sec,
            )
        ):
            self._set_target_tracked(False)

    def step(self, tick: TickInput, prev_t: float) -> Output:
        dt = max(1e-4, tick.t - prev_t)
        if tick.u is not None and tick.v is not None:
            self._update_tracking_from_detection(tick.t, tick.u, tick.v)

        self._update_tracking_from_tick(tick.t)
        obs_fresh = self._is_fresh_time(self.last_detection_time, tick.t, self.timeout_sec)

        if not self.target_tracked:
            self.last_cmd = (0.0, 0.0, 0.0)
            return Output(0.0, 0.0, 0.0, False, "TRACKING_STAGED_PID_NO_TARGET")

        if obs_fresh and self.latest_centroid is not None:
            err_x = self.latest_centroid[0] - self.target_u
            err_y = self.latest_centroid[1] - self.target_v
            vx = self.pid_x.update(err_x, dt)
            vy = -self.pid_y.update(err_y, dt)
            yaw = 0.0
            if self.enable_yaw:
                yaw = clamp_abs(0.8 * (err_x / max(1.0, self.image_width * 0.5)), self.max_yaw)
            self.last_cmd = (vx, vy, yaw)
            return Output(vx, vy, yaw, True, "TRACKING_STAGED_PID_HORIZONTAL")

        vx, vy, yaw = self.last_cmd
        self.last_cmd = (vx * self.decay_rate, vy * self.decay_rate, yaw * self.decay_rate)
        return Output(*self.last_cmd, True, "TRACKING_STAGED_PID_HORIZONTAL_STALE")


def compare_sequence() -> tuple[list[dict], list[dict]]:
    sequence = [
        TickInput(0.0, 380.0, 300.0),
        TickInput(0.1, 370.0, 304.0),
        TickInput(0.2, 356.0, 310.0),
        TickInput(0.3, 340.0, 316.0),
        TickInput(0.82, None, None),
    ]

    old = HorizontalModel(use_grapple_pid=False)
    new = HorizontalModel(use_grapple_pid=True)
    rows = []
    mismatches = []
    prev_t = 0.0

    for idx, tick in enumerate(sequence):
        old_out = old.step(tick, prev_t)
        new_out = new.step(tick, prev_t)
        row = {
            "tick": idx,
            "input": {"t": tick.t, "u": tick.u, "v": tick.v},
            "old": old_out.__dict__,
            "new": new_out.__dict__,
        }
        rows.append(row)
        for field in ("vx", "vy", "yaw"):
            if abs(getattr(old_out, field) - getattr(new_out, field)) > 1e-12:
                mismatches.append({
                    "tick": idx,
                    "field": field,
                    "old": getattr(old_out, field),
                    "new": getattr(new_out, field),
                    "input": row["input"],
                    "branch": old_out.status,
                })
        for field in ("target_tracked", "status"):
            if getattr(old_out, field) != getattr(new_out, field):
                mismatches.append({
                    "tick": idx,
                    "field": field,
                    "old": getattr(old_out, field),
                    "new": getattr(new_out, field),
                    "input": row["input"],
                    "branch": old_out.status,
                })
        prev_t = tick.t

    return rows, mismatches


def write_report(rows: list[dict], mismatches: list[dict], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "phase1_horizontal_compare.json"
    md_path = out_dir / "PHASE1_INDEPENDENT_HORIZONTAL_COMPARE_RESULT.md"
    json_path.write_text(json.dumps({"rows": rows, "mismatches": mismatches}, indent=2), encoding="utf-8")

    status = "PASS" if not mismatches else "FAIL"
    lines = [
        "# Phase 1 Horizontal Compare Result",
        "",
        "Comparison type: independent horizontal model / PID math check.",
        "",
        f"Status: {status}",
        "",
        "The parameters mirror the Phase 1 launch values for horizontal behavior.",
        "",
        "| tick | input | old vx/vy/yaw | new vx/vy/yaw | tracked | status |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        old = row["old"]
        new = row["new"]
        inp = row["input"]
        lines.append(
            "| {tick} | t={t:.2f}, u={u}, v={v} | {ovx:.12g}, {ovy:.12g}, {oyaw:.12g} | "
            "{nvx:.12g}, {nvy:.12g}, {nyaw:.12g} | {tracked} | {status} |".format(
                tick=row["tick"],
                t=inp["t"],
                u=inp["u"],
                v=inp["v"],
                ovx=old["vx"],
                ovy=old["vy"],
                oyaw=old["yaw"],
                nvx=new["vx"],
                nvy=new["vy"],
                nyaw=new["yaw"],
                tracked=new["target_tracked"],
                status=new["status"],
            )
        )
    if mismatches:
        lines.extend(["", "## Mismatches", ""])
        for mismatch in mismatches:
            lines.append(
                "- tick={tick}, field={field}, old={old}, new={new}, input={input}, branch={branch}".format(
                    **mismatch
                )
            )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out-dir",
        default=str(REPO_ROOT / "artifacts" / "20260415" / "IF-pid"),
        help="Directory for JSON and markdown comparison reports.",
    )
    args = parser.parse_args()

    rows, mismatches = compare_sequence()
    write_report(rows, mismatches, Path(args.out_dir))
    if mismatches:
        print(f"FAIL: {len(mismatches)} mismatches")
        return 1
    print("PASS: horizontal-only staged PID outputs match")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
