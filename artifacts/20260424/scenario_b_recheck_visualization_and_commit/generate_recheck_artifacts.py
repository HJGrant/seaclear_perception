from __future__ import annotations

import csv
import math
import os
import sqlite3
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Iterable, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import rclpy
import yaml
from matplotlib.animation import FFMpegWriter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from sc2_visual_servo_controller.visual_servo_controller_node_implementation import (
    PIDController,
    PIDGains,
    VisualServoControllerNode,
)
from sc2_visual_servo_perception.tracker_selector_core import (
    DetectionCandidate,
    TrackState,
    TRACK_CONFIRMED,
    TRACK_ESCALATE,
    TRACK_HOLD,
    TRACK_LOST,
    TRACK_TENTATIVE,
    associate_two_stage,
    create_track,
    update_track_with_match,
    update_track_with_miss,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_ROOT = REPO_ROOT / "artifacts/20260424/scenario_b_recheck_visualization_and_commit"
VIDEO_DIR = ARTIFACT_ROOT / "videos"
FIG_DIR = ARTIFACT_ROOT / "figures"
CODE_SNAPSHOT_ROOT = ARTIFACT_ROOT / "code_snapshot"

BAG_PATH = REPO_ROOT / "rosbag2_2026_04_22-15_22_39"
BAG_WINDOW_START = 21.8
BAG_WINDOW_END = 22.8
FPS_SYNTH = 10
FPS_BAG = 12

STATE_TO_Y = {
    "NONE": -1,
    TRACK_TENTATIVE: 0,
    TRACK_CONFIRMED: 1,
    TRACK_HOLD: 2,
    TRACK_LOST: 3,
    TRACK_ESCALATE: 4,
}
STATE_ORDER = ["NONE", TRACK_TENTATIVE, TRACK_CONFIRMED, TRACK_HOLD, TRACK_LOST, TRACK_ESCALATE]


@dataclass
class TrackerConfig:
    name: str
    n_confirm: int
    hold_max_misses: int
    lost_max_misses: int
    t_stale_sec: float
    gate_px: float
    selector_switch_margin: float
    selector_selected_track_bonus: float
    confidence_ema_alpha: float = 0.35
    alpha_uv_predict: float = 0.0


@dataclass
class TaggedDetection:
    tag: str
    candidate: DetectionCandidate


@dataclass
class FrameRecord:
    frame_index: int
    time_sec: float
    detections: list[TaggedDetection]
    selected_track_id: Optional[int]
    selected_valid: bool
    track_state: str
    reason_code: str
    selected_tag: str
    selected_u: float
    selected_v: float
    selected_w: float
    selected_h: float
    matched_tags_by_track: dict[int, str]
    track_states: dict[int, str]
    track_boxes: dict[int, tuple[float, float, float, float]]
    controller_status: str = ""
    controller_locked: bool = False
    cmd_vx: float = 0.0
    cmd_vy: float = 0.0
    cmd_vz: float = 0.0
    cmd_yaw: float = 0.0
    depth_m: float = 1.0
    vel_vx: float = 0.0
    vel_vy: float = 0.0
    vel_yaw_rate: float = 0.0


@dataclass
class ScenarioMetrics:
    scenario: str
    tracker_config: str
    switches: int
    valid_switches: int
    false_spans: int
    false_frames: int
    clutter_false_selects: int
    hold_transitions: int
    lost_transitions: int
    escalate_transitions: int
    hold_rematch_attempts: int
    hold_rematch_successes: int
    true_reacquire_attempts: int
    true_reacquire_successes: int
    hold_rematch_latency_frames_mean: float
    true_reacquire_latency_frames_mean: float


def clamp(value: float, limit: float) -> float:
    lim = max(0.0, float(limit))
    return max(-lim, min(lim, float(value)))


def cand(
    u: float,
    v: float,
    score: float,
    cls_id: str = "3",
    w: float = 42.0,
    h: float = 36.0,
) -> DetectionCandidate:
    return DetectionCandidate(
        detection=None,
        cls_id=cls_id,
        score=float(score),
        u=float(u),
        v=float(v),
        w=float(w),
        h=float(h),
        high_confidence=score >= 0.5,
    )


def selection_score(track: TrackState, now_sec: float, t_stale_sec: float) -> float:
    freshness = 1.0
    if t_stale_sec > 0.0:
        freshness = max(0.0, min(1.0, 1.0 - ((now_sec - track.last_match_time_sec) / t_stale_sec)))
    if track.state == TRACK_CONFIRMED and freshness > 0.0:
        return 1.0 + 0.7 * float(track.confidence_ema) + 0.3 * float(freshness)
    if track.state == TRACK_HOLD:
        return 0.2 + 0.2 * float(track.confidence_ema) + 0.2 * float(freshness)
    return -1.0


def select_track_local(
    tracks: Iterable[TrackState],
    now_sec: float,
    t_stale_sec: float,
    previous_selected_track_id: Optional[int],
    selector_switch_margin: float,
    selector_selected_track_bonus: float,
) -> Optional[TrackState]:
    candidates = [track for track in tracks if track.state in {TRACK_CONFIRMED, TRACK_HOLD}]
    if not candidates:
        return None
    scores = {int(track.track_id): selection_score(track, now_sec, t_stale_sec) for track in candidates}
    best = max(candidates, key=lambda track: scores[int(track.track_id)])
    if previous_selected_track_id is None:
        return best

    previous = None
    for track in candidates:
        if int(track.track_id) == int(previous_selected_track_id):
            previous = track
            break
    if previous is None or previous.state != TRACK_CONFIRMED or previous.freshness <= 0.0:
        return best

    previous_score = scores[int(previous.track_id)] + max(0.0, float(selector_selected_track_bonus))
    switch_threshold = previous_score + max(0.0, float(selector_switch_margin))
    if scores[int(best.track_id)] > switch_threshold:
        return best
    return previous


def scenario_a(seed: int) -> list[list[TaggedDetection]]:
    rng = np.random.default_rng(seed)
    frames: list[list[TaggedDetection]] = []
    base_u = 320.0
    base_v = 240.0
    occlusions = {20, 21, 22, 45, 46, 70, 71, 72, 73, 95, 96}
    for i in range(120):
        dets: list[TaggedDetection] = []
        if i not in occlusions:
            du = 10.0 * math.sin(i * 0.17) + rng.uniform(-3.0, 3.0)
            dv = 8.0 * math.cos(i * 0.11) + rng.uniform(-2.0, 2.0)
            tw = 42.0 + rng.uniform(-2.0, 2.0)
            th = 36.0 + rng.uniform(-2.0, 2.0)
            dets.append(TaggedDetection("target", cand(base_u + du, base_v + dv, 0.80, w=tw, h=th)))
        if i % 9 == 0 or i % 17 == 0:
            cw = 46.0 + rng.uniform(-4.0, 4.0)
            ch = 33.0 + rng.uniform(-4.0, 4.0)
            dets.append(
                TaggedDetection(
                    "competing",
                    cand(
                        base_u + rng.uniform(-45.0, 45.0),
                        base_v + rng.uniform(-35.0, 35.0),
                        0.80 if i % 18 == 0 else 0.25,
                        w=cw,
                        h=ch,
                    ),
                )
            )
        frames.append(dets)
    return frames


def scenario_b_legacy(seed: int, clutter_density: int = 4) -> list[list[TaggedDetection]]:
    rng = np.random.default_rng(seed)
    frames: list[list[TaggedDetection]] = []
    for i in range(120):
        dets: list[TaggedDetection] = []
        if i % 2 == 0:
            dets.append(
                TaggedDetection(
                    "target",
                    cand(
                        160.0 + 40.0 * math.sin(i * 0.08) + rng.uniform(-4.0, 4.0),
                        300.0 + 20.0 * math.cos(i * 0.14) + rng.uniform(-4.0, 4.0),
                        0.80,
                    ),
                )
            )
        for _ in range(clutter_density):
            cls_id = "3" if rng.uniform() < 0.6 else "7"
            dets.append(
                TaggedDetection(
                    "clutter",
                    cand(
                        rng.uniform(0.0, 640.0),
                        rng.uniform(0.0, 640.0),
                        0.80 if rng.uniform() < 0.25 else 0.25,
                        cls_id=cls_id,
                    ),
                )
            )
        if i % 11 == 0:
            dets.append(
                TaggedDetection(
                    "clutter",
                    cand(
                        170.0 + rng.uniform(-20.0, 20.0),
                        300.0 + rng.uniform(-20.0, 20.0),
                        0.80,
                    ),
                )
            )
        frames.append(dets)
    return frames


def scenario_b_variable_bbox(seed: int, clutter_density: int = 5) -> list[list[TaggedDetection]]:
    rng = np.random.default_rng(seed)
    frames: list[list[TaggedDetection]] = []
    clutter_tracks: list[dict[str, object]] = []
    size_modes = [(18.0, 18.0), (34.0, 26.0), (64.0, 52.0)]
    aspect_modes = [(1.0, 1.0), (0.6, 1.4), (1.6, 0.8)]
    for i in range(120):
        dets: list[TaggedDetection] = []
        if i % 2 == 0:
            tw = 40.0 + 4.0 * math.sin(i * 0.15) + rng.uniform(-3.0, 3.0)
            th = 34.0 + 3.0 * math.cos(i * 0.12) + rng.uniform(-2.5, 2.5)
            dets.append(
                TaggedDetection(
                    "target",
                    cand(
                        160.0 + 36.0 * math.sin(i * 0.08) + rng.uniform(-5.0, 5.0),
                        300.0 + 18.0 * math.cos(i * 0.14) + rng.uniform(-5.0, 5.0),
                        0.82,
                        w=tw,
                        h=th,
                    ),
                )
            )

        for track in clutter_tracks:
            track["ttl"] = int(track["ttl"]) - 1
        clutter_tracks = [track for track in clutter_tracks if int(track["ttl"]) > 0]

        while len(clutter_tracks) < clutter_density:
            base_w, base_h = size_modes[int(rng.integers(0, len(size_modes)))]
            scale_x, scale_y = aspect_modes[int(rng.integers(0, len(aspect_modes)))]
            if rng.uniform() < 0.35:
                u = 160.0 + rng.uniform(-55.0, 55.0)
                v = 300.0 + rng.uniform(-45.0, 45.0)
            else:
                u = rng.uniform(0.0, 640.0)
                v = rng.uniform(0.0, 640.0)
            clutter_tracks.append(
                {
                    "u": u,
                    "v": v,
                    "w": base_w * scale_x,
                    "h": base_h * scale_y,
                    "ttl": int(rng.integers(1, 4)),
                    "cls": "3" if rng.uniform() < 0.65 else "7",
                    "score": 0.82 if rng.uniform() < 0.30 else 0.20,
                }
            )

        for clutter in clutter_tracks:
            dets.append(
                TaggedDetection(
                    "clutter",
                    cand(
                        float(clutter["u"]) + rng.uniform(-4.0, 4.0),
                        float(clutter["v"]) + rng.uniform(-4.0, 4.0),
                        float(clutter["score"]),
                        cls_id=str(clutter["cls"]),
                        w=float(clutter["w"]) + rng.uniform(-3.0, 3.0),
                        h=float(clutter["h"]) + rng.uniform(-3.0, 3.0),
                    ),
                )
            )
        frames.append(dets)
    return frames


def run_tracker_sequence(
    scenario_name: str,
    frames: list[list[TaggedDetection]],
    config: TrackerConfig,
    dt_sec: float = 0.1,
) -> tuple[ScenarioMetrics, list[FrameRecord]]:
    tracks: dict[int, TrackState] = {}
    next_track_id = 1
    selected_track_id: Optional[int] = None
    previous_selected_valid = False
    previous_selected_id: Optional[int] = None
    records: list[FrameRecord] = []

    switches = 0
    valid_switches = 0
    false_spans = 0
    false_frames = 0
    clutter_false_selects = 0
    hold_transitions = 0
    lost_transitions = 0
    escalate_transitions = 0
    hold_rematch_attempts = 0
    hold_rematch_successes = 0
    true_reacquire_attempts = 0
    true_reacquire_successes = 0
    hold_rematch_latencies: list[int] = []
    true_reacquire_latencies: list[int] = []

    gap_active = False
    gap_start_idx = -1
    gap_prev_selected_id: Optional[int] = None
    gap_continuity_cut = False
    prev_track_states: dict[int, str] = {}

    for frame_index, detections in enumerate(frames):
        now_sec = frame_index * dt_sec
        candidates = [item.candidate for item in detections]
        active_tracks = [track for track in tracks.values() if track.state not in {TRACK_LOST, TRACK_ESCALATE}]
        assignments, unmatched = associate_two_stage(
            active_tracks,
            candidates,
            gate_px=config.gate_px,
            alpha_uv_predict=config.alpha_uv_predict,
        )
        matched_track_ids = set(assignments.keys())
        matched_tags_by_track: dict[int, str] = {}

        for track_id, (candidate, reason) in assignments.items():
            update_track_with_match(
                tracks[track_id],
                candidate,
                now_sec,
                n_confirm=config.n_confirm,
                confidence_ema_alpha=config.confidence_ema_alpha,
                reason_code=reason,
            )
            for tagged in detections:
                if tagged.candidate is candidate:
                    matched_tags_by_track[int(track_id)] = tagged.tag
                    break

        for track_id, track in list(tracks.items()):
            if track_id in matched_track_ids:
                continue
            update_track_with_miss(
                track,
                now_sec,
                t_stale_sec=config.t_stale_sec,
                hold_max_misses=config.hold_max_misses,
                lost_max_misses=config.lost_max_misses,
            )

        for candidate_index in unmatched:
            candidate = candidates[candidate_index]
            if not candidate.high_confidence:
                continue
            tracks[next_track_id] = create_track(next_track_id, candidate, now_sec)
            matched_tags_by_track[next_track_id] = detections[candidate_index].tag
            next_track_id += 1

        selected = select_track_local(
            tracks.values(),
            now_sec,
            config.t_stale_sec,
            previous_selected_track_id=selected_track_id,
            selector_switch_margin=config.selector_switch_margin,
            selector_selected_track_bonus=config.selector_selected_track_bonus,
        )
        selected_track_id = int(selected.track_id) if selected is not None else None
        selected_valid = bool(selected is not None and selected.state == TRACK_CONFIRMED)
        selected_tag = ""
        if selected is not None:
            selected_tag = matched_tags_by_track.get(int(selected.track_id), "")

        if previous_selected_id is not None and selected_track_id is not None and previous_selected_id != selected_track_id:
            switches += 1
            if previous_selected_valid and selected_valid:
                valid_switches += 1

        if not selected_valid:
            false_frames += 1
            if not gap_active and previous_selected_valid:
                gap_active = True
                gap_start_idx = frame_index
                gap_prev_selected_id = previous_selected_id
                gap_continuity_cut = False
                hold_rematch_attempts += 1
                false_spans += 1
            elif not gap_active and not previous_selected_valid:
                false_spans += 1
                gap_active = True
                gap_start_idx = frame_index
                gap_prev_selected_id = previous_selected_id
                gap_continuity_cut = False
        else:
            if gap_active:
                latency = frame_index - gap_start_idx
                if gap_prev_selected_id is not None and not gap_continuity_cut and selected_track_id == gap_prev_selected_id:
                    hold_rematch_successes += 1
                    hold_rematch_latencies.append(latency)
                elif gap_continuity_cut and selected_tag == "target":
                    true_reacquire_successes += 1
                    true_reacquire_latencies.append(latency)
                gap_active = False

        if gap_active and gap_prev_selected_id is not None:
            prev_state = tracks.get(gap_prev_selected_id).state if gap_prev_selected_id in tracks else TRACK_LOST
            if prev_state in {TRACK_LOST, TRACK_ESCALATE} and not gap_continuity_cut:
                gap_continuity_cut = True
                true_reacquire_attempts += 1

        if selected_valid and selected_tag != "target":
            clutter_false_selects += 1

        current_states = {track_id: track.state for track_id, track in tracks.items()}
        for track_id, state in current_states.items():
            prev_state = prev_track_states.get(track_id)
            if state == TRACK_HOLD and prev_state != TRACK_HOLD:
                hold_transitions += 1
            if state == TRACK_LOST and prev_state != TRACK_LOST:
                lost_transitions += 1
            if state == TRACK_ESCALATE and prev_state != TRACK_ESCALATE:
                escalate_transitions += 1
        prev_track_states = current_states

        track_boxes = {
            int(track_id): (track.u, track.v, track.w, track.h)
            for track_id, track in tracks.items()
            if track.state not in {TRACK_LOST, TRACK_ESCALATE}
        }
        records.append(
            FrameRecord(
                frame_index=frame_index,
                time_sec=now_sec,
                detections=detections,
                selected_track_id=selected_track_id,
                selected_valid=selected_valid,
                track_state=selected.state if selected is not None else "NONE",
                reason_code=selected.reason_code if selected is not None else "not_selected",
                selected_tag=selected_tag,
                selected_u=float(selected.u) if selected is not None else math.nan,
                selected_v=float(selected.v) if selected is not None else math.nan,
                selected_w=float(selected.w) if selected is not None else math.nan,
                selected_h=float(selected.h) if selected is not None else math.nan,
                matched_tags_by_track=matched_tags_by_track,
                track_states=current_states,
                track_boxes=track_boxes,
            )
        )
        previous_selected_valid = selected_valid
        previous_selected_id = selected_track_id

    metrics = ScenarioMetrics(
        scenario=scenario_name,
        tracker_config=config.name,
        switches=switches,
        valid_switches=valid_switches,
        false_spans=false_spans,
        false_frames=false_frames,
        clutter_false_selects=clutter_false_selects,
        hold_transitions=hold_transitions,
        lost_transitions=lost_transitions,
        escalate_transitions=escalate_transitions,
        hold_rematch_attempts=hold_rematch_attempts,
        hold_rematch_successes=hold_rematch_successes,
        true_reacquire_attempts=true_reacquire_attempts,
        true_reacquire_successes=true_reacquire_successes,
        hold_rematch_latency_frames_mean=float(np.mean(hold_rematch_latencies)) if hold_rematch_latencies else math.nan,
        true_reacquire_latency_frames_mean=float(np.mean(true_reacquire_latencies)) if true_reacquire_latencies else math.nan,
    )
    return metrics, records


def _dummy_publish(*_args, **_kwargs) -> None:
    return None


def make_velocity_msg(vx: float, vy: float, yaw_rate: float):
    linear = SimpleNamespace(x=float(vx), y=float(vy), z=0.0)
    angular = SimpleNamespace(x=0.0, y=0.0, z=float(yaw_rate))
    return SimpleNamespace(twist=SimpleNamespace(linear=linear, angular=angular))


def make_target_obs_msg(record: FrameRecord):
    return SimpleNamespace(
        selected_valid=bool(record.selected_valid),
        track_state=str(record.track_state),
        reason_code=str(record.reason_code),
        u=0.0 if math.isnan(record.selected_u) else float(record.selected_u),
        v=0.0 if math.isnan(record.selected_v) else float(record.selected_v),
    )


def configure_controller_node(node: VisualServoControllerNode) -> None:
    node._use_target_observation = True
    node._controller_mode = "staged_pid"
    node._timeout_sec = 0.2
    node._target_point_timeout_sec = 0.2
    node._max_vx = 0.20
    node._max_vy = 0.20
    node._max_vz = 0.20
    node._max_yaw = 0.35
    node._depth_mode = "DVL"
    node._staged_pid_z_source = "dvl"
    node.image_width = 640.0
    node.image_height = 640.0
    node._target_u = 320.0
    node._target_v = 320.0
    node._staged_pid_target_centroid = np.array([320.0, 320.0], dtype=np.float32)
    node._desired_target_dist = 0.5
    node._desired_vertical_dist = 1.0
    node._staged_pid_enable_yaw = True
    node._hold_maintain_enable = True
    node._hold_maintain_z_kp_scale = 0.5
    node._hold_maintain_velocity_damping_enable = True
    node._hold_maintain_xy_damping = 0.4
    node._hold_maintain_yaw_damping = 0.3
    node._hold_maintain_velocity_timeout_sec = 0.5
    node._tracked_enter_window_obs = 10
    node._tracked_enter_min_obs = 2
    node._tracked_exit_no_obs_sec = 0.6
    node._tracking_valid_observation_history = deque(maxlen=node._tracked_enter_window_obs)
    node._xy_pid_x = PIDController(
        PIDGains(kp=0.0025, ki=0.0, kd=0.0008, integral_limit=200.0, output_limit=node._max_vx)
    )
    node._xy_pid_y = PIDController(
        PIDGains(kp=0.0025, ki=0.0, kd=0.0008, integral_limit=200.0, output_limit=node._max_vy)
    )
    node._z_pid = PIDController(
        PIDGains(kp=0.8, ki=0.0, kd=0.10, integral_limit=1.0, output_limit=node._max_vz)
    )
    node._staged_pid_horizontal_tol_x_px = 150.0
    node._staged_pid_horizontal_tol_y_px = 150.0
    node._staged_pid_vertical_tol_px = 150.0
    node._staged_pid_distance_tol_m = 0.08
    node._staged_pid_vertical_gate_window_ticks = 10
    node._staged_pid_vertical_enter_ratio = 0.8
    node._staged_pid_vertical_exit_ratio = 0.4
    node._staged_pid_vertical_enter_hold_ticks = 2
    node._staged_pid_vertical_exit_hold_ticks = 2
    node._staged_pid_recent_horizontal_ok = deque(maxlen=node._staged_pid_vertical_gate_window_ticks)
    node._staged_pid_vertical_enabled = False
    node._staged_pid_enter_counter = 0
    node._staged_pid_exit_counter = 0
    node._latest_depth_measurement = 1.0
    node._last_velocity_measurement_time = None
    node.latest_velocity_measurement = None
    node._centroid_error_publisher.publish = _dummy_publish
    node._image_plane_vel_pub.publish = _dummy_publish
    node._sc2_base_link_vel_pub.publish = _dummy_publish
    node._cmd_pub.publish = _dummy_publish
    node._sc2_cmd_pub_vec_6.publish = _dummy_publish


def run_controller_harness(
    records: list[FrameRecord],
    depths: list[float],
    velocities: list[tuple[float, float, float]],
) -> list[FrameRecord]:
    node = VisualServoControllerNode()
    configure_controller_node(node)

    status_holder = {"status": "", "locked": False}

    def capture_status(status: str) -> None:
        status_holder["status"] = str(status)

    def capture_locked(value: str) -> None:
        status_holder["locked"] = str(value).strip().lower() == "true"

    def capture_cmd(vx: float, vy: float, vz: float, yaw: float) -> None:
        node._last_cmd = (float(vx), float(vy), float(vz), float(yaw))

    node._publish_status = capture_status
    node._publish_is_locked = capture_locked
    node._publish_cmd_direct_body = capture_cmd

    for idx, record in enumerate(records):
        depth = float(depths[idx])
        vel_vx, vel_vy, vel_yaw = velocities[idx]
        node._dvl_altitude_cb(SimpleNamespace(altitude=depth))
        node._vel_measurement_cb(make_velocity_msg(vel_vx, vel_vy, vel_yaw))
        node._target_observation_cb(make_target_obs_msg(record))
        prev_time = records[idx - 1].time_sec if idx > 0 else record.time_sec - 0.1
        dt_ctrl = max(1e-3, float(record.time_sec - prev_time))
        node._tick_staged_pid(dt_ctrl, 0.0)
        record.depth_m = depth
        record.vel_vx = vel_vx
        record.vel_vy = vel_vy
        record.vel_yaw_rate = vel_yaw
        record.controller_status = str(status_holder["status"])
        record.controller_locked = bool(status_holder["locked"])
        record.cmd_vx, record.cmd_vy, record.cmd_vz, record.cmd_yaw = node._last_cmd

    node.destroy_node()
    return records


def write_parameter_comparison(metrics_rows: list[ScenarioMetrics]) -> None:
    csv_path = ARTIFACT_ROOT / "scenario_b_parameter_comparison.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "scenario",
            "tracker_config",
            "switches",
            "valid_switches",
            "false_spans",
            "false_frames",
            "clutter_false_selects",
            "hold_transitions",
            "lost_transitions",
            "escalate_transitions",
            "hold_rematch_attempts",
            "hold_rematch_successes",
            "true_reacquire_attempts",
            "true_reacquire_successes",
            "hold_rematch_latency_frames_mean",
            "true_reacquire_latency_frames_mean",
        ])
        for row in metrics_rows:
            writer.writerow([
                row.scenario,
                row.tracker_config,
                row.switches,
                row.valid_switches,
                row.false_spans,
                row.false_frames,
                row.clutter_false_selects,
                row.hold_transitions,
                row.lost_transitions,
                row.escalate_transitions,
                row.hold_rematch_attempts,
                row.hold_rematch_successes,
                row.true_reacquire_attempts,
                row.true_reacquire_successes,
                f"{row.hold_rematch_latency_frames_mean:.3f}" if not math.isnan(row.hold_rematch_latency_frames_mean) else "",
                f"{row.true_reacquire_latency_frames_mean:.3f}" if not math.isnan(row.true_reacquire_latency_frames_mean) else "",
            ])

    lines = [
        "# Scenario B Parameter Comparison",
        "",
        "Scenario B を legacy synthetic 条件と production 寄り条件で比較した。",
        "",
        "## Parameter Sets",
        "",
        "- legacy synthetic",
        "  - `n_confirm=3`",
        "  - `hold_max_misses=6`",
        "  - `lost_max_misses=12`",
        "  - `gate_px=70`",
        "  - `selector_switch_margin=0.05`",
        "  - `selector_selected_track_bonus=0.05`",
        "- production-like",
        "  - `n_confirm=5`",
        "  - `hold_max_misses=12`",
        "  - `lost_max_misses=30`",
        "  - `gate_px=80`",
        "  - `selector_switch_margin=0.05`",
        "  - `selector_selected_track_bonus=0.05`",
        "",
        "差分が残る点:",
        "- synthetic generator は bag/実機 detector の score 分布を再現していない",
        "- low-confidence 閾値は candidate score 生成で間接的に与えており、observer node の全 filtering path をそのまま通してはいない",
        "",
        "| scenario | config | switches | valid switches | false spans | false frames | clutter false selects | HOLD | LOST | ESCALATE | hold rematch | true reacquire |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in metrics_rows:
        lines.append(
            f"| `{row.scenario}` | `{row.tracker_config}` | {row.switches} | {row.valid_switches} | "
            f"{row.false_spans} | {row.false_frames} | {row.clutter_false_selects} | "
            f"{row.hold_transitions} | {row.lost_transitions} | {row.escalate_transitions} | "
            f"{row.hold_rematch_successes}/{row.hold_rematch_attempts} | "
            f"{row.true_reacquire_successes}/{row.true_reacquire_attempts} |"
        )
    (ARTIFACT_ROOT / "scenario_b_parameter_comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_feedback_note() -> None:
    text = """# Feedback Adoption Note

## Adopted

- `switches=29` を軽く扱わない
  - 120 frame / 12 s で 29 switch は約 0.41 s ごとの切替であり、追従 continuity リスクとして Red 扱いに上げる
- synthetic / production parameter 差分を明示する
  - legacy synthetic と production-like を並列比較する
- reacquire 指標を分ける
  - `hold_rematch_success`
  - `true_reacquire_success`

## Partially Adopted

- Scenario B が実海中 clutter をどこまで代表するか
  - 代表性は未証明
  - ただし stress test としては十分に有用なので gate 判断には使う
- tuning だけでよいか
  - まず tuning gate を置く
  - Red の場合は score 設計や空間一貫性追加も候補に上げる

## Held / Not Claimed

- Scenario B の結果だけで production rejection を即断すること
  - そこまでは言わない
  - ただし実機前の暫定 gate では Red 扱いにする

## Updated Decision Flow

1. production-like Scenario B を再評価する
2. `valid_switches` と `clutter_false_selects` を優先指標にする
3. Yellow までは tuning 再試行
4. Red は設計変更 table を先に開く
5. 最終判断は bag / 実機で確認する
"""
    (ARTIFACT_ROOT / "feedback_adoption_note.md").write_text(text, encoding="utf-8")


def write_scenario_spec() -> None:
    text = """# Synthetic Scenario Spec

## Scenario A

- 主 target は近傍を滑らかに移動
- occlusion を短 / 中 / 長で混ぜる
- bbox size は微小揺れを持つ
- competing candidate は近傍に出し、size を少しずらす

## Scenario B Legacy

- 既存 stress 条件の再現
- clutter bbox は固定寸法
- same-class clutter を高頻度に混ぜる

## Scenario B Variable BBox

- clutter bbox を size / aspect ratio 可変に拡張
- size: 小 / 中 / 大
- aspect ratio: 正方 / 縦長 / 横長
- 一部 clutter を true target 近傍へ出す
- same-class / other-class を混在させる
- clutter 寿命は 1-3 frame 中心

## Fixed Seeds

- Scenario A: `1234`
- Scenario B Legacy: `5678`
- Scenario B Production-like: `5678`
- Scenario B Variable BBox: `9012`
"""
    (ARTIFACT_ROOT / "synthetic_scenario_spec.md").write_text(text, encoding="utf-8")


def write_reacquire_definition(metrics_rows: list[ScenarioMetrics]) -> None:
    text = """# Reacquire Metric Definition

## hold_rematch_success

- invalid / HOLD span に入る直前の selected track を continuity 候補とする
- continuity が切れる前に、同じ `selected_track_id` へ valid 状態で戻ったら success

## true_reacquire_success

- invalid span 中に previous selected track が `LOST` または `ESCALATE` へ移行したら continuity cut とみなす
- その後、true target へ valid selection が戻ったら success
- synthetic では tag=`target` を ground truth として判定した

## Notes

- hold rematch は track table continuity が残っている戻り
- true reacquire は実質的に new track 再取得を含む戻り
- latency は invalid span 開始から success までの frame 数平均
"""
    (ARTIFACT_ROOT / "reacquire_metric_definition.md").write_text(text, encoding="utf-8")

    csv_path = ARTIFACT_ROOT / "reacquire_metric_table.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "scenario",
            "tracker_config",
            "hold_rematch_attempts",
            "hold_rematch_successes",
            "hold_rematch_latency_frames_mean",
            "true_reacquire_attempts",
            "true_reacquire_successes",
            "true_reacquire_latency_frames_mean",
        ])
        for row in metrics_rows:
            writer.writerow([
                row.scenario,
                row.tracker_config,
                row.hold_rematch_attempts,
                row.hold_rematch_successes,
                f"{row.hold_rematch_latency_frames_mean:.3f}" if not math.isnan(row.hold_rematch_latency_frames_mean) else "",
                row.true_reacquire_attempts,
                row.true_reacquire_successes,
                f"{row.true_reacquire_latency_frames_mean:.3f}" if not math.isnan(row.true_reacquire_latency_frames_mean) else "",
            ])


def write_design_change_gate(metrics_lookup: dict[tuple[str, str], ScenarioMetrics]) -> None:
    gate_text = """# Design Change Gate

## Temporary Gate

| gate | condition | action |
|---|---|---|
| Green | `switches <= 3`, `clutter_false_selects = 0`, no repeated `valid_switches` burst | keep tuning only |
| Yellow | `4 <= switches <= 9` or `clutter_false_selects = 1` | retune margin / bonus / gate and rerun |
| Red | `switches >= 10` or `clutter_false_selects >= 2` or repeated `valid_switches` | open design-change options table before real-world test |

## Current Placement

- `scenario_b` legacy: Red
- `scenario_b` production-like: Red if the rerun still exceeds the threshold
- `variable_bbox_b` production-like: treat as Red when it exceeds the same threshold
"""
    (ARTIFACT_ROOT / "design_change_gate.md").write_text(gate_text, encoding="utf-8")

    rows = [
        ["symptom", "tuning candidate", "design candidate", "note"],
        [
            "many valid switches",
            "increase `selector_switch_margin`, increase selected-track bonus",
            "add temporal continuity term / hysteresis over multiple frames",
            "only tuning if Yellow",
        ],
        [
            "same-class clutter false selects",
            "tighten gate, raise confirm requirement",
            "class-aware or shape-aware score, add spatial consistency penalty",
            "Red if repeated while valid",
        ],
        [
            "reacquire too weak after continuity cut",
            "relax hold / lost transition boundary",
            "explicit reacquire state or target identity memory",
            "consider only after production-like Red",
        ],
    ]
    with (ARTIFACT_ROOT / "design_change_options_table.md").open("w", encoding="utf-8", newline="") as handle:
        for row in rows:
            handle.write("| " + " | ".join(row) + " |\n")
            if row is rows[0]:
                handle.write("|---|---|---|---|\n")


def plot_case_response(case_name: str, records: list[FrameRecord]) -> None:
    time_axis = [record.time_sec for record in records]
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
    axes[0].plot(time_axis, [len(record.detections) for record in records], label="detections")
    axes[0].set_ylabel("detections")
    axes[0].legend(loc="upper right")

    axes[1].plot(time_axis, [record.selected_track_id if record.selected_track_id is not None else -1 for record in records], label="selected id")
    axes[1].plot(time_axis, [1 if record.selected_valid else 0 for record in records], label="valid")
    axes[1].set_ylabel("selection")
    axes[1].legend(loc="upper right")

    axes[2].plot(time_axis, [record.selected_u - 320.0 if not math.isnan(record.selected_u) else math.nan for record in records], label="u error")
    axes[2].plot(time_axis, [record.selected_v - 320.0 if not math.isnan(record.selected_v) else math.nan for record in records], label="v error")
    axes[2].set_ylabel("bbox err [px]")
    axes[2].legend(loc="upper right")

    axes[3].plot(time_axis, [record.cmd_vx for record in records], label="vx")
    axes[3].plot(time_axis, [record.cmd_vy for record in records], label="vy")
    axes[3].plot(time_axis, [record.cmd_vz for record in records], label="vz")
    axes[3].plot(time_axis, [record.cmd_yaw for record in records], label="yaw")
    axes[3].set_ylabel("cmd")
    axes[3].legend(loc="upper right", ncol=4)

    axes[4].plot(time_axis, [STATE_TO_Y.get(record.track_state, -1) for record in records], label="track state")
    axes[4].plot(time_axis, [record.depth_m for record in records], label="depth")
    axes[4].set_yticks(list(STATE_TO_Y.values()))
    axes[4].set_yticklabels(STATE_ORDER)
    axes[4].set_ylabel("state / depth")
    axes[4].set_xlabel("time [s]")
    axes[4].legend(loc="upper right")

    fig.suptitle(case_name)
    fig.tight_layout()
    fig.savefig(FIG_DIR / f"{case_name}_control_response.png", dpi=160)
    plt.close(fig)


def plot_parameter_comparison(metrics_rows: list[ScenarioMetrics]) -> None:
    labels = [f"{row.scenario}\n{row.tracker_config}" for row in metrics_rows]
    switches = [row.switches for row in metrics_rows]
    valid_switches = [row.valid_switches for row in metrics_rows]
    clutter_false = [row.clutter_false_selects for row in metrics_rows]
    x = np.arange(len(labels))
    width = 0.25
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.bar(x - width, switches, width=width, label="switches")
    ax.bar(x, valid_switches, width=width, label="valid switches")
    ax.bar(x + width, clutter_false, width=width, label="clutter false selects")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("count")
    ax.set_title("Scenario comparison")
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "scenario_parameter_comparison.png", dpi=160)
    plt.close(fig)


def load_metadata(bag_dir: Path) -> dict:
    return yaml.safe_load((bag_dir / "metadata.yaml").read_text(encoding="utf-8"))["rosbag2_bagfile_information"]


def first_db3(bag_dir: Path) -> Path:
    return sorted(bag_dir.glob("*.db3"))[0]


def iter_topic_messages(conn: sqlite3.Connection, topic_name: str) -> Iterable[tuple[int, object]]:
    cur = conn.cursor()
    cur.execute("SELECT id, type FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if row is None:
        return
    topic_id, topic_type = row
    msg_type = get_message(topic_type)
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,))
    for ts, blob in cur:
        yield int(ts), deserialize_message(blob, msg_type)


def image_to_rgb(msg) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(msg.encoding).lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if encoding in {"rgb8", "bgr8"}:
        image = data.reshape((height, width, 3))
        if encoding == "bgr8":
            image = image[:, :, ::-1]
        return image
    if encoding == "mono8":
        image = data.reshape((height, width))
        return np.stack([image, image, image], axis=-1)
    raise RuntimeError(f"unsupported image encoding: {msg.encoding}")


def detection_msg_to_candidates(msg) -> list[TaggedDetection]:
    tagged: list[TaggedDetection] = []
    for det in msg.detections:
        score = 0.0
        cls_id = ""
        if det.results:
            hyp = det.results[0].hypothesis
            cls_id = str(hyp.class_id)
            score = float(hyp.score)
        if cls_id != "3" or score < 0.15:
            continue
        tagged.append(
            TaggedDetection(
                tag="bag_target",
                candidate=cand(
                    float(det.bbox.center.position.x),
                    float(det.bbox.center.position.y),
                    score,
                    cls_id=cls_id,
                    w=float(det.bbox.size_x),
                    h=float(det.bbox.size_y),
                ),
            )
        )
    return tagged


def build_bag_records(config: TrackerConfig) -> tuple[list[FrameRecord], list[np.ndarray]]:
    conn = sqlite3.connect(str(first_db3(BAG_PATH)))
    metadata = load_metadata(BAG_PATH)
    bag_start_ns = int(metadata["starting_time"]["nanoseconds_since_epoch"])

    detections: list[tuple[int, list[TaggedDetection]]] = []
    for ts, msg in iter_topic_messages(conn, "/detections_output"):
        rel_sec = (ts - bag_start_ns) * 1e-9
        if rel_sec > BAG_WINDOW_END + 0.6:
            break
        detections.append((ts, detection_msg_to_candidates(msg)))

    tracks: dict[int, TrackState] = {}
    next_track_id = 1
    selected_track_id: Optional[int] = None
    records: list[FrameRecord] = []
    dvl_times: list[int] = []
    dvl_values: list[float] = []
    odom_times: list[int] = []
    odom_values: list[tuple[float, float, float]] = []

    for ts, msg in iter_topic_messages(conn, "/waterlinked_dvl_driver/velocity_report"):
        dvl_times.append(ts)
        dvl_values.append(float(abs(msg.altitude)))
    for ts, msg in iter_topic_messages(conn, "/sc2/odometry_ekf"):
        odom_times.append(ts)
        twist = msg.twist.twist
        odom_values.append((float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)))

    for frame_index, (ts, detections_frame) in enumerate(detections):
        now_sec = (ts - bag_start_ns) * 1e-9
        candidates = [item.candidate for item in detections_frame]
        active_tracks = [track for track in tracks.values() if track.state not in {TRACK_LOST, TRACK_ESCALATE}]
        assignments, unmatched = associate_two_stage(
            active_tracks,
            candidates,
            gate_px=config.gate_px,
            alpha_uv_predict=config.alpha_uv_predict,
        )
        matched_track_ids = set(assignments.keys())
        matched_tags_by_track: dict[int, str] = {}
        for track_id, (candidate, reason) in assignments.items():
            update_track_with_match(
                tracks[track_id],
                candidate,
                now_sec,
                n_confirm=config.n_confirm,
                confidence_ema_alpha=config.confidence_ema_alpha,
                reason_code=reason,
            )
            matched_tags_by_track[int(track_id)] = "bag_target"
        for track_id, track in list(tracks.items()):
            if track_id in matched_track_ids:
                continue
            update_track_with_miss(
                track,
                now_sec,
                t_stale_sec=config.t_stale_sec,
                hold_max_misses=config.hold_max_misses,
                lost_max_misses=config.lost_max_misses,
            )
        for candidate_index in unmatched:
            candidate = candidates[candidate_index]
            if not candidate.high_confidence:
                continue
            tracks[next_track_id] = create_track(next_track_id, candidate, now_sec)
            next_track_id += 1
        selected = select_track_local(
            tracks.values(),
            now_sec,
            config.t_stale_sec,
            previous_selected_track_id=selected_track_id,
            selector_switch_margin=config.selector_switch_margin,
            selector_selected_track_bonus=config.selector_selected_track_bonus,
        )
        selected_track_id = int(selected.track_id) if selected is not None else None
        rel_sec = (ts - bag_start_ns) * 1e-9
        if BAG_WINDOW_START <= rel_sec <= BAG_WINDOW_END:
            dvl_idx = max(0, bisect_right(dvl_times, ts) - 1)
            odom_idx = max(0, bisect_right(odom_times, ts) - 1)
            depth_m = dvl_values[dvl_idx] if dvl_values else 1.0
            vel = odom_values[odom_idx] if odom_values else (0.0, 0.0, 0.0)
            records.append(
                FrameRecord(
                    frame_index=len(records),
                    time_sec=rel_sec - BAG_WINDOW_START,
                    detections=detections_frame,
                    selected_track_id=selected_track_id,
                    selected_valid=bool(selected is not None and selected.state == TRACK_CONFIRMED),
                    track_state=selected.state if selected is not None else "NONE",
                    reason_code=selected.reason_code if selected is not None else "not_selected",
                    selected_tag="bag_target" if selected is not None else "",
                    selected_u=float(selected.u) if selected is not None else math.nan,
                    selected_v=float(selected.v) if selected is not None else math.nan,
                    selected_w=float(selected.w) if selected is not None else math.nan,
                    selected_h=float(selected.h) if selected is not None else math.nan,
                    matched_tags_by_track=matched_tags_by_track,
                    track_states={tid: track.state for tid, track in tracks.items()},
                    track_boxes={tid: (track.u, track.v, track.w, track.h) for tid, track in tracks.items()},
                    depth_m=depth_m,
                    vel_vx=vel[0],
                    vel_vy=vel[1],
                    vel_yaw_rate=vel[2],
                )
            )

    image_frames: list[np.ndarray] = []
    image_times: list[float] = []
    for ts, msg in iter_topic_messages(conn, "/oak/rgb/image_raw"):
        rel_sec = (ts - bag_start_ns) * 1e-9
        if rel_sec < BAG_WINDOW_START or rel_sec > BAG_WINDOW_END:
            continue
        image_frames.append(image_to_rgb(msg))
        image_times.append(rel_sec - BAG_WINDOW_START)
    conn.close()

    aligned_records: list[FrameRecord] = []
    if records:
        record_times = [record.time_sec for record in records]
        for time_sec, image in zip(image_times, image_frames):
            idx = max(0, bisect_right(record_times, time_sec) - 1)
            frame = records[idx]
            aligned_records.append(
                FrameRecord(
                    frame_index=len(aligned_records),
                    time_sec=time_sec,
                    detections=frame.detections,
                    selected_track_id=frame.selected_track_id,
                    selected_valid=frame.selected_valid,
                    track_state=frame.track_state,
                    reason_code=frame.reason_code,
                    selected_tag=frame.selected_tag,
                    selected_u=frame.selected_u,
                    selected_v=frame.selected_v,
                    selected_w=frame.selected_w,
                    selected_h=frame.selected_h,
                    matched_tags_by_track=frame.matched_tags_by_track,
                    track_states=frame.track_states,
                    track_boxes=frame.track_boxes,
                    depth_m=frame.depth_m,
                    vel_vx=frame.vel_vx,
                    vel_vy=frame.vel_vy,
                    vel_yaw_rate=frame.vel_yaw_rate,
                )
            )
    return aligned_records, image_frames


def build_synthetic_depth_and_velocity(records: list[FrameRecord], profile: str) -> tuple[list[float], list[tuple[float, float, float]]]:
    depths: list[float] = []
    velocities: list[tuple[float, float, float]] = []
    for idx, record in enumerate(records):
        time_sec = record.time_sec
        depths.append(1.0 + 0.06 * math.sin(time_sec * 0.7))
        if profile == "quiet":
            velocities.append((0.02 * math.sin(time_sec), 0.015 * math.cos(time_sec * 0.9), 0.04 * math.sin(time_sec * 0.8)))
        else:
            velocities.append((0.09 * math.sin(time_sec * 1.2), 0.07 * math.cos(time_sec * 0.8), 0.20 * math.sin(time_sec * 0.7)))
    return depths, velocities


def render_synthetic_video(path: Path, title: str, records: list[FrameRecord], fps: int) -> None:
    writer = FFMpegWriter(fps=fps, codec="mpeg4", bitrate=1200)
    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    with writer.saving(fig, str(path), 120):
        for record in records:
            ax.clear()
            ax.set_facecolor("black")
            ax.set_xlim(0, 640)
            ax.set_ylim(640, 0)
            ax.set_title(title)
            for tagged in record.detections:
                color = "yellow" if tagged.tag == "target" else ("red" if tagged.tag == "clutter" else "orange")
                rect = patches.Rectangle(
                    (tagged.candidate.u - tagged.candidate.w / 2.0, tagged.candidate.v - tagged.candidate.h / 2.0),
                    tagged.candidate.w,
                    tagged.candidate.h,
                    linewidth=1.6,
                    edgecolor=color,
                    facecolor="none",
                )
                ax.add_patch(rect)
                ax.text(tagged.candidate.u, tagged.candidate.v - tagged.candidate.h / 2.0 - 4.0, tagged.tag, color=color, fontsize=7)
            if record.selected_track_id is not None and not math.isnan(record.selected_u):
                rect = patches.Rectangle(
                    (record.selected_u - record.selected_w / 2.0, record.selected_v - record.selected_h / 2.0),
                    record.selected_w,
                    record.selected_h,
                    linewidth=2.5,
                    edgecolor="lime",
                    facecolor="none",
                )
                ax.add_patch(rect)
            ax.text(8, 18, f"t={record.time_sec:5.2f}s", color="white", fontsize=9)
            ax.text(8, 40, f"selected_valid={record.selected_valid}", color="white", fontsize=9)
            ax.text(8, 62, f"selected_track_id={record.selected_track_id}", color="white", fontsize=9)
            ax.text(8, 84, f"track_state={record.track_state}", color="white", fontsize=9)
            ax.text(8, 106, f"reason={record.reason_code}", color="white", fontsize=9)
            ax.text(8, 128, "legend: target=yellow clutter=red selected=lime", color="white", fontsize=9)
            writer.grab_frame()
    plt.close(fig)


def render_bag_video(path: Path, title: str, records: list[FrameRecord], images: list[np.ndarray], fps: int) -> None:
    writer = FFMpegWriter(fps=fps, codec="mpeg4", bitrate=1800)
    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    stride = max(1, int(round(len(images) / max(1, int((BAG_WINDOW_END - BAG_WINDOW_START) * fps)))))
    sampled = list(range(0, len(images), stride))
    with writer.saving(fig, str(path), 120):
        for image_index in sampled:
            image = images[image_index]
            record = records[min(image_index, len(records) - 1)]
            ax.clear()
            ax.imshow(image)
            ax.set_title(title)
            ax.set_axis_off()
            for tagged in record.detections:
                rect = patches.Rectangle(
                    (tagged.candidate.u - tagged.candidate.w / 2.0, tagged.candidate.v - tagged.candidate.h / 2.0),
                    tagged.candidate.w,
                    tagged.candidate.h,
                    linewidth=1.5,
                    edgecolor="yellow",
                    facecolor="none",
                )
                ax.add_patch(rect)
                ax.text(tagged.candidate.u, tagged.candidate.v - tagged.candidate.h / 2.0 - 4.0, f"{tagged.candidate.cls_id}:{tagged.candidate.score:.2f}", color="yellow", fontsize=7)
            if record.selected_track_id is not None and not math.isnan(record.selected_u):
                rect = patches.Rectangle(
                    (record.selected_u - record.selected_w / 2.0, record.selected_v - record.selected_h / 2.0),
                    record.selected_w,
                    record.selected_h,
                    linewidth=2.5,
                    edgecolor="lime",
                    facecolor="none",
                )
                ax.add_patch(rect)
                ax.text(record.selected_u, record.selected_v + record.selected_h / 2.0 + 12.0, f"id={record.selected_track_id}", color="lime", fontsize=8)
            ax.text(8, 18, f"t={record.time_sec + BAG_WINDOW_START:5.2f}s", color="white", fontsize=9, bbox={"facecolor": "black", "alpha": 0.6, "pad": 2})
            ax.text(8, 40, f"selected_valid={record.selected_valid}", color="white", fontsize=9, bbox={"facecolor": "black", "alpha": 0.6, "pad": 2})
            ax.text(8, 62, f"track_state={record.track_state}", color="white", fontsize=9, bbox={"facecolor": "black", "alpha": 0.6, "pad": 2})
            ax.text(8, 84, f"reason={record.reason_code}", color="white", fontsize=9, bbox={"facecolor": "black", "alpha": 0.6, "pad": 2})
            writer.grab_frame()
    plt.close(fig)


def write_video_manifest(video_paths: list[Path]) -> None:
    lines = ["# Video Manifest", ""]
    for video in video_paths:
        size_mb = video.stat().st_size / (1024.0 * 1024.0)
        lines.append(f"- `{video.name}`: {size_mb:.2f} MiB")
    (ARTIFACT_ROOT / "video_manifest.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_control_response_summary(case_map: dict[str, list[FrameRecord]]) -> None:
    lines = [
        "# Control Response Summary",
        "",
        "controller command は actual `VisualServoControllerNode` を in-process harness で駆動して取得した。",
        "",
        "経路:",
        "- synthetic / bag 由来の `TargetObservation` 相当入力を `_target_observation_cb` に投入",
        "- depth は `_dvl_altitude_cb` へ投入",
        "- body velocity measurement は `_vel_measurement_cb` へ投入",
        "- command は `_tick_staged_pid` から `_publish_cmd_direct_body_with_slew` 経由で取得",
        "",
        "| case | max | mean abs vx | mean abs vy | mean abs vz | mean abs yaw | first hold status |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    for case_name, records in case_map.items():
        max_cmd = max(
            max(abs(r.cmd_vx), abs(r.cmd_vy), abs(r.cmd_vz), abs(r.cmd_yaw))
            for r in records
        )
        hold_status = next((r.controller_status for r in records if "HOLD_SELECTED_INVALID" in r.controller_status), "")
        lines.append(
            f"| `{case_name}` | {max_cmd:.3f} | "
            f"{np.mean([abs(r.cmd_vx) for r in records]):.3f} | "
            f"{np.mean([abs(r.cmd_vy) for r in records]):.3f} | "
            f"{np.mean([abs(r.cmd_vz) for r in records]):.3f} | "
            f"{np.mean([abs(r.cmd_yaw) for r in records]):.3f} | `{hold_status}` |"
        )
    (ARTIFACT_ROOT / "control_response_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def copy_snapshot(files: list[Path]) -> None:
    for file_path in files:
        rel = file_path.relative_to(REPO_ROOT)
        dst = CODE_SNAPSHOT_ROOT / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(file_path.read_bytes())


def build_manifest(files: list[Path], videos: list[Path], figures: list[Path]) -> None:
    lines = [
        "# Artifact Manifest",
        "",
        "## Text / Source",
        "",
    ]
    for file_path in files:
        lines.append(f"- `{file_path.relative_to(ARTIFACT_ROOT)}`")
    lines.extend(["", "## Videos", ""])
    for video in videos:
        lines.append(f"- `{video.relative_to(ARTIFACT_ROOT)}`")
    lines.extend(["", "## Figures", ""])
    for figure in figures:
        lines.append(f"- `{figure.relative_to(ARTIFACT_ROOT)}`")
    lines.extend(["", "## Code Snapshot", ""])
    for file_path in files:
        rel = file_path.relative_to(REPO_ROOT)
        lines.append(f"- `code_snapshot/{rel}`")
    (ARTIFACT_ROOT / "ARTIFACT_MANIFEST.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    changed = [
        "# Changed Files List",
        "",
        "## New",
        "",
    ]
    for file_path in files + videos + figures:
        changed.append(f"- `{file_path.relative_to(REPO_ROOT)}`")
    changed.extend(["", "## Updated", "", "- none", "", "## Deleted", "", "- none"])
    (ARTIFACT_ROOT / "changed_files_list.md").write_text("\n".join(changed) + "\n", encoding="utf-8")


def main() -> None:
    ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)
    VIDEO_DIR.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    CODE_SNAPSHOT_ROOT.mkdir(parents=True, exist_ok=True)

    write_feedback_note()
    write_scenario_spec()

    legacy_cfg = TrackerConfig(
        name="legacy_synth",
        n_confirm=3,
        hold_max_misses=6,
        lost_max_misses=12,
        t_stale_sec=1.0,
        gate_px=70.0,
        selector_switch_margin=0.05,
        selector_selected_track_bonus=0.05,
    )
    prod_cfg = TrackerConfig(
        name="production_like",
        n_confirm=5,
        hold_max_misses=12,
        lost_max_misses=30,
        t_stale_sec=1.0,
        gate_px=80.0,
        selector_switch_margin=0.05,
        selector_selected_track_bonus=0.05,
    )

    metrics_rows: list[ScenarioMetrics] = []
    case_records: dict[str, list[FrameRecord]] = {}
    video_paths: list[Path] = []

    scenario_a_records = run_tracker_sequence("scenario_a", scenario_a(1234), prod_cfg)[1]
    depths, velocities = build_synthetic_depth_and_velocity(scenario_a_records, "quiet")
    case_records["scenario_a"] = run_controller_harness(scenario_a_records, depths, velocities)
    render_synthetic_video(VIDEO_DIR / "scenario_a.mp4", "Synthetic Scenario A", case_records["scenario_a"], FPS_SYNTH)
    video_paths.append(VIDEO_DIR / "scenario_a.mp4")

    for scenario_name, frames, config, profile in [
        ("scenario_b", scenario_b_legacy(5678), legacy_cfg, "aggressive"),
        ("scenario_b", scenario_b_legacy(5678), prod_cfg, "aggressive"),
        ("variable_bbox_b", scenario_b_variable_bbox(9012), prod_cfg, "aggressive"),
    ]:
        metrics, records = run_tracker_sequence(scenario_name, frames, config)
        metrics_rows.append(metrics)
        depths, velocities = build_synthetic_depth_and_velocity(records, profile)
        case_name = f"{scenario_name}_{config.name}"
        case_records[case_name] = run_controller_harness(records, depths, velocities)
        render_synthetic_video(VIDEO_DIR / f"{case_name}.mp4", case_name, case_records[case_name], FPS_SYNTH)
        video_paths.append(VIDEO_DIR / f"{case_name}.mp4")

    write_parameter_comparison(metrics_rows)
    write_reacquire_definition(metrics_rows)
    write_design_change_gate({(m.scenario, m.tracker_config): m for m in metrics_rows})

    bag_records, bag_images = build_bag_records(prod_cfg)
    if bag_records and bag_images:
        case_records["bag_multi_object_window"] = run_controller_harness(
            bag_records,
            [record.depth_m for record in bag_records],
            [(record.vel_vx, record.vel_vy, record.vel_yaw_rate) for record in bag_records],
        )
        render_bag_video(
            VIDEO_DIR / "bag_multi_object_window.mp4",
            "Bag Multi-Object Window",
            case_records["bag_multi_object_window"],
            bag_images,
            FPS_BAG,
        )
        video_paths.append(VIDEO_DIR / "bag_multi_object_window.mp4")

    for case_name, records in case_records.items():
        plot_case_response(case_name, records)
    plot_parameter_comparison(metrics_rows)
    write_control_response_summary(case_records)
    write_video_manifest(video_paths)

    files = [
        ARTIFACT_ROOT / "feedback_adoption_note.md",
        ARTIFACT_ROOT / "scenario_b_parameter_comparison.md",
        ARTIFACT_ROOT / "scenario_b_parameter_comparison.csv",
        ARTIFACT_ROOT / "synthetic_scenario_spec.md",
        ARTIFACT_ROOT / "reacquire_metric_definition.md",
        ARTIFACT_ROOT / "reacquire_metric_table.csv",
        ARTIFACT_ROOT / "video_manifest.md",
        ARTIFACT_ROOT / "control_response_summary.md",
        ARTIFACT_ROOT / "design_change_gate.md",
        ARTIFACT_ROOT / "design_change_options_table.md",
        ARTIFACT_ROOT / "generate_recheck_artifacts.py",
    ]
    figures = sorted(FIG_DIR.glob("*.png"))
    build_manifest(files, video_paths, figures)
    files.extend([
        ARTIFACT_ROOT / "ARTIFACT_MANIFEST.md",
        ARTIFACT_ROOT / "changed_files_list.md",
    ])
    copy_snapshot(files)


if __name__ == "__main__":
    rclpy.init(args=None)
    try:
        main()
    finally:
        rclpy.shutdown()
