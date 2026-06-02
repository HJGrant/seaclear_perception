from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt

from sc2_visual_servo_perception.tracker_selector_core import (
    DetectionCandidate,
    TrackState,
    TRACK_ESCALATE,
    TRACK_HOLD,
    TRACK_LOST,
    associate_two_stage,
    create_track,
    select_track,
    update_track_with_match,
    update_track_with_miss,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_ROOT = REPO_ROOT / "artifacts/20260423/wsl_only_dvl_and_tracker_validation"
FIG_DIR = ARTIFACT_ROOT / "figures"

HIGH_CONF = 0.8
LOW_CONF = 0.25


@dataclass
class ScenarioMetrics:
    name: str
    selected_switches: int
    selected_false_count: int
    selected_false_total_frames: int
    reacquire_successes: int
    reacquire_attempts: int
    clutter_false_selects: int
    hold_transitions: int
    lost_transitions: int
    escalate_transitions: int


def cand(u: float, v: float, score: float, cls_id: str = "3") -> DetectionCandidate:
    return DetectionCandidate(
        detection=None,
        cls_id=cls_id,
        score=float(score),
        u=float(u),
        v=float(v),
        w=42.0,
        h=36.0,
        high_confidence=score >= 0.5,
    )


def run_sequence(name: str, frames: list[list[tuple[str, DetectionCandidate]]]) -> tuple[ScenarioMetrics, dict]:
    tracks: dict[int, TrackState] = {}
    selected_track_id = None
    next_track_id = 1
    series = {
        "selected_id": [],
        "selected_valid": [],
        "track_state": [],
        "num_det": [],
        "selected_u": [],
        "selected_v": [],
    }
    switches = 0
    selected_false_count = 0
    selected_false_total = 0
    reacquire_attempts = 0
    reacquire_successes = 0
    clutter_false_selects = 0
    hold_transitions = 0
    lost_transitions = 0
    escalate_transitions = 0
    in_false_span = False
    target_visible_after_gap = False
    prev_track_states: dict[int, str] = {}

    for idx, tagged in enumerate(frames):
        now_sec = idx * 0.1
        candidates = [c for _tag, c in tagged]
        tags = [tag for tag, _c in tagged]
        active_tracks = [t for t in tracks.values() if t.state not in {TRACK_LOST, TRACK_ESCALATE}]
        assignments, unmatched = associate_two_stage(active_tracks, candidates, gate_px=70.0, alpha_uv_predict=0.0)
        matched_track_ids = set(assignments.keys())
        for track_id, (candidate, reason) in assignments.items():
            update_track_with_match(
                tracks[track_id],
                candidate,
                now_sec,
                n_confirm=3,
                confidence_ema_alpha=0.35,
                reason_code=reason,
            )
        for track_id, track in list(tracks.items()):
            if track_id in matched_track_ids:
                continue
            update_track_with_miss(
                track,
                now_sec,
                t_stale_sec=1.0,
                hold_max_misses=6,
                lost_max_misses=12,
            )
        for cand_idx in unmatched:
            c = candidates[cand_idx]
            if not c.high_confidence:
                continue
            tracks[next_track_id] = create_track(next_track_id, c, now_sec)
            next_track_id += 1

        selected = select_track(
            tracks.values(),
            now_sec,
            1.0,
            previous_selected_track_id=selected_track_id,
            selector_switch_margin=0.05,
            selector_selected_track_bonus=0.05,
        )
        prev_selected = selected_track_id
        selected_track_id = selected.track_id if selected is not None else None

        selected_valid = bool(selected is not None and selected.state == "CONFIRMED")
        selected_state = selected.state if selected is not None else "NONE"
        series["selected_id"].append(selected_track_id if selected_track_id is not None else -1)
        series["selected_valid"].append(1 if selected_valid else 0)
        series["track_state"].append(selected_state)
        series["num_det"].append(len(candidates))
        series["selected_u"].append(selected.u if selected is not None else math.nan)
        series["selected_v"].append(selected.v if selected is not None else math.nan)

        if prev_selected is not None and selected_track_id is not None and prev_selected != selected_track_id:
            switches += 1

        if not selected_valid:
            selected_false_total += 1
            if not in_false_span:
                selected_false_count += 1
                in_false_span = True
                reacquire_attempts += 1
                target_visible_after_gap = False
        else:
            if in_false_span:
                if target_visible_after_gap:
                    reacquire_successes += 1
                in_false_span = False

        if "target" in tags and not selected_valid and in_false_span:
            target_visible_after_gap = True

        if selected_valid and "target" not in tags and any(tag == "clutter" for tag in tags):
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

    metrics = ScenarioMetrics(
        name=name,
        selected_switches=switches,
        selected_false_count=selected_false_count,
        selected_false_total_frames=selected_false_total,
        reacquire_successes=reacquire_successes,
        reacquire_attempts=reacquire_attempts,
        clutter_false_selects=clutter_false_selects,
        hold_transitions=hold_transitions,
        lost_transitions=lost_transitions,
        escalate_transitions=escalate_transitions,
    )
    return metrics, series


def scenario_a(seed: int) -> list[list[tuple[str, DetectionCandidate]]]:
    rng = random.Random(seed)
    frames = []
    base_u = 320.0
    base_v = 240.0
    occlusions = {20, 21, 22, 45, 46, 70, 71, 72, 73, 95, 96}
    for i in range(120):
        dets: list[tuple[str, DetectionCandidate]] = []
        if i not in occlusions:
            du = 10.0 * math.sin(i * 0.17) + rng.uniform(-3.0, 3.0)
            dv = 8.0 * math.cos(i * 0.11) + rng.uniform(-2.0, 2.0)
            dets.append(("target", cand(base_u + du, base_v + dv, HIGH_CONF)))
        if i % 9 == 0 or i % 17 == 0:
            dets.append(
                (
                    "competing",
                    cand(
                        base_u + rng.uniform(-45.0, 45.0),
                        base_v + rng.uniform(-35.0, 35.0),
                        HIGH_CONF if i % 18 == 0 else LOW_CONF,
                    ),
                )
            )
        frames.append(dets)
    return frames


def scenario_b(seed: int, clutter_density: int = 3) -> list[list[tuple[str, DetectionCandidate]]]:
    rng = random.Random(seed)
    frames = []
    for i in range(120):
        dets: list[tuple[str, DetectionCandidate]] = []
        if i % 2 == 0:
            dets.append(
                (
                    "target",
                    cand(
                        160.0 + 40.0 * math.sin(i * 0.08) + rng.uniform(-4.0, 4.0),
                        300.0 + 20.0 * math.cos(i * 0.14) + rng.uniform(-4.0, 4.0),
                        HIGH_CONF,
                    ),
                )
            )
        for _ in range(clutter_density):
            cls_id = "3" if rng.random() < 0.6 else "7"
            dets.append(
                (
                    "clutter",
                    cand(
                        rng.uniform(0.0, 640.0),
                        rng.uniform(0.0, 640.0),
                        HIGH_CONF if rng.random() < 0.25 else LOW_CONF,
                        cls_id=cls_id,
                    ),
                )
            )
        if i % 11 == 0:
            dets.append(("clutter", cand(170.0 + rng.uniform(-20.0, 20.0), 300.0 + rng.uniform(-20.0, 20.0), HIGH_CONF)))
        frames.append(dets)
    return frames


def plot_series(name: str, series: dict) -> None:
    x = list(range(len(series["selected_id"])))
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(x, series["num_det"])
    axes[0].set_ylabel("detections")
    axes[1].plot(x, series["selected_id"])
    axes[1].set_ylabel("selected id")
    axes[2].plot(x, series["selected_valid"])
    axes[2].set_ylabel("valid")
    state_map = {"NONE": -1, "TENTATIVE": 0, "CONFIRMED": 1, "HOLD": 2, "LOST": 3, "ESCALATE": 4}
    axes[3].plot(x, [state_map.get(v, -1) for v in series["track_state"]])
    axes[3].set_ylabel("state")
    axes[3].set_xlabel("frame")
    fig.suptitle(f"selector timeline: {name}")
    fig.tight_layout()
    fig.savefig(FIG_DIR / f"{name}_timeline.png", dpi=160)
    plt.close(fig)

    plt.figure(figsize=(5, 5))
    plt.plot(series["selected_u"], series["selected_v"], marker="o", markersize=2, linewidth=0.8)
    plt.xlim(0, 640)
    plt.ylim(640, 0)
    plt.xlabel("u [px]")
    plt.ylabel("v [px]")
    plt.title(f"selected bbox trajectory: {name}")
    plt.tight_layout()
    plt.savefig(FIG_DIR / f"{name}_trajectory.png", dpi=160)
    plt.close()


def write_results(metrics_list: list[ScenarioMetrics]) -> None:
    lines = [
        "# Synthetic Results",
        "",
        "production validation ではなく algorithm-level validation として実施した。",
        "",
        "| scenario | switches | false spans | false frames | reacquire success / attempts | clutter false selects | hold transitions | lost transitions | escalate transitions |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for m in metrics_list:
        lines.append(
            f"| `{m.name}` | {m.selected_switches} | {m.selected_false_count} | "
            f"{m.selected_false_total_frames} | {m.reacquire_successes}/{m.reacquire_attempts} | "
            f"{m.clutter_false_selects} | {m.hold_transitions} | {m.lost_transitions} | {m.escalate_transitions} |"
        )
    (ARTIFACT_ROOT / "synthetic_results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_plan() -> None:
    text = """# Synthetic Validation Plan

- seed 固定の deterministic generator を使う
- Scenario A: 近接・再出現 target + 低頻度 competing candidate
- Scenario B: random clutter + target 近傍 clutter
- 評価指標:
  - `selected_track_id` switch count
  - `selected_valid=false` spans / frames
  - reacquire success rate
  - clutter false select count
  - HOLD / LOST / ESCALATE accumulation
"""
    (ARTIFACT_ROOT / "synthetic_validation_plan.md").write_text(text, encoding="utf-8")


def main() -> None:
    ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    write_plan()

    scenarios = {
        "scenario_a": scenario_a(1234),
        "scenario_b": scenario_b(5678, clutter_density=4),
    }
    metrics = []
    for name, frames in scenarios.items():
        m, series = run_sequence(name, frames)
        metrics.append(m)
        plot_series(name, series)
    write_results(metrics)


if __name__ == "__main__":
    main()
