# Algorithm As Implemented

## Overview

実装は `/detections_output` を主入力にする track-table based tracker/selector である。`/best_detection` は controller 側に互換 input として残しているが、launch 上の primary path では `TargetObservation` を使う。

精密 camera/TF reprojection は入れていない。P2 は motion-aware gate expansion であり、EKF speed と IMU yaw rate で association gate を広げるだけである。

## Track Table

`target_observer_node` は `self._tracks: Dict[int, TrackState]` を持つ。各 track は以下を持つ。

- `track_id`
- `state`: `TENTATIVE`, `CONFIRMED`, `HOLD`, `LOST`, `ESCALATE`
- bbox center: `u`, `v`
- bbox size: `w`, `h`
- image-plane velocity: `du`, `dv`
- `age_frames`
- `hit_count`
- `hit_streak`
- `miss_count`
- `confidence_ema`
- `freshness`
- `last_match_time_sec`
- `last_match_confidence`
- `last_class_id`
- `reason_code`

`max_tracks` を超える場合は、`LOST` / `ESCALATE`、freshness 低、confidence 低、hit count 低の順で pruning する。

## Per-frame Processing

Pseudo-code:

```text
on Detection2DArray:
  candidates = detections with target_class and score >= low_confidence
  active_tracks = tracks not in LOST or ESCALATE
  gate_px = base_gate_px * motion_gate_scale

  assignments = two_stage_association(active_tracks, candidates, gate_px)

  for assigned track:
    update u/v/w/h, du/dv
    reset miss_count
    increment hit_streak and hit_count
    update confidence_ema
    TENTATIVE -> CONFIRMED if hit_streak >= n_confirm

  for unmatched active track:
    increment miss_count
    hit_streak = 0
    update freshness and state to HOLD / LOST / ESCALATE by lifecycle budget

  for unmatched high-confidence detection:
    create new TENTATIVE track

  selected_track = select over track table
  publish TargetObservation for selected_track
```

## Two-stage Association

Stage 1:

- high-confidence detections only
- greedy assignment over all active tracks
- cost is image-plane Euclidean distance from predicted track center
- pairs outside `gate_px` are rejected

Stage 2:

- only tracks unmatched by Stage 1
- low-confidence detections only
- same greedy distance/gate rule
- reason code becomes `low_confidence_recovery`

New tracks are created only from unmatched high-confidence detections. Low-confidence detections recover existing tracks but do not create new tracks.

## Prediction And Motion-aware Gate

Prediction is conservative.

```text
u_pred = u_last + alpha_uv_predict * du
v_pred = v_last + alpha_uv_predict * dv
```

Default `alpha_uv_predict = 0.0`, so the default is last-observation hold.

Gate expansion:

```text
s_v = clamp(norm(v_ekf) / v_ref, 0, 1)
s_omega = clamp(abs(omega_z) / omega_ref, 0, 1)
scale = min(r_gate_max_scale, 1 + beta_v * s_v + beta_omega * s_omega)
gate_px = base_gate_px * scale
```

This is not precise ego-motion compensation.

## Freshness

Freshness is:

```text
freshness = clamp(1 - (t_now - t_last_match) / T_stale, 0, 1)
```

`age_frames` is persistence. Freshness is wall-clock staleness from last matched detection. They are not treated as the same quantity.

## State Transition

```text
new high-confidence detection -> TENTATIVE
TENTATIVE + matched for N_confirm consecutive frames -> CONFIRMED
CONFIRMED + no match and hold miss budget remains -> HOLD
HOLD + matched -> CONFIRMED
HOLD + miss_count > hold_max_misses -> LOST
any active track + miss_count >= lost_max_misses -> ESCALATE
```

`ESCALATE` remains observer-side output. Manual assist / reposition / abort remains an upper-state-machine responsibility.

## Selection

Selection is separate from association. After all track updates, the selector ranks tracks by:

1. `CONFIRMED` with freshness > 0
2. `HOLD` with freshness > 0
3. `TENTATIVE`
4. freshness
5. `confidence_ema`
6. `hit_count`

Only `CONFIRMED` with freshness > 0 publishes `selected_valid=true`. `HOLD` can still be selected for continuity, but publishes `selected_valid=false`.

Task A/B update: selection now includes previous-selected continuity hysteresis. The base score is `10 * state_score + freshness + confidence_ema + min(hit_count, 100) * 0.001`. If the previous selected track is still `CONFIRMED` and fresh, `selector_selected_track_bonus` is added and another track must exceed that effective score by `selector_switch_margin` before selection switches.

`REACQUIRE` is not a separate state. `HOLD` passively serves as `REACQUIRE`: it remains in active association and can return to `CONFIRMED` through high-confidence match or low-confidence recovery. `LOST` / `ESCALATE` tracks are excluded and any later detection starts a new track id.

## Controller HOLD Maintain

When `use_target_observation=true`, controller staged PID treats `TargetObservation` as the primary gate.

If `selected_valid=true`:

- normal staged PID visual correction is allowed.

If `selected_valid=false` or stale:

- visual horizontal correction is stopped.
- `/best_detection` is not used as a replacement primary path.
- controller computes HOLD maintain through existing feedback:
  - body velocity measurement from `vel_measurement_topic` (`/mavros/local_position/velocity_body` by default) damps XY and yaw drift
  - `_resolve_staged_pid_vertical_measurement()`
  - DVL path via `/waterlinked_dvl_driver/velocity_report` when `staged_pid_z_source=dvl`
  - existing `_z_pid`
- command is `(vx=velocity-damping, vy=velocity-damping, vz=depth-maintain, yaw=velocity-damping)` when both feedback paths are fresh.

If DVL/altitude is not fresh, HOLD maintain falls back to zero vertical command but can still publish XY/yaw velocity damping when velocity feedback is fresh. If velocity feedback is also stale, the command becomes zero and the status reports the missing feedback.

Remaining limitation:

- This is depth maintain plus velocity damping, not full 6-DoF pose hold.
- There is no localization setpoint based horizontal station-keeping controller in this repo path.
- If DVL/altitude and body velocity are stale, maintain falls back to zero command.
