# Intended vs Implemented Diff

## P1 Tracker / Selector

Intended:

- multiple track table
- association over track x detection pairs
- selection over tracks, not direct detection selection

Implemented:

- `target_observer_node` now has `self._tracks: Dict[int, TrackState]`.
- association computes track x detection distances and uses deterministic greedy assignment.
- selection happens after track update using track state, freshness, confidence EMA, and hit count.

Remaining simplification:

- Hungarian assignment is not implemented. Greedy assignment is explicit and deterministic.
- Track appearance features are not used. Association is image-plane distance only.

## P2 Motion-aware Gate Expansion

Intended:

- use EKF/IMU/DVL only as gate/hold auxiliary signal
- no precise underwater camera reprojection

Implemented:

- EKF odometry linear speed and IMU yaw rate expand the image-plane gate.
- Default prediction is last-observation hold with `alpha_uv_predict=0.0`.

Remaining simplification:

- DVL velocity is not directly used in observer gate expansion. Controller uses DVL for vertical maintain.
- No camera/base TF is consumed.

## P3 Two-stage Association

Intended:

- high-confidence association first
- low-confidence recovery for unmatched tracks

Implemented:

- Stage 1 assigns high-confidence detections to tracks.
- Stage 2 assigns low-confidence detections only to tracks unmatched in Stage 1.
- Low-confidence detections do not start new tracks.

Remaining simplification:

- No ByteTrack score fusion or advanced lifecycle management.
- Low-confidence threshold is a provisional parameter.

## P4 HOLD Gate

Intended:

- `selected_valid=false` stops aggressive visual correction.
- HOLD should be closer to pose/depth maintain than zero command.

Implemented:

- `selected_valid=false` stops horizontal visual correction.
- Controller does not fall back to `/best_detection` as primary.
- Controller uses existing feedback paths to maintain depth/distance and damp body-frame drift:
  - body velocity measurement from `vel_measurement_topic` (`/mavros/local_position/velocity_body` default)
  - DVL altitude from `/waterlinked_dvl_driver/velocity_report`
  - `_resolve_staged_pid_vertical_measurement()`
  - existing `_z_pid`
- HOLD command is not all zero when depth or velocity feedback is fresh:

```text
vx = -hold_maintain_xy_damping * measured_body_vx
vy = -hold_maintain_xy_damping * measured_body_vy
vz = z PID maintain output scaled by hold_maintain_z_kp_scale
yaw = -hold_maintain_yaw_damping * measured_body_yaw_rate
```

Remaining limitation:

- This is depth maintain plus velocity damping, not full 6-DoF pose hold.
- There is no localization setpoint based horizontal station-keeping controller in this repo path.
- If DVL/altitude is stale, maintain falls back to zero vertical command.
- If body velocity feedback is stale, XY/yaw damping is disabled and those axes command zero.

## Design Intent Not Implemented

- P5 target_point / geometry refinement reconnection.
- underwater precise camera calibration or reprojection.
- detector retraining or `/best_detection` forensic.
- full pose hold using localization setpoints.

## Task A/B Addendum

- Selector continuity は Task A/B fixes で bonus + switch margin 方式に更新した。
- `selector_selected_track_bonus` を previous selected confirmed/fresh track に加える。
- 他候補へ切り替えるのは `base_score(other) > base_score(previous) + bonus + selector_switch_margin` の場合のみ。
- 明示的 `REACQUIRE` state は未導入であり、現実装では `HOLD` が passive reacquire を兼ねる。
- `HOLD` 中は low-confidence recovery / miss-budget 内再マッチで `CONFIRMED` に戻れる。
- `LOST` / `ESCALATE` 後の再取得は new track として扱うため、track id continuity はそこで失われる。

## Why These Simplifications Were Kept

- Bag forensic showed `/tf` is 0 messages and camera/base/DVL TF is not confirmed.
- noimage bag showed `/detections_output`, `/best_detection`, odom, IMU, and DVL exist, but `target_point` remains 0.
- Therefore the implemented path avoids unverified precise geometry and uses observed 2D detections, DVL/altitude vertical maintain, and body velocity damping.
