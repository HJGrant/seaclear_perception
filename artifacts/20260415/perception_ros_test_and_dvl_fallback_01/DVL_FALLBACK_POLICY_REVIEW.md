# DVL Fallback Policy Review

Date: 2026-04-15

## Current Controller Behavior

Current staged PID vertical source setting:

```text
staged_pid_z_source = target_point_primary_dvl_fallback
```

Measurement resolution:

```text
if target_point is fresh:
    z_meas = target_point.pose.position.z
    z_source = target_point
elif DVL altitude is fresh:
    z_meas = abs(DVL altitude)
    z_source = dvl_fallback
else:
    z_meas = None
    z_source = none
```

## Fallback Entry / Exit

| item | current behavior |
| --- | --- |
| fallback entry | target_point stale/unavailable and DVL fresh |
| fallback exit to primary | next fresh target_point immediately becomes primary |
| fallback unavailable | no target_point and no fresh DVL -> `z_fresh=False`, `vz=0` |

## Conditions Still Required During Fallback

DVL fallback does not bypass the staged PID visual gate. `allow_z` still requires:

```text
vertical_enabled
horizontal_aligned_now
z_fresh
z_meas is not None
```

In `_tick_staged_pid`, vertical command generation is only reached while:

```text
target_tracked == True
fresh detection exists
latest_centroid exists
```

If detection is stale but `target_tracked` has not exited yet, the controller uses horizontal stale decay and does not compute z.

## Current Fallback Command Strength

If fallback is active and `allow_z=True`, the controller computes:

```text
err_z = z_meas - desired_vertical_distance_m
vz_raw = z_pid.update(err_z, dt)
vz = horizontal_ok_ratio * vz_raw
```

There is no extra DVL-specific scale-down today. The same z PID gains and clamps apply.

## Terminal Distance Hook

The current Phase 2 terminal hook also observes fallback `z_meas`:

```text
if z_meas <= staged_pid_hold_zero_distance_m:
    status = TRACKING_STAGED_PID_TARGET_DISTANCE_REACHED_<SOURCE>
```

This is status-only in current grapple Phase 2; it does not enter a hold-zero state or override command output.

## Risk

DVL altitude is floor/terrain-relative, not target-relative. It may be a useful continuity/safety measurement but is not semantically equivalent to `target_point.pose.position.z`. Therefore using DVL fallback with full z PID authority can command vertical motion based on a value that is not the target distance.

The current gate requirements reduce this risk, but they do not remove the semantic mismatch.

## Policy Options

| option | behavior | safety | target-relative validity | cost | risk |
| --- | --- | --- | --- | --- | --- |
| A. current | fallback DVL can drive normal z PID if visual gate is active | medium | weak during fallback | none | DVL/floor distance can be mistaken for target distance |
| B. conservative | require current gates, scale fallback `vz`, disable terminal-distance hook for fallback or keep status-only | high | medium, because fallback is explicitly degraded | low | needs parameter and compare updates |
| C. ultra-conservative | DVL fallback never drives `vz`; DVL is status/safety only | highest | strong, because only target_point drives target-relative z | low | can lose vertical continuity during short target_point gaps |

## Recommendation

Adopt B for the next controller implementation step, but do not change runtime behavior in this test-focused commit.

Recommended B details:

```text
staged_pid_dvl_fallback_vz_scale = 0.25 to 0.5
fallback still requires target_tracked, fresh detection, horizontal_aligned_now, vertical_enabled
fallback terminal distance should not assert target-distance reached; use source-specific status only
target_point recovery immediately restores primary authority
```

Rationale:

- A is too strong for hardware because DVL is not target-relative.
- C is safest but discards a potentially useful short-gap continuity signal.
- B is a pragmatic next step: it preserves the current fallback mechanism but makes its lower semantic quality explicit.

The implementation should be paired with compare cases for fallback scale, terminal hook suppression during fallback, and primary recovery.
