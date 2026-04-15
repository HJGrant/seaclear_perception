# Perception Filtering Update

Date: 2026-04-15

## Scope

This update improves the current default experiment path:

```bash
ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_ibvs:=true launch_foxglove:=false
```

It does not enable nvblox in the default path.

## Added Parameters

| parameter | default | purpose |
| --- | ---: | --- |
| `max_depth_age_ms` | `300.0` | rejects stale depth images independently of target bbox age |
| `enable_temporal_filter` | `True` | enables EMA smoothing and jump rejection |
| `filter_alpha_uv` | `0.4` | EMA coefficient for bbox center and representative UV |
| `filter_alpha_depth` | `0.3` | EMA coefficient for representative range |
| `filter_alpha_size` | `0.3` | EMA coefficient for bbox width/height |
| `max_uv_jump_px` | `80.0` | large bbox/representative UV jump threshold |
| `max_depth_jump_m` | `0.5` | large representative depth jump threshold |
| `reject_large_jumps` | `True` | rejects jumps and holds last valid pose when possible |

The launch file now passes these parameters explicitly to `target_point_estimator_node`.

## Processing Algorithm

```text
on depth image:
    convert to cv image
    store image and receive time

on target Detection2D:
    if detection stamp is older than max_obs_age_ms:
        status = STALE_TARGET_OBS
        return

    if no depth image:
        status = NO_DEPTH
        reset filters
        return

    if stored depth receive time is older than max_depth_age_ms:
        if last valid pose is within max_hold_ms:
            publish held pose, status = HOLD_LAST_VALID_DEPTH_STALE
        else:
            status = STALE_DEPTH
            reset filters
        return

    EMA-filter bbox center/size
    if bbox jump exceeds max_uv_jump_px:
        hold last valid pose if possible, status = REJECTED_LARGE_JUMP
        return

    compute representative local ROI depth near median UV
    if no valid representative depth:
        hold last valid pose if possible, status = HOLD_LAST_VALID
        otherwise status = NO_VALID_TARGET_POINT and reset filters
        return

    EMA-filter representative (u, v, depth)
    if representative UV or depth jump is too large:
        hold last valid pose if possible, status = REJECTED_LARGE_JUMP
        return

    publish PoseStamped with position.z = target-relative range
    status = TRACKING_TARGET_POINT
```

## Hold / Stale / Reset Rules

| condition | output |
| --- | --- |
| short invalid depth gap within `max_hold_ms` | publish last valid pose, `HOLD_LAST_VALID` |
| stale depth within `max_hold_ms` | publish last valid pose, `HOLD_LAST_VALID_DEPTH_STALE` |
| long invalid/stale gap | do not publish a new pose; reset filters |
| large bbox or depth jump | publish last valid pose if available; otherwise suppress new pose |

Filter state is reset when the estimator can no longer produce or hold a reliable target point.

## Test Coverage Added

`src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py` now covers:

- field semantics: `position.z` is target-relative range
- temporal filter smoothing of pixel jitter into metric target point output
- large depth jump rejection
- short-gap hold
- long-gap invalidation

ROS message packages are not available on this host, so pytest collection cannot complete here. Syntax checking passed, and these tests are ready for the ROS container.
