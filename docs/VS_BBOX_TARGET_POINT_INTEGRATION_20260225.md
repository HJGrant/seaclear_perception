# Visual Servo BBox+Target-Point Integration (2026-02-25)

Scope: active design note for bbox+depth ROI to target-point integration in the MVP stack.
Source of truth: use this file for current target-point input assumptions, tunable parameters, and integration boundaries.
Last updated: 2026-03-13
Supersedes: exploratory implementation notes previously mixed into status/debug documents.

## Scope

This note records:
- investigation findings for bbox/depth/mask wiring in this repository
- implementation added for visual-servo target-point input
- compatibility constraints and tunable parameters for field validation

## Investigation Summary

### What is active in current `oakd_nvblox.launch.py` path

- Detection pipeline includes `publish_mask` and mask resizing:
  - `src/oakd_isaac_ros/launch/tools/detection.launch.py`
  - `src/oakd_isaac_ros/src/publish_mask.cpp`
- `publish_mask.cpp` subscribes `/detections_output` and publishes mask/overlay:
  - `/camera0/detection/mask`
  - `/camera0/detection/mask/camera_info`
  - `/mask_overlay`
- nvblox consumes resized mask via remap:
  - `camera_0/mask/image <- /camera0/detection/mask_resized`
  - `src/oakd_isaac_ros/launch/tools/nvblox.launch.py`

### Important clarification

`maskFromDetections(...)` exists in nvblox core:
- `src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/semantics/mask_from_detections.cu`

However, the current OAK-D launch path relies on `publish_mask.cpp` mask topic wiring.
For this reason, target-point estimation is implemented with a robust bbox+depth ROI path (`bbox_depth_roi`) as the primary source mode.

Important naming clarification:
- The D5.2 "grasp pose" / gripping-points logic belongs to the final smart-grapple grasping stage.
- The node described here does not produce that final grasp pose.
- It produces a camera-frame representative target point for visual-servo approach control.

## Implemented Components

### 1) Target-point estimation node (new)

- File:
  - `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_point_estimator_node.py`
- Role:
  - Input: `TargetObservation` + depth + camera_info (+ optional mask)
  - Output: `/sc2/visual_servo/target_point` (`geometry_msgs/PoseStamped`)
- Source mode:
  - `bbox_depth_roi` (implemented and default)
  - `nvblox_dynamic_points` (reserved for future; currently falls back to `bbox_depth_roi` with warning)

### 2) ROI representative depth algorithm (new)

- File:
  - `src/sc2_visual_servo_perception/sc2_visual_servo_perception/conversion.py`
- Function:
  - `compute_representative_depth_near_uv_median(...)`
- Method:
  1. Build valid ROI points inside bbox.
  2. Compute `(u_med, v_med)` from valid ROI pixels.
  3. Select nearest `N` points around `(u_med, v_med)` under radius gate.
  4. Use median depth of selected points as representative depth.
- Reason:
  - Avoid unstable global ROI median when bbox includes broad background.

### 3) Visual-servo controller mode (extended)

- File:
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node.py`
- New mode:
  - `controller_mode:=bbox_target_point`
- Behavior:
  - Keep bbox centering behavior for `vy/vz/yaw`.
  - Use target-point distance for `vx` approach control.
  - On missing target point:
    - fallback to bbox-only behavior, or
    - stop if `require_target_point:=true`.

### 4) Launch integration (extended, backward-compatible)

- File:
  - `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py`
- Added flags:
  - `enable_target_point_estimator` (default `false`)
  - `target_point_topic`, `target_point_status_topic`
  - `min_valid_points`, `num_nearest_points`, `max_search_radius_px`
  - `use_mask_gating`, `mask_topic`
  - `hold_last_valid`, `max_hold_ms`
  - `controller_target_point_timeout_ms`, `k_target_distance`, `desired_target_distance_m`, `require_target_point`

Default behavior remains unchanged when `enable_target_point_estimator:=false`.

## Field-Tuning Parameters (On/Off and Safety)

- Robustness switches:
  - `min_valid_points`
  - `max_search_radius_px`
  - `num_nearest_points`
  - `use_uv_median_depth`
- Mask gating:
  - `use_mask_gating`
  - `mask_topic`
  - `mask_threshold`
- Dropout handling:
  - `hold_last_valid`
  - `max_hold_ms`
- Controller safety:
  - `require_target_point`
  - `controller_target_point_timeout_ms`
  - existing velocity saturation/slew parameters

## Quick Run

```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py \
  use_stub_target:=false \
  controller_mode:=bbox_target_point \
  enable_target_point_estimator:=true \
  depth_topic:=/oak/stereo/image_raw \
  min_valid_points:=20 \
  max_search_radius_px:=40.0
```

## Test Coverage Added

- Conversion unit tests:
  - `src/sc2_visual_servo_perception/test/test_conversion.py`
- Target observer integration test:
  - `src/sc2_visual_servo_perception/test/test_target_observer_integration.py`
- Target-point estimator integration test:
  - `src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py`
- Explicit pytest execution list updated:
  - `scripts/colcon_test_venv.sh` runs controller-sign, conversion, target-observer, and target-point-estimator checks after `colcon test`

## Future Filter Note (not implemented in this change)

- **LPF/hysteresis**:
  - simple, robust, low tuning burden
  - no explicit motion/noise model

- **Kalman filter**:
  - can outperform LPF when target motion and sensor noise are reasonably modeled
  - gives predictive smoothing and uncertainty handling
  - requires stronger tuning and model consistency

**Implementation steps:**
1. Keep current spatially robust ROI estimator and configurable hold/fallback.
2. Add optional temporal LPF/hysteresis first.
3. Add an optional Kalman mode after collecting real logs for noise/model identification.
