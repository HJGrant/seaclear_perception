# Visual Servo BBox+Grasp Integration (2026-02-25)

## Scope

This note records:
- investigation findings for bbox/depth/mask wiring in this repository
- implementation added for visual-servo grasp-pose input
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
For this reason, grasp-pose estimation is implemented with a robust bbox+depth ROI path (`bbox_depth_roi`) as the primary source mode.

## Implemented Components

### 1) Grasp pose estimation node (new)

- File:
  - `src/sc2_visual_servo_perception/sc2_visual_servo_perception/grasp_pose_estimator_node.py`
- Role:
  - Input: `TargetObservation` + depth + camera_info (+ optional mask)
  - Output: `/sc2/visual_servo/grasp_pose` (`geometry_msgs/PoseStamped`)
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
  - `controller_mode:=bbox_grasp`
- Behavior:
  - Keep bbox centering behavior for `vy/vz/yaw`.
  - Use grasp-pose distance for `vx` approach control.
  - On missing grasp pose:
    - fallback to bbox-only behavior, or
    - stop if `require_grasp_pose:=true`.

### 4) Launch integration (extended, backward-compatible)

- File:
  - `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py`
- Added flags:
  - `enable_grasp_pose_estimator` (default `false`)
  - `grasp_pose_topic`, `grasp_status_topic`
  - `min_valid_points`, `num_nearest_points`, `max_search_radius_px`
  - `use_mask_gating`, `mask_topic`
  - `hold_last_valid`, `max_hold_ms`
  - `controller_grasp_timeout_ms`, `k_grasp_distance`, `desired_grasp_distance_m`, `require_grasp_pose`

Default behavior remains unchanged when `enable_grasp_pose_estimator:=false`.

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
  - `require_grasp_pose`
  - `controller_grasp_timeout_ms`
  - existing velocity saturation/slew parameters

## Quick Run

```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py \
  use_stub_target:=false \
  controller_mode:=bbox_grasp \
  enable_grasp_pose_estimator:=true \
  depth_topic:=/oak/stereo/image_raw \
  min_valid_points:=20 \
  max_search_radius_px:=40.0
```

## Test Coverage Added

- Conversion unit tests:
  - `src/sc2_visual_servo_perception/test/test_conversion.py`
- Grasp estimator integration test:
  - `src/sc2_visual_servo_perception/test/test_grasp_pose_estimator_integration.py`
- Explicit pytest execution list updated:
  - `scripts/colcon_test_venv.sh`

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
