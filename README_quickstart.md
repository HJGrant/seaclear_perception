# SeaClear2 Visual Servo MVP Quickstart

This quickstart runs the ROS2 visual-servo MVP in simulation mode (no SLAM required), with uv-managed Python deps that stay compatible with ROS2 Humble.
Perception topic map (live OAK-D / bag replay): `docs/PERCEPTION_TOPIC_MAP.md`.

## 0) uv Bootstrap (recommended first)

```bash
cd /workspaces/oakd_isaac_ros
source /opt/ros/humble/setup.bash
./scripts/uv_bootstrap.sh
# Optional MPPI extras on x86_64:
# ./scripts/uv_bootstrap.sh --with-mppi
```

Notes:
- venv is created with Python 3.10 and `--system-site-packages` so ROS apt packages (`rclpy`, `cv_bridge`, message modules) remain importable.
- `numpy<2` is pinned in `pyproject.toml` to avoid common `cv_bridge` ABI issues.
- Torch in `mppi` group is intended for x86_64 development; on Jetson, use NVIDIA-provided torch wheel and skip `--with-mppi`.

## 1) Build using venv Python

```bash
./scripts/colcon_build_venv.sh --packages-up-to \
  sc2_visual_servo_bringup \
  sc2_visual_servo_controller \
  sc2_visual_servo_perception \
  sc2_visual_servo_interfaces \
  sc2_visual_servo_synth
source install/setup.bash
```

## 2) One-command run

```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py
```

Default launch starts:
- scripted target observation publisher
- visual servo controller (`controller_mode:=ibvs`)
- fake vehicle integrator
- CSV logger (`/tmp/sc2_visual_servo_mvp.csv`)
- MAVLink command stub logger

## 3) Repro checks

Target centered -> near-zero command:
```bash
ros2 topic pub /sc2/visual_servo/target_obs sc2_visual_servo_interfaces/msg/TargetObservation \
"{header: {frame_id: 'oak_rgb_camera_optical_frame'}, track_id: 1, u: 640.0, v: 360.0, w: 130.0, h: 90.0, depth_m: 1.0, confidence: 0.95, keypoints_uv: [], source: 'manual'}" --once
ros2 topic echo /sc2/visual_servo/cmd_vel_body --once
```

Target offset -> command reduces image error:
```bash
ros2 topic pub /sc2/visual_servo/target_obs sc2_visual_servo_interfaces/msg/TargetObservation \
"{header: {frame_id: 'oak_rgb_camera_optical_frame'}, track_id: 1, u: 920.0, v: 520.0, w: 90.0, h: 70.0, depth_m: 2.0, confidence: 0.95, keypoints_uv: [], source: 'manual'}" --once
ros2 topic echo /sc2/visual_servo/cmd_vel_body --once
```

Safety stop on timeout:
```bash
# Stop publishing target obs for > timeout_ms (default 500 ms)
ros2 topic echo /sc2/visual_servo/status
```
You should see `NO_TARGET_TIMEOUT` and zero velocity on `/sc2/visual_servo/cmd_vel_body`.

## 4) Optional: run with perception wrapper input

```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py use_stub_target:=false
```
This enables `target_observer_node` consuming:
- `/detections_output` (`vision_msgs/Detection2DArray`)
- `/oak/stereo/image_raw` (depth image)
and publishing `/sc2/visual_servo/target_obs`.

Inspect available perception topics quickly:
```bash
ros2 topic list && ros2 topic info /detections_output && ros2 topic info /oak/stereo/image_raw && ros2 interface show vision_msgs/msg/Detection2DArray
```

For alternative topic names:
```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py \
  use_stub_target:=false \
  detections_topic:=/detections_output \
  depth_topic:=/oak/stereo/image_raw \
  target_class:=bottle \
  min_confidence:=0.4 \
  pick_strategy:=highest_confidence
```

## 5) MPPI mode (Phase 2 path)

```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py controller_mode:=mppi
```
If MPPI dependencies are not available at runtime, node falls back to `ibvs`.

## 6) Optional: bbox + grasp-pose mode (bbox_depth_roi)

Enable grasp-pose estimation from bbox ROI depth and run controller mode `bbox_grasp`:

```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py \
  use_stub_target:=false \
  controller_mode:=bbox_grasp \
  enable_grasp_pose_estimator:=true \
  depth_topic:=/oak/stereo/image_raw \
  min_valid_points:=20 \
  max_search_radius_px:=40.0
```

Key tuning switches for field validation:
- `min_valid_points` (minimum valid ROI points)
- `max_search_radius_px` (N-point search radius near `(u_med, v_med)`)
- `use_mask_gating` + `mask_topic` (gate ROI by mask image)
- `hold_last_valid` / `max_hold_ms` (temporary dropout handling)
- `require_grasp_pose` (controller hard-stop if grasp pose is missing)

Design note and detailed investigation are documented in:
- `docs/VS_BBOX_GRASP_INTEGRATION_20260225.md`

## 7) Run tests (venv + colcon)

```bash
./scripts/colcon_test_venv.sh --packages-select \
  sc2_visual_servo_perception \
  sc2_visual_servo_controller \
  sc2_visual_servo_bringup
```

## 8) Reproducible rosbag workflow (no hardware)

Record perception topics while pipeline is live:
```bash
./scripts/record_perception_bag.sh /tmp/sc2_perception_bag 60
```

Replay the bag:
```bash
./scripts/play_perception_bag.sh /tmp/sc2_perception_bag 1.0
```

In another terminal, run visual-servo with real perception input:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py use_stub_target:=false
```

Observe outputs:
```bash
ros2 topic echo /sc2/visual_servo/target_obs --once
ros2 topic hz /sc2/visual_servo/target_obs
ros2 topic hz /sc2/visual_servo/cmd_vel_body
ros2 topic hz /sc2/visual_servo/debug_markers
ls -l /tmp/sc2_visual_servo_mvp.csv
```

Visualization note:
- `visual_servo_debug_viz_node` is enabled by default in `mvp_sim.launch.py` and publishes:
  - `/sc2/visual_servo/debug_markers` (`visualization_msgs/msg/MarkerArray`)
- This can be viewed in RViz2 or Foxglove.

## 9) BlueROV MAVROS quickcheck (synthetic detections)

For detector-free actuation validation run:

```bash
ros2 launch sc2_visual_servo_bringup blue_rov_synth_vs.launch.py \
  pattern:=fixed_offsets \
  controller_mode:=bbox_approach \
  mavros_backend:=dry_run \
  deadman:=true \
  enable_debug_viz:=true
```

Switch to MAVROS RC override backend:

```bash
ros2 launch sc2_visual_servo_bringup blue_rov_synth_vs.launch.py \
  pattern:=size_ramp \
  controller_mode:=bbox_approach \
  mavros_backend:=rc_override \
  deadman:=true
```

Detailed runbook:
- `docs/BLUEROV_MAVROS_QUICKCHECK.md`

## Known Issues / Deferments

- Detection pipeline deferment:
  - TensorRT/ONNX incompatibilities can prevent a stable detection publisher.
  - In affected runs, `/detections_output` may exist but still have publisher count `0`.
  - Detection topic/remap finalization is deferred pending colleague input on current launch conventions.
- Workaround for controller validation without detections:
  - run `use_stub_target:=true`, or
  - publish `TargetObservation` manually and verify `/sc2/visual_servo/cmd_vel_body`.

## Host vs Container Python note

- NumPy/cv_bridge ABI mismatch was observed when Python/ROS commands were run outside the Humble container.
- Supported workflow: run ROS2 commands and `cv_bridge`-dependent nodes inside the Humble container.
- If host-side Python execution is unavoidable, NumPy alignment with host ROS apt packages may be required, but host Python setup is out of scope for this project workflow.

## Troubleshooting

- `No module named rclpy._rclpy_pybind11`:
  interpreter mismatch. Recreate venv with Python 3.10 and `--system-site-packages`:
  `rm -rf .venv && ./scripts/uv_bootstrap.sh`
- `cv_bridge` import errors mentioning NumPy 2:
  verify you are running inside the Humble container first; host-side Python/cv_bridge mismatch is not part of the supported workflow.
- Avoid installing `opencv-python` into this venv unless explicitly required; it can conflict with system OpenCV / `cv_bridge` bindings.
