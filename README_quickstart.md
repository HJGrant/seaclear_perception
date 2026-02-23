# SeaClear2 Visual Servo MVP Quickstart

This quickstart runs the ROS2 visual-servo MVP in simulation mode (no SLAM required), with uv-managed Python deps that stay compatible with ROS2 Humble.

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
  sc2_visual_servo_interfaces
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

## 5) MPPI mode (Phase 2 path)

```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py controller_mode:=mppi
```
If MPPI dependencies are not available at runtime, node falls back to `ibvs`.

## 6) Run tests (venv + colcon)

```bash
./scripts/colcon_test_venv.sh --packages-select \
  sc2_visual_servo_perception \
  sc2_visual_servo_controller \
  sc2_visual_servo_bringup
```

## Troubleshooting

- `No module named rclpy._rclpy_pybind11`:
  interpreter mismatch. Recreate venv with Python 3.10 and `--system-site-packages`:
  `rm -rf .venv && ./scripts/uv_bootstrap.sh`
- `cv_bridge` import errors mentioning NumPy 2:
  ensure `numpy<2` is locked and re-sync:
  `uv sync --python .venv/bin/python --group dev`
- Avoid installing `opencv-python` into this venv unless explicitly required; it can conflict with system OpenCV / `cv_bridge` bindings.
