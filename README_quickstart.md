# SeaClear2 Visual Servo MVP Quickstart

Scope: primary run/build/test entry point for the current visual-servo MVP workflow.
Source of truth: use this file for reproducible commands; deeper design details live under `docs/`.
Last updated: 2026-03-23
Supersedes: duplicated command blocks in older status/debug notes.

This quickstart runs the ROS2 visual-servo MVP in simulation mode (no SLAM required), with uv-managed Python deps that stay compatible with ROS2 Humble.
Perception topic map (live OAK-D / bag replay): `docs/PERCEPTION_TOPIC_MAP.md`.

Current integrated launch status on `dev/aoki_detector_integration`:
- integrated entry: `src/oakd_isaac_ros/launch/oakd_system.launch.py`
- detector-only wrapper: `src/oakd_isaac_ros/launch/debug/object_detection.launch.py`
- current detector asset defaults: `v12s.onnx` / `v12s.plan`

## Host-side prerequisites for current `dev/aoki_detector_integration`

Use the supported workflow: clone/update on the host, then do ROS2/CV build/test/run inside the ROS Humble dev container. Host-side Python execution is not the supported path for the MVP checks below.

For a fresh clone:

```bash
git clone --recurse-submodules <repo-url>
cd oakd_isaac_ros
git submodule sync --recursive
git submodule update --init --recursive
```

WSL + Docker Desktop notes for the current LRZ `dev/aoki_detector_integration` branch:
- Start Docker Desktop before running `run_dev.sh`.
- If private submodules or private image access rely on your SSH agent, start `ssh-agent` on the host and run `run_dev.sh` from the same terminal session so `SSH_AUTH_SOCK` is forwarded into the container.
- If your clone is missing unrelated Git LFS payloads but you only need the visual-servo MVP quickstart, use `run_dev.sh --skip_lfs_check`. This only skips the preflight completeness gate; it does not fetch missing LFS assets.
- `SKIP_LFS_CHECK=1` is an environment variable read by `run_dev.sh`. Use it only when you are calling another wrapper that does not expose `--skip_lfs_check` directly.

Example host-side container start:

```bash
cd /path/to/oakd_isaac_ros
cp .isaac_ros_common_config ~/.isaac_ros_common-config
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/<your_key>
./src/isaac_ros_common/scripts/run_dev.sh --skip_lfs_check --isaac_ros_dev_dir "$(pwd)"
```

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
- `.venv` lives under the workspace, so it persists across container recreation as long as the workspace is bind-mounted.
- `uv` is installed into a workspace-local location by `scripts/ensure_uv.sh`, so it also persists across container recreation as long as the workspace is bind-mounted.
- To avoid manually deciding when `uv_bootstrap.sh` is needed, run `./scripts/ensure_dev_container_ready.sh` inside the container. It only bootstraps when `.venv` is missing or broken.
- `uv_bootstrap.sh` is now safe to rerun; it recreates `.venv` deterministically when a stale environment already exists.
- Workspace setup sourcing is wrapped by `scripts/source_workspace_env_safe.sh` to avoid `set -u` / colcon setup-script failures during automated checks.

## 0.1) One-command environment check

Inside the ROS Humble container:

```bash
cd /workspaces/oakd_isaac_ros
./scripts/ensure_dev_container_ready.sh
```

From the host, using Docker:

```bash
cd /path/to/oakd_isaac_ros
SKIP_LFS_CHECK=1 ./scripts/run_oakd_system_check_docker.sh --profile no_hw
```

This host-side wrapper:
- starts or reuses the Isaac ROS dev container,
- runs the container-side bootstrap/build/launch check,
- writes logs under `diagnoses_oakd_system/<timestamp>/`.
- `run_oakd_system_check_docker.sh` itself does not expose `--skip_lfs_check`, so `SKIP_LFS_CHECK=1` is passed via the environment to the underlying `run_dev.sh`.
- Do not run `run_oakd_system_check_docker.sh` inside the container; use `run_oakd_system_check.sh` there instead.

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

## 1.1) Run tests for the MVP packages

```bash
./scripts/colcon_test_venv.sh --packages-select \
  sc2_visual_servo_perception \
  sc2_visual_servo_controller \
  sc2_visual_servo_bringup
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

## 6) Optional: bbox + target-point mode (bbox_depth_roi)

Enable target-point estimation from bbox ROI depth and run controller mode `bbox_target_point`:

```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py \
  use_stub_target:=false \
  controller_mode:=bbox_target_point \
  enable_target_point_estimator:=true \
  depth_topic:=/oak/stereo/image_raw \
  min_valid_points:=20 \
  max_search_radius_px:=40.0
```

Key tuning switches for field validation:
- `min_valid_points` (minimum valid ROI points)
- `max_search_radius_px` (N-point search radius near `(u_med, v_med)`)
- `use_mask_gating` + `mask_topic` (gate ROI by mask image)
- `hold_last_valid` / `max_hold_ms` (temporary dropout handling)
- `require_target_point` (controller hard-stop if target point is missing)

Design note and detailed investigation are documented in:
- `docs/VS_BBOX_TARGET_POINT_INTEGRATION_20260225.md`

Minimal reproducibility check for the current MVP path:
1. Inside the container, run `./scripts/ensure_dev_container_ready.sh`.
2. Build the five `sc2_visual_servo_*` packages with `./scripts/colcon_build_venv.sh --packages-up-to ...`.
3. Run `./scripts/colcon_test_venv.sh --packages-select sc2_visual_servo_perception sc2_visual_servo_controller sc2_visual_servo_bringup`.
4. Launch `ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py use_stub_target:=false`.
5. Publish a centered manual `TargetObservation` and verify `/sc2/visual_servo/cmd_vel_body` is near zero.
6. Publish an offset manual `TargetObservation` and verify the command magnitude increases in the error-reducing direction.

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

## 8.1) `oakd_system.launch.py` smoke checks

WSL / Docker preflight without OAK-D hardware:

```bash
cd /path/to/oakd_isaac_ros
./scripts/run_oakd_system_check_docker.sh --profile no_hw
```

Jetson live check inside the container:

```bash
cd /workspaces/oakd_isaac_ros
./scripts/run_oakd_system_check.sh --profile jetson_live
```

Jetson live check from the host:

```bash
cd /path/to/oakd_isaac_ros
./scripts/run_oakd_system_check_docker.sh --profile jetson_live
```

Generated logs are kept under:

```bash
diagnoses_oakd_system/<timestamp>/
```

For the `jetson_live` profile, also inspect:

```bash
cat diagnoses_oakd_system/<timestamp>/perception_preflight.log
```

This captures:
- existence/type of `isaac_ros_assets/models/seaclear/v12s.onnx` and `v12s.plan`
- detection-related launch files currently present under `src/oakd_isaac_ros/launch/`

Jetson integrated launch example for this branch:

```bash
cd /workspaces/oakd_isaac_ros
ros2 launch oakd_isaac_ros oakd_system.launch.py \
  detector_model_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.onnx \
  detector_engine_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan
```

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
