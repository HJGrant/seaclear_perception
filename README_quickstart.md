# SeaClear2 Visual Servo MVP Quickstart

Scope: primary run/build/test entry point for the current visual-servo MVP workflow.
Source of truth: use this file for reproducible commands; deeper design details live under `docs/`.
Last updated: 2026-03-24
Supersedes: duplicated command blocks in older status/debug notes.

This quickstart runs the ROS2 visual-servo MVP in simulation mode (no SLAM required), with uv-managed Python deps that stay compatible with ROS2 Humble.
Perception topic map (live OAK-D / bag replay): `docs/PERCEPTION_TOPIC_MAP.md`.

Current integrated launch status on `dev/aoki_detector_integration`:
- integrated entry: `src/oakd_isaac_ros/launch/oakd_system.launch.py`
- detector-only wrapper: `src/oakd_isaac_ros/launch/debug/object_detection.launch.py`
- current detector model default: `v12s.onnx`
- current detector engine output default: `v12s.plan` (typically generated on first launch per hardware target)
- end-to-end OAK-D detector -> target observer -> controller pipeline is launched from `oakd_system.launch.py`
- `object_detection.launch.py` is the detector-only wrapper for camera + detector debugging, not the full VS/MAVROS pipeline
- integrated `oakd_system.launch.py` now reuses `src/oakd_isaac_ros/launch/debug/object_detection.launch.py` for camera + detector wiring, but overrides the camera params to `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml` because the full path needs both RGB detection input and stereo depth / camera info
- detector-only `object_detection.launch.py` still defaults to `src/oakd_isaac_ros/config/oak_d/oakd_dnn_depth.yaml` for a lighter camera+detector check

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
- `my_ws/scripts/launch_container.sh` does not expose `--skip_lfs_check` directly. If the Jetson host clone still fails the repo-wide LFS completeness preflight, start it as `SKIP_LFS_CHECK=1 ./my_ws/scripts/launch_container.sh` and then verify the specific detector assets you actually need inside the container.
- Jetson note from the latest `dnn_stereo_depth` validation: the full workspace built end-to-end inside the container on Jetson, but some container-side whole-workspace builds can fail in Google Test discovery/configuration. If that happens, retry with `colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install`.
- Jetson note for March 2026 container builds: newer NVIDIA apt repositories expose `libcudnn9` as versioned packages such as `libcudnn9-cuda-12`. If the container build fails in `Dockerfile.aarch64` with `E: Package 'libcudnn9' has no installation candidate`, update to a revision that installs `libcudnn9-cuda-12` instead of the unversioned virtual package.

Example host-side container start:

```bash
cd /path/to/oakd_isaac_ros
cp .isaac_ros_common_config ~/.isaac_ros_common-config
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/<your_key>
./src/isaac_ros_common/scripts/run_dev.sh --skip_lfs_check --isaac_ros_dev_dir "$(pwd)"
```

## Operational assumptions to keep in mind

These are not all obvious from the launch command, but the current integrated path assumes them.

- `oakd_system.launch.py` is the end-to-end entry for OAK-D detections feeding visual servo and optional MAVROS bridge.
- `object_detection.launch.py` is detector-only. Use it to validate camera + detector wiring, not the full VS pipeline.
- Default integrated topics are:
  - detections: `/detections_output`
  - depth: `/oak/stereo/image_raw`
  - camera info: `/oak/stereo/camera_info`
  - mask: `/camera0/detection/mask`
- Default control mode in the integrated launch is `bbox_target_point`, not plain `ibvs`.
- The controller assumes a 1280x720 image with principal target at `(640, 360)` unless you override the corresponding parameters.
- The target-point estimator assumes `bbox_depth_roi` mode. Other `source_mode` values currently fall back to that mode.
- If `camera_info` is missing, the target-point estimator falls back to `fx=600`, `fy=600`, `cx=640`, `cy=360`, which is only a rough default.
- Detector launch defaults assume a TensorRT model with bindings:
  - input binding: `images`
  - output binding: `output0`
  If the exported engine uses different names, detection will fail until the launch args are overridden.
- Detector preprocessing assumes RGB8 input on `/oak/rgb/image_raw_rgb8` and a 640x640 network input. The mask-resize path assumes a 1280x720 output image.
- Current detector launch defaults assume:
  - ONNX model at `isaac_ros_assets/models/seaclear/v12s.onnx`
  - TensorRT engine output path `isaac_ros_assets/models/seaclear/v12s.plan`
- `v12s.plan` is not expected to be committed in the repo. On a new Jetson / GPU runtime, TensorRT may generate it from the ONNX model on first launch.
- If the ONNX file is missing or unresolved, detector runtime will fail.
- On first engine generation, detector launch may stay alive while inference is not yet ready for roughly 2-3 minutes.
- Some Jetson/container workflows in this repository assume FastDDS Discovery Server rather than simple multicast discovery:
  - `my_ws/scripts/launch_container.sh` passes `FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/oakd_isaac_ros/config/fastdds_client_profile.xml`
  - `config/fastdds_client_profile.xml` currently points at Discovery Server IP `192.168.2.95` on port `11815`
  - for BlueROV2 tests, verify that this IP still matches the intended server host; if discovery is hosted elsewhere, update the XML before launching the container

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

If you are validating detector runtime on Jetson, make sure you are actually building and launching inside the container. If `isaac_ros_tensor_rt` is missing from the runtime environment, try:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install -y ros-humble-isaac-ros-tensor-rt
```

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

If you need to build more than the five MVP packages, or the whole workspace on Jetson, and hit a gtest / Google Test related CMake failure inside the container, use:

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

This workaround came from a colleague who successfully built the latest `dnn_stereo_depth` end-to-end on Jetson.

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

## 8) Reproducible rosbag workflow

Use this section for replayable perception-to-controller checks. There are two modes:
- with hardware: record a short bag from the live OAK-D / detector pipeline
- without hardware: replay an existing bag and verify the visual-servo path from bagged detections/depth

Important scope note:
- `record_perception_bag.sh` requires a live publisher on the recorded topics, so it is not a no-hardware step by itself
- the actual no-hardware workflow starts from bag replay

### 8.0) Record a short bag from a live perception session (optional, requires hardware)

If you already have a live OAK-D / detector pipeline running and want a reproducible bag for later no-hardware replay:

```bash
cd /workspaces/oakd_isaac_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/record_perception_bag.sh /tmp/sc2_perception_bag 60
```

This records:
- `/detections_output`
- `/oak/stereo/image_raw`
- `/oak/stereo/camera_info`
- `/oak/rgb/image_raw`
- `/oak/rgb/image_raw_rgb8`
- `/oak/rgb/camera_info`
- `/tf`
- `/tf_static`

### 8.1) Replay an existing bag (no hardware path starts here)

Terminal 1: replay the bag with ROS time enabled.

```bash
cd /workspaces/oakd_isaac_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/play_perception_bag.sh /tmp/sc2_perception_bag 1.0
```

Terminal 2: run the visual-servo stack against the replayed perception topics.

```bash
cd /workspaces/oakd_isaac_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py \
  use_stub_target:=false \
  use_sim_time:=true
```

Why `use_sim_time:=true` matters:
- `play_perception_bag.sh` uses `ros2 bag play --clock`
- without simulated time, timeout-sensitive nodes can behave as if no valid replay time is advancing

Observe outputs:

```bash
ros2 topic echo /sc2/visual_servo/target_obs --once
ros2 topic hz /sc2/visual_servo/target_obs
ros2 topic hz /sc2/visual_servo/cmd_vel_body
ros2 topic hz /sc2/visual_servo/debug_markers
ls -l /tmp/sc2_visual_servo_mvp.csv
```

Useful preflight checks for the replayed bag:

```bash
ros2 bag info /tmp/sc2_perception_bag
ros2 topic info /detections_output
ros2 topic info /oak/stereo/image_raw
ros2 topic info /oak/stereo/camera_info
```

Expected behavior during successful replay:
- `target_observer_node` consumes `/detections_output` and `/oak/stereo/image_raw`
- `/sc2/visual_servo/target_obs` is republished from bagged detections/depth
- `/sc2/visual_servo/cmd_vel_body` is produced by the controller
- `/tmp/sc2_visual_servo_mvp.csv` is updated by `csv_logger_node`

Visualization note:
- `visual_servo_debug_viz_node` is enabled by default in `mvp_sim.launch.py` and publishes:
  - `/sc2/visual_servo/debug_markers` (`visualization_msgs/msg/MarkerArray`)
- This can be viewed in RViz2 or Foxglove.

## 8.2) `oakd_system.launch.py` smoke checks

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
- existence/type of `isaac_ros_assets/models/seaclear/v12s.onnx`
- detection-related launch files currently present under `src/oakd_isaac_ros/launch/`

Jetson integrated launch example for this branch:

```bash
cd /workspaces/oakd_isaac_ros
ros2 launch oakd_isaac_ros oakd_system.launch.py \
  detector_model_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.onnx \
  detector_engine_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan
```

If `/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan` does not exist yet, that is acceptable. On first run, TensorRT may generate it for the current hardware. Allow a few minutes before treating the detector path as failed.

Concrete first-run pattern when `v12s.plan` is absent:

```bash
cd /workspaces/oakd_isaac_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan

ros2 launch oakd_isaac_ros oakd_system.launch.py \
  run_oakd:=true \
  enable_detection:=true \
  enable_nvblox:=false \
  enable_visualization:=false \
  enable_visual_servo:=true \
  enable_target_point_estimator:=true \
  enable_debug_viz:=true \
  enable_mavros_bridge:=false \
  controller_mode:=bbox_target_point \
  detector_model_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.onnx \
  detector_engine_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan
```

Then, in a second terminal inside the same container:

```bash
cd /workspaces/oakd_isaac_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

watch -n 5 'ls -lh /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan || true'
```

Also watch the launch terminal for TensorRT completion / engine-build messages. On first hardware-specific generation, waiting 2-3 minutes is normal.

If the launch stays alive but `/detections_output` never appears after that wait, check:

```bash
ros2 topic info -v /detections_output
ros2 node list | sort | grep -E 'tensor|yolo|detection'
ls -lh /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan
```

Before live Jetson / BlueROV2 tests, also verify the DDS discovery profile that the container inherits:

```bash
echo "$FASTRTPS_DEFAULT_PROFILES_FILE"
sed -n '1,80p' /workspaces/oakd_isaac_ros/config/fastdds_client_profile.xml
```

If the Discovery Server is not running on the grapple PC at `192.168.2.95:11815`, update `config/fastdds_client_profile.xml` to the correct host before starting the container.

## 8.3) Jetson integrated verification after pull/rebase (no BlueROV)

Use this when the latest code is already pulled onto the Jetson and you want to answer three questions:
- does the rebased code still build on Jetson?
- does `oakd_system.launch.py` stay up with the real OAK-D?
- do camera image -> detector -> visual-servo controller topics reach `/sc2/visual_servo/cmd_vel_body` without requiring BlueROV hardware?

Goal for this check:
- OAK-D RGB image is published
- detector publishes `/detections_output`
- stereo depth / camera info are available for ROI depth
- integrated visual-servo stack publishes `/sc2/visual_servo/target_obs`, optionally `/sc2/visual_servo/target_point`, and `/sc2/visual_servo/cmd_vel_body`

This is not a vehicle actuation test. Keep:
- `enable_mavros_bridge:=false`
- no BlueROV requirement
- focus on perception + controller output only

### 8.3.1 Start the Jetson container with the expected DDS settings

On the Jetson host:

```bash
cd /mnt/orin_ssd/workspaces/oakd_isaac_ros
./my_ws/scripts/launch_container.sh
```

If this stops before the container starts because `launch_container.sh` reports unrelated missing Git LFS payloads, rerun:

```bash
cd /mnt/orin_ssd/workspaces/oakd_isaac_ros
SKIP_LFS_CHECK=1 ./my_ws/scripts/launch_container.sh
```

This only bypasses the preflight completeness gate. It does not fix the missing LFS objects themselves, so continue only after you verify the detector model you need is actually present inside the container.

Inside the container, verify the expected FastDDS-related environment:

```bash
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}"
sed -n '1,80p' /workspaces/oakd_isaac_ros/config/fastdds_client_profile.xml
```

Expected baseline for the current Jetson workflow:
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` or unset but effectively FastDDS from the Isaac ROS image
- `ROS_DOMAIN_ID=8`
- `ROS_LOCALHOST_ONLY=0`
- `FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/oakd_isaac_ros/config/fastdds_client_profile.xml`

If the Discovery Server host for the current bench setup is not `192.168.2.95:11815`, update `config/fastdds_client_profile.xml` before continuing.

### 8.3.2 Bootstrap and build

Inside the container:

```bash
cd /workspaces/oakd_isaac_ros
./scripts/ensure_dev_container_ready.sh
```

Build the integrated OAK-D + visual-servo packages first:

```bash
./scripts/colcon_build_venv.sh --packages-up-to \
  oakd_isaac_ros \
  sc2_visual_servo_interfaces \
  sc2_visual_servo_perception \
  sc2_visual_servo_controller \
  sc2_visual_servo_bringup
source install/setup.bash
```

If you want to validate a larger Jetson build and hit a gtest / Google Test CMake failure inside the container, retry with:

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
source install/setup.bash
```

If the container image itself failed to build earlier with:

```text
E: Package 'libcudnn9' has no installation candidate
```

you are using a Dockerfile revision that still asks apt for the old virtual package name. Update the workspace to a revision where `src/isaac_ros_common/docker/Dockerfile.aarch64` installs `libcudnn9-cuda-12`, then rebuild the container before continuing.

Verify that the detector assets are present and not obvious broken pointers:

```bash
ls -lah /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear
file /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.onnx
```

`v12s.plan` does not need to exist before the first Jetson run. It is typically generated for the active hardware target when TensorRT initializes successfully.

If you want to force a clean first-run engine generation test, use:

```bash
rm -f /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan
ros2 launch oakd_isaac_ros oakd_system.launch.py \
  run_oakd:=true \
  enable_detection:=true \
  enable_nvblox:=false \
  enable_visualization:=false \
  enable_visual_servo:=true \
  enable_target_point_estimator:=true \
  enable_debug_viz:=true \
  enable_mavros_bridge:=false \
  controller_mode:=bbox_target_point \
  detector_model_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.onnx \
  detector_engine_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan
```

Keep the launch running for a few minutes before concluding failure, and verify whether `v12s.plan` appears on disk.

### 8.3.3 Run the scripted Jetson smoke check first

Inside the container:

```bash
cd /workspaces/oakd_isaac_ros
./scripts/run_oakd_system_check.sh --profile jetson_live --launch-seconds 20
```

This script:
- rebuilds the integrated packages unless `--skip-build` is used
- launches `oakd_system.launch.py`
- checks that the launch stays alive
- collects node / topic snapshots under `diagnoses_oakd_system/<timestamp>/`
- records topic info for:
  - `/detections_output`
  - `/sc2/visual_servo/target_obs`
  - `/sc2/visual_servo/target_point`
  - `/sc2/visual_servo/cmd_vel_body`

Inspect at minimum:

```bash
LATEST_LOGDIR="$(ls -dt diagnoses_oakd_system/* | head -n1)"
echo "$LATEST_LOGDIR"
cat "$LATEST_LOGDIR/perception_preflight.log"
cat "$LATEST_LOGDIR/topic_info_core.log"
tail -n 80 "$LATEST_LOGDIR/oakd_system_launch.log"
```

If this smoke check fails before the launch stays up, fix that first before trying the manual integrated run below.

### 8.3.4 Manual integrated launch for perception + controller output

For the first end-to-end Jetson check without BlueROV, prefer disabling subsystems that are not needed for the question at hand:
- keep OAK-D on
- keep detector on
- keep visual-servo on
- keep target-point estimator on
- disable MAVROS bridge
- disable nvblox unless you specifically want its side path during this check

Recommended command:

```bash
cd /workspaces/oakd_isaac_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch oakd_isaac_ros oakd_system.launch.py \
  run_oakd:=true \
  enable_detection:=true \
  enable_nvblox:=false \
  enable_visualization:=false \
  enable_visual_servo:=true \
  enable_target_point_estimator:=true \
  enable_debug_viz:=true \
  enable_mavros_bridge:=false \
  controller_mode:=bbox_target_point \
  detector_model_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.onnx \
  detector_engine_file_path:=/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan
```

Why this launch shape:
- `oakd_system.launch.py` is the integrated entry point for the current branch
- it reuses `debug/object_detection.launch.py` for camera + detector wiring
- it overrides the camera params to `config/oak_d/oakd_rgbd.yaml`, so stereo depth remains available for target-point estimation
- `enable_nvblox:=false` keeps the first Jetson validation focused on the detector -> controller path instead of the mapping side path
- if `v12s.plan` is absent, TensorRT may generate it on first run; keep the launch alive for a few minutes before concluding that the detector failed

### 8.3.5 Runtime checks while the launch is up

Open a second terminal in the same container and source the workspace:

```bash
cd /workspaces/oakd_isaac_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Check that the expected nodes are present:

```bash
ros2 node list | sort | grep -E 'oak|tensor|yolo|target|visual_servo|image_converter'
```

Check that the camera and detector side are alive:

```bash
ros2 topic list | sort | grep -E '^/oak|^/detections_output|^/camera0/detection'
ros2 topic info -v /detections_output
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/stereo/image_raw
ros2 topic hz /detections_output
```

Check the visual-servo outputs:

```bash
ros2 topic info -v /sc2/visual_servo/target_obs
ros2 topic info -v /sc2/visual_servo/target_point
ros2 topic info -v /sc2/visual_servo/cmd_vel_body
ros2 topic hz /sc2/visual_servo/target_obs
ros2 topic hz /sc2/visual_servo/target_point
ros2 topic hz /sc2/visual_servo/cmd_vel_body
ros2 topic echo /sc2/visual_servo/status --once
ros2 topic echo /sc2/visual_servo/cmd_vel_body --once
```

Useful sanity checks for the image/depth geometry assumptions:

```bash
ros2 topic echo /oak/stereo/camera_info --once
ros2 topic echo /sc2/visual_servo/target_obs --once
ros2 topic echo /sc2/visual_servo/target_point --once
```

### 8.3.6 Pass criteria for this Jetson check

Treat the Jetson validation as successful if all of the following are true:
- the integrated launch stays alive without immediate crash
- `/oak/rgb/image_raw` is publishing
- if `v12s.plan` was initially absent, it is created or TensorRT clearly reports successful engine generation for the current hardware
- `/detections_output` has a live publisher and non-empty messages when the target is visible
- `/sc2/visual_servo/target_obs` is being published from real detections
- `/sc2/visual_servo/cmd_vel_body` is being published by the controller
- if `controller_mode:=bbox_target_point`, `/sc2/visual_servo/target_point` also publishes when depth ROI is valid

### 8.3.7 If the detector path is unstable but you still need to prove controller output

The primary goal above is real camera -> detector -> controller. If detector bringup is the only failing piece, you can still isolate the controller path by manually publishing `TargetObservation` while `oakd_system.launch.py` is running:

```bash
ros2 topic pub /sc2/visual_servo/target_obs sc2_visual_servo_interfaces/msg/TargetObservation \
"{header: {frame_id: 'oak_rgb_camera_optical_frame'}, track_id: 1, u: 920.0, v: 520.0, w: 90.0, h: 70.0, depth_m: 2.0, confidence: 0.95, keypoints_uv: [], source: 'manual'}" --once
ros2 topic echo /sc2/visual_servo/cmd_vel_body --once
```

Use this only as a fallback diagnostic. For the main Jetson validation, prefer proving that `/sc2/visual_servo/target_obs` comes from live detections.

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
- The same mismatch can also happen inside the container if a stale `.venv` with `numpy>=2` is reused against ROS apt packages that were built against NumPy 1.x.
- Supported workflow: run ROS2 commands and `cv_bridge`-dependent nodes inside the Humble container, and use `./scripts/ensure_dev_container_ready.sh` to validate or rebuild `.venv` before launching.
- If host-side Python execution is unavoidable, NumPy alignment with host ROS apt packages may be required, but host Python setup is out of scope for this project workflow.

## Troubleshooting

- `No module named rclpy._rclpy_pybind11`:
  interpreter mismatch. Recreate venv with Python 3.10 and `--system-site-packages`:
  `rm -rf .venv && ./scripts/uv_bootstrap.sh`
- `cv_bridge` import errors mentioning NumPy 2:
  if you are already inside the Humble container, force a clean venv rebuild:
  `./scripts/ensure_dev_container_ready.sh --force-bootstrap`
- `./scripts/ensure_dev_container_ready.sh --force-bootstrap` fails with `Exec format error` from `.tools/uv/uv`:
  the workspace-local `uv` binary was created on a different CPU architecture. Remove it and rerun:
  `rm -f .tools/uv/uv && ./scripts/ensure_dev_container_ready.sh --force-bootstrap`
- Avoid installing `opencv-python` into this venv unless explicitly required; it can conflict with system OpenCV / `cv_bridge` bindings.
