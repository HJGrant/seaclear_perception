# OAK-D PoE + Isaac ROS Visual SLAM Debug Notes

## Scope
This note tracks **air setup** debugging for OAK-D-PoE + Isaac ROS Visual SLAM.
Underwater cable/system discussions are intentionally out-of-scope for this stage.

## 1) Parameter integrity check (implementation-backed)

The following checks were done against `depthai_ros_driver` parameter declarations, not by guesswork.

### Definite invalid keys in current `oakd_rgbd.yaml`

- `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml:20` `left.i_low_bandiwdth_frame_freq`
  - Typo; valid key is `i_low_bandwidth_frame_freq`.
  - Reference: `src/depthai-ros/depthai_ros_driver/src/param_handlers/sensor_param_handler.cpp:26`
- `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml:29` `right.i_low_bandiwdth_frame_freq`
  - Typo; valid key is `i_low_bandwidth_frame_freq`.
  - Reference: `src/depthai-ros/depthai_ros_driver/src/param_handlers/sensor_param_handler.cpp:26`
- `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml:37` `rgb.i_low_bandiwdth_frame_freq`
  - Typo; valid key is `i_low_bandwidth_frame_freq`.
  - Reference: `src/depthai-ros/depthai_ros_driver/src/param_handlers/sensor_param_handler.cpp:26`
- `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml:69` `stereo.i_publish_left_rect`
  - Invalid in current driver; valid key is `i_left_rect_publish_topic`.
  - Reference: `src/depthai-ros/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp:78`
- `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml:70` `stereo.i_publish_right_rect`
  - Invalid in current driver; valid key is `i_right_rect_publish_topic`.
  - Reference: `src/depthai-ros/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp:91`
- `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml:71` `stereo.i_rectify`
  - Not declared in current driver.
  - Rectified outputs are controlled by `i_left_rect_publish_topic` / `i_right_rect_publish_topic`.
  - References: `src/depthai-ros/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp:78`, `src/depthai-ros/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp:91`, `src/depthai-ros/depthai_ros_driver/src/dai_nodes/sensors/stereo.cpp:101`
- `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml:72` `stereo.i_publish_depth`
  - Not declared in current driver.
  - Depth/disparity publish is controlled via `stereo.i_publish_topic` (+ optional `i_output_disparity`).
  - References: `src/depthai-ros/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp:66`, `src/depthai-ros/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp:69`
- `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml:46` `rgb.i_enable`
  - Not declared in current sensor params.
  - Use `i_publish_topic` and/or `i_disable_node` for behavior control.
  - References: `src/depthai-ros/depthai_ros_driver/src/param_handlers/sensor_param_handler.cpp:33`, `src/depthai-ros/depthai_ros_driver/src/param_handlers/sensor_param_handler.cpp:114`
- `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml:15` `pipeline_gen.i_enable_rgbd`
  - Not declared in current pipeline-gen handler.
  - References: `src/depthai-ros/depthai_ros_driver/src/param_handlers/pipeline_gen_param_handler.cpp:12`, `src/depthai-ros/depthai_ros_driver/src/param_handlers/pipeline_gen_param_handler.cpp:13`, `src/depthai-ros/depthai_ros_driver/src/param_handlers/pipeline_gen_param_handler.cpp:14`

### Keys confirmed valid (examples from current file)

- `camera.i_pipeline_type` valid: `src/depthai-ros/depthai_ros_driver/src/param_handlers/camera_param_handler.cpp:25`
- `stereo.i_publish_synced_rect_pair` valid: `src/depthai-ros/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp:77`
- `stereo.i_stereo_conf_threshold` valid: `src/depthai-ros/depthai_ros_driver/src/param_handlers/stereo_param_handler.cpp:141`
- `left/right.i_resolution` valid: `src/depthai-ros/depthai_ros_driver/src/param_handlers/sensor_param_handler.cpp:63`

## 2) Staged YAMLs (A-E)

New files created under `src/oakd_isaac_ros/config/oak_d/`:

- A: `oakd_stereo_only_30hz.yaml`
  - Raw left/right only, pipeline `Stereo`, RGB disabled.
- B: `oakd_stereo_rect_30hz.yaml`
  - Rectified left/right enabled (`i_left_rect_publish_topic`, `i_right_rect_publish_topic`), synced rect pair disabled.
- C: `oakd_stereo_rect_synced_30hz.yaml`
  - Same as B + `i_publish_synced_rect_pair=true`.
- D: `oakd_rgb_only_30hz.yaml`
  - RGB-only baseline at 30 Hz.
- E: `oakd_rgb_plus_stereo_30hz.yaml`
  - RGB + stereo raw coexistence baseline.

Notes:
- A uses `camera.i_pipeline_type=Stereo` (left/right sensors only).
- B/C use `camera.i_pipeline_type=Depth` because rectified streams are produced by the StereoDepth node path.

## 3) Launch updates for reproducibility

Updated `src/oakd_isaac_ros/launch/oakd_visual_slam.launch.py`:

- Added launch argument `params_file` (switch among A-E without file edits).
- Added launch argument `enable_vslam` (turn Visual SLAM + republish nodes on/off).
- Camera node is always loaded; VisualSlamNode is conditionally loaded via `LoadComposableNodes`.

Examples:

```bash
# Camera-only run (for transport isolation)
ros2 launch oakd_isaac_ros oakd_visual_slam.launch.py \
  params_file:=src/oakd_isaac_ros/config/oak_d/oakd_stereo_only_30hz.yaml \
  enable_vslam:=false

# Camera + VSLAM run
ros2 launch oakd_isaac_ros oakd_visual_slam.launch.py \
  params_file:=src/oakd_isaac_ros/config/oak_d/oakd_rgb_plus_stereo_30hz.yaml \
  enable_vslam:=true
```

## 4) One-command diagnostics run

Script: `src/oakd_isaac_ros/scripts/diag_oakd_run_and_monitor.sh`

What it logs:

- `ros2 launch ...`
- `ros2 topic hz` for left/right raw, camera_info, rect topics
- `ros2 topic bw` for left/right raw
- `dmesg -w`
- `ip -s link show <iface>` every 1s
- `ethtool -S <iface>` filtered counters every 1s
- Optional `docker stats`
- Triggered snapshots on `X_LINK_ERROR`, `ping was missed`, `No Data`, `double free`, `bad_alloc`

Usage example:

```bash
src/oakd_isaac_ros/scripts/diag_oakd_run_and_monitor.sh \
  --params-file src/oakd_isaac_ros/config/oak_d/oakd_stereo_rect_synced_30hz.yaml \
  --enable-vslam false \
  --iface eno1 \
  --duration-sec 600
```

Log layout:

- `logs/oakd_diag/<timestamp>/launch.log`
- `logs/oakd_diag/<timestamp>/hz_*.log`, `bw_*.log`
- `logs/oakd_diag/<timestamp>/ip_link_stats.log`
- `logs/oakd_diag/<timestamp>/ethtool_stats.log`
- `logs/oakd_diag/<timestamp>/dmesg_follow.log`
- `logs/oakd_diag/<timestamp>/events.log`
- `logs/oakd_diag/<timestamp>/snapshots/*.log`

## 5) ROS 2 daemon/discovery recovery playbook

### Quick health check

```bash
ros2 daemon status
ros2 node list
ros2 topic list
ros2 node list --no-daemon
ros2 topic list --no-daemon
```

If `topic list` works but `node list` does not, compare daemon vs no-daemon behavior.

### Recovery

```bash
ros2 daemon stop
ros2 daemon start
ros2 daemon status
```

If still broken:

```bash
rm -rf ~/.ros/daemon
ros2 daemon start
```

Warning: deleting `~/.ros/daemon` clears daemon cache/state.

### Preflight consistency (container vs host)

Check both sides (inside container and host shell):

```bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
```

Mismatch in `ROS_DOMAIN_ID` or `RMW_IMPLEMENTATION` can break discovery and invalidate measurements.

## 6) Isaac ROS Visual SLAM requirements (release-3.2 reference)

Primary source (user-requested):
- https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#quickstart

Relevant requirements from the same release docs:

- VisualSlamNode subscribes to stereo image + camera_info pairs (`visual_slam/image_0`, `camera_info_0`, `image_1`, `camera_info_1`).
- Camera system guidance for acceptable tracking quality recommends synchronized stereo and around 30 Hz camera frame rate.

Additional release-3.2 page used for explicit camera guidance:
- https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/visual_slam/cuvslam/tutorial_realsense.html#camera-system-requirements

Why this matters here:
- If OAK-D transport/driver cannot sustain near-30Hz stable stereo delivery, SLAM failures can be caused by input starvation before SLAM algorithm tuning becomes relevant.

## 7) Experiment matrix to run now

Run with `enable_vslam=false` first, then `true`:

1. A (`oakd_stereo_only_30hz.yaml`): verify raw stereo transport stability.
2. B (`oakd_stereo_rect_30hz.yaml`): add rectified left/right.
3. C (`oakd_stereo_rect_synced_30hz.yaml`): isolate synced-rect-pair path.
4. D (`oakd_rgb_only_30hz.yaml`): RGB-only baseline.
5. E (`oakd_rgb_plus_stereo_30hz.yaml`): coexistence stress case.

For each run, record:

- Stable runtime duration
- left/right Hz and bandwidth
- Error presence (`X_LINK_ERROR`, `No Data`, crash signatures)
- NIC counters and kernel logs at/near error time

## 8) Current focused hypotheses

- Invalid/ignored YAML keys masked intended behavior (rect outputs not really enabled, bandwidth params ignored).
- `i_publish_synced_rect_pair=true` path may trigger a driver/runtime bug under some timing/load combinations.
- PoE/NIC transport instability (`ping was missed`, `X_LINK_ERROR`) is an independent failure mode that can stop streams even with valid params.

## 9) Upstream issue preparation

If C (or a smaller diff) reproduces `double free`/`bad_alloc`, use:
- `docs/oakd-vslam/upstream_issue_template_depthai_ros_driver.md`

Attach:
- minimal YAML,
- launch command,
- exact commit hashes,
- snapshot logs from `logs/oakd_diag/<run>/snapshots/`.

## 10) Historical repository-local modifications to keep in mind

These are not necessarily the root cause of the current detector / TensorRT failures, but they are repo-local deviations from `main` that should be remembered during future debugging.

### 10.1 `oakd_nvblox.launch.py`: Foxglove compressed video block is commented out

- File: `src/oakd_isaac_ros/launch/oakd_nvblox.launch.py:119`
- Current state:
  - The block that launches the Foxglove compressed video streaming node is wrapped in a triple-quoted comment.
  - The commented block also shows a package-name substitution from `foxglove_compressed_video` to `foxglove_compressed_video_transport`.
- Practical effect:
  - `oakd_nvblox.launch.py` no longer starts that compressed-video helper.
  - Foxglove users may still see standard image topics, but should not assume that the dedicated compressed-video stream is active.
  - If a future debugging session expects `/oak/compressed_video` or related transport behavior, first confirm whether this commented block is intentional or should be restored.
- Scope note:
  - This change is outside the visual-servo MVP core path, but it can affect monitoring workflows and performance assumptions during OAK-D bringup.

### 10.2 `Dockerfile.oakd`: apt update workaround was added

- File: `my_ws/my_docker/Dockerfile.oakd:10`
- Current state:
  - The original plain `apt update && apt-get upgrade -y` step was replaced with:
    - `rm -rf /var/lib/apt/lists/*`
    - a `sed` command targeting `isaac.download.nvidia.com`
    - `apt update && apt-get upgrade -y`
  - The older line remains as a comment.
- Practical effect:
  - Container builds are no longer using the exact same apt bootstrap sequence as `main`.
  - This workaround may be necessary for a specific mirror/index condition, but it also means image reproducibility differs from upstream assumptions.
  - When debugging package availability issues, do not assume the container image matches a stock Isaac ROS dev image.
