# SC2 Visual Servo Perception Topic Map

Scope: active topic contract for `target_observer_node` and related perception inputs.
Source of truth: use this file when wiring detections, depth, or bag replay into the MVP pipeline.
Last updated: 2026-03-23
Supersedes: topic snippets previously spread across status/debug notes.

This map documents the practical perception topics used by visual-servo MVP (`use_stub_target:=false`).

Current launch entry points in this tree:
- integrated entry: `src/oakd_isaac_ros/launch/oakd_system.launch.py`
- detector-only wrapper: `src/oakd_isaac_ros/launch/debug/object_detection.launch.py`
- detector core: `src/oakd_isaac_ros/launch/tools/detection.launch.py`

## Launch role split

- `src/oakd_isaac_ros/launch/oakd_system.launch.py`
  - full integrated path
  - camera + detector + target observer + target-point estimator + controller + optional MAVROS bridge
  - uses `src/oakd_isaac_ros/config/oak_d/oakd_rgbd.yaml`
- `src/oakd_isaac_ros/launch/debug/object_detection.launch.py`
  - detector-only path
  - camera + detector debugging
  - uses `src/oakd_isaac_ros/config/oak_d/oakd_dnn_depth.yaml`
- `src/oakd_isaac_ros/launch/tools/detection.launch.py`
  - detector core only
  - expects camera / image conversion topics to already exist

## How to inspect on your system

```bash
ros2 topic list
ros2 topic info /detections_output
ros2 topic info /oak/stereo/image_raw
ros2 topic info /oak/rgb/image_raw
ros2 interface show vision_msgs/msg/Detection2DArray
ros2 topic hz /detections_output
ros2 topic hz /oak/stereo/image_raw
ros2 topic hz /oak/rgb/image_raw
```

## Host vs Container Python note

- Supported workflow is to run ROS2 topic inspection and cv_bridge-dependent nodes inside the Humble container.
- If similar commands are run from host-side Python, NumPy/cv_bridge ABI mismatch may appear depending on host package versions.
- Host Python package alignment is out of scope for the supported project workflow.

## Topic map

| Topic | Message type | Producer path | frame_id convention | Approx frequency | Sync status |
|---|---|---|---|---|---|
| `/detections_output` | `vision_msgs/msg/Detection2DArray` | `src/oakd_isaac_ros/launch/tools/detection.launch.py` (detector decode path in current tree) | Usually RGB optical frame (`oak_rgb_camera_optical_frame` or equivalent) | Typically lower than camera FPS; model-dependent (often ~5-30 Hz) | Not guaranteed hard-sync with depth by itself |
| `/oak/rgb/image_raw` | `sensor_msgs/msg/Image` | OAK-D camera pipeline (`launch/tools/camera.launch.py`) | RGB camera optical frame | Commonly ~30 Hz with current configs | Camera-native timestamp |
| `/oak/rgb/image_raw_rgb8` | `sensor_msgs/msg/Image` | `src/oakd_isaac_ros/scripts/image_converter.py` | Same as RGB frame | Typically same as `/oak/rgb/image_raw` | Derived from RGB |
| `/oak/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | OAK-D camera pipeline | RGB camera frame | Matches RGB image stream | Camera-native timestamp |
| `/oak/stereo/image_raw_depth` | `sensor_msgs/msg/Image` | OAK-D depth source subscribed by `publish_mask` | Stereo/depth optical frame | Camera/depth dependent | Camera-native timestamp |
| `/oak/stereo/image_raw` | `sensor_msgs/msg/Image` | Republished by `src/oakd_isaac_ros/src/publish_mask.cpp` | Depth camera frame (`/oak/stereo/camera_info`) | Follows depth source | Timestamp overwritten to RGB stamp in `publish_mask.cpp` |
| `/oak/stereo/camera_info` | `sensor_msgs/msg/CameraInfo` | OAK-D + republish path in `publish_mask.cpp` | Stereo/depth frame | Matches depth stream | Republished with RGB stamp in `publish_mask.cpp` |

## Notes for visual-servo observer

Default observer input parameters:
- `detections_topic:=/detections_output`
- `depth_topic:=/oak/stereo/image_raw`
- `source:=yolo_detection2d`
- `pick_strategy:=highest_confidence`
- `max_age_ms:=300`
- `timeout_ms:=500`
- `depth_min_m:=0.2`
- `depth_max_m:=20.0`

Observer-side implicit assumptions:
- `/detections_output` is `vision_msgs/msg/Detection2DArray`.
- The observer selects one detection after class/confidence filtering; it is not a multi-target tracker.
- If depth is enabled and the ROI depth is invalid, the observer still publishes `TargetObservation` with `depth_m=NaN`.
- Stale detections older than `max_age_ms` are dropped before they reach the controller side.

Recommended checks before running `use_stub_target:=false`:

```bash
ros2 topic echo /detections_output --once
ros2 topic echo /oak/stereo/image_raw --once
ros2 topic hz /detections_output
ros2 topic hz /oak/stereo/image_raw
```

If `/oak/stereo/image_raw` is missing, either:
- run the detection pipeline path that includes `publish_mask.cpp` republish, or
- remap observer `depth_topic` to the available depth topic.

## Target-point estimator assumptions

Current integrated launch feeds the controller through `target_point_estimator_node`, which assumes:

- `source_mode:=bbox_depth_roi`
  - other modes are not implemented and fall back to `bbox_depth_roi`
- default inputs:
  - target obs: `/sc2/visual_servo/target_obs`
  - depth: `/oak/stereo/image_raw`
  - camera info: `/oak/stereo/camera_info`
  - mask: `/camera0/detection/mask`
- default filters:
  - `min_valid_points:=20`
  - `num_nearest_points:=25`
  - `max_search_radius_px:=40.0`
  - `use_mask_gating:=false`
  - `hold_last_valid:=true`
  - `max_hold_ms:=500.0`
- if `camera_info` is unavailable, fallback intrinsics are:
  - `fx=600`, `fy=600`, `cx=640`, `cy=360`
- default output frame falls back to `oak_rgb_camera_optical_frame`

This means the integrated path is implicitly tuned to a 1280x720 RGB view and a stereo-depth ROI backprojection path.

## Camera intrinsic / extrinsic assumptions

These are easy to miss, but they are part of the current integrated path.

Intrinsic assumptions:

- The target-point estimator backprojects image coordinates using `camera_info` from `/oak/stereo/camera_info`.
- If `camera_info` is unavailable, it silently falls back to:
  - `fx=600`
  - `fy=600`
  - `cx=640`
  - `cy=360`
- The controller is also tuned around a 1280x720 image with image center `(640, 360)`.
- In practice, this means the current integrated setup implicitly assumes:
  - a 1280x720-like image geometry
  - principal point near the image center
  - intrinsics that are reasonably close to the fallback values if `camera_info` is missing

Extrinsic assumptions:

- `src/oakd_isaac_ros/launch/oakd_system.launch.py` publishes static TFs:
  - `map -> odom` as identity
  - `odom -> oak-d-base-frame` with translation `(0, 0, 1.0)` and rotation `(roll=0, pitch=3.14, yaw=1.5708)`
  - `oak-d-base-frame -> oak_rgb_camera_optical_frame` as identity
- These transforms are operational placeholders, not a calibrated vehicle/camera extrinsic model.
- Therefore, any downstream use of target pose, target point, or MAVROS/body-frame command interpretation implicitly assumes those static transforms are acceptable for the current bench or dry-run setup.
- For real vehicle deployment, these TFs should be treated as items to verify, not as proven calibration.

Practical implication:

- If the OAK-D calibration, mounted camera pose, or frame conventions differ from the above assumptions, the detector may still run, but the target-point estimate and controller behavior can become biased or misleading.

## Controller-side assumptions

Current integrated launch sets `controller_mode:=bbox_target_point`. In practice this means:

- `/sc2/visual_servo/target_point` is part of the intended path, even though `require_target_point` defaults to `false`
- controller image geometry defaults are:
  - `image_width:=1280`
  - `image_height:=720`
  - `target_u:=640`
  - `target_v:=360`
- distance control defaults are:
  - `desired_target_distance_m:=0.6`
  - `k_target_distance:=0.6`
  - `target_distance_mode:=z`
- timeout behavior defaults are:
  - observation timeout: `500 ms`
  - target-point timeout: `500 ms`
  - on timeout, controller publishes zero command
- command frame defaults to `body_frame_id:=base_link`

If your camera resolution, optical center, or distance convention differs, you should override these instead of relying on the defaults.

## Detector-core assumptions

`src/oakd_isaac_ros/launch/tools/detection.launch.py` currently assumes:

- RGB input topic: `/oak/rgb/image_raw_rgb8`
- RGB camera info: `/oak/rgb/camera_info`
- TensorRT input resolution: `640x640`
- TensorRT bindings:
  - input binding name: `images`
  - output binding name: `output0`
- decoder thresholds:
  - confidence: `0.6`
  - NMS: `0.45`
- resized mask output path assumes a 1280x720 image geometry

These are model/export-specific assumptions, not generic detector guarantees.

## Verified on Jetson (2026-02-23)

Status: Verified
Observed:
- Depth topic: `/oak/stereo/image_raw` (`sensor_msgs/msg/Image`)
- Encoding: `16UC1`
- Approx observed statistics: min `633`, median `1609`, max `65535`

Interpretation:
- Depth is interpreted as mm-scale in this session.
- `65535` behaves as an invalid/sentinel value and must be excluded from ROI depth statistics.

## Provisional detections mapping

Status: Deferred
Observed:
- Candidate detections topic in current graph: `/detections_output` (`vision_msgs/msg/Detection2DArray`).
- In problematic session, `ros2 topic info -v /detections_output` reported publisher count `0` (subscriber present: `tensor_rt_container`).

Operational guidance:
- Mapping/remap names remain provisional and require colleague confirmation.
- Do not block controller validation on detections availability; use stub/manual `TargetObservation` or rosbag path.
- Also verify that the configured detection model files exist:
  - `ls -lah /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear`
  - `file /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.onnx`
  - `file /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan`
- Team guidance indicates that the production detector is a teammate-maintained underwater model path rather than a generic off-the-shelf detector package.
- `src/oakd_isaac_ros/launch/debug/object_detection.launch.py` is present in the current tree and serves as the detector-only wrapper path.
- `src/oakd_isaac_ros/launch/oakd_system.launch.py` is the integrated detector + visual-servo entry in the current branch.
- Current integrated launch uses top-level overrides `detector_model_file_path` and `detector_engine_file_path` if the detector asset location must be changed from the defaults.

Colleague-confirmed mapping placeholder:
- Status: TBD
- Topic name: `TBD`
- Message type: `TBD`
- Remap source (launch path): `TBD`
