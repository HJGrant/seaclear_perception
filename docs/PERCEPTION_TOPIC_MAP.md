# SC2 Visual Servo Perception Topic Map

This map documents the practical perception topics used by visual-servo MVP (`use_stub_target:=false`).

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
| `/detections_output` | `vision_msgs/msg/Detection2DArray` | `src/oakd_isaac_ros/launch/tools/detection.launch.py` (YOLOv8 decoder) | Usually RGB optical frame (`oak_rgb_camera_optical_frame` or equivalent) | Typically lower than camera FPS; model-dependent (often ~5-30 Hz) | Not guaranteed hard-sync with depth by itself |
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

## Jetson observations (2026-02-23)

Verified:
- `/oak/stereo/image_raw` reported `sensor_msgs/msg/Image` with `encoding: 16UC1`.
- Observed depth value distribution included median around `1609` (mm-scale interpretation: about `1.6 m`).
- Observed max often reached `65535` (sentinel-like value in `16UC1` stream).

Suspected:
- In one observed session, `/detections_output` existed but had publisher count `0`; subscriber side was present.
- ONNX/TensorRT startup issues likely explain missing stable detection publisher in that session.

Deferred:
- Final detection topic mapping/remaps are pending colleague confirmation.
- Topic names can vary by launch profile and container composition; treat detection topic as `TBD` until confirmed by the team-maintained launch.
