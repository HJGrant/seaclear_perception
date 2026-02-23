# DEBUG NOTES — Jetson Perception Bringup (2026-02-23)

Purpose: capture the recent investigation timeline for depth/detection topics and why detection-topic stabilization is currently deferred.

## Status labels

- Verified: directly observed with commands in this session window.
- Suspected: likely from logs/symptoms, but not yet fully reproduced under a stable run.
- Deferred: intentionally parked until colleague input clarifies launch/remap conventions.

## Host vs Container Python note

- NumPy/cv_bridge ABI mismatch was observed when Python/ROS commands were executed outside the Humble container.
- Supported workflow is to run ROS2 and cv_bridge-dependent commands inside the Humble container.
- Host-side Python environment alignment may be needed only for non-supported host execution paths.

## Chronological timeline (approx JST)

- 2026-02-23 09:00-10:30
  - Verified:
    - Humble container workflow validated (`uv` + venv + colcon build/test scripts).
    - Launch/runtime blockers fixed (libexec shim, nounset-safe env source, shutdown handling).
  - Notes:
    - WSL/container shell had empty topic graph until actual ROS launch was running.

- 2026-02-23 10:30-12:00
  - Verified:
    - MVP stub path worked end-to-end (`target_obs_publisher` to controller to logger/stub).
  - Notes:
    - Confirmed that topic checks must be done while relevant launch graph is active.

- 2026-02-23 13:00-15:00
  - Verified:
    - Jetson depth stream checks on `/oak/stereo/image_raw`.
    - `encoding` observed as `16UC1`.
    - Raw-buffer stats indicated median around `1609` and frequent max `65535`.
  - Suspected:
    - `65535` behaves like sentinel/out-of-range value for depth stream.

- 2026-02-23 15:00-18:00
  - Verified:
    - `/detections_output` type observed as `vision_msgs/msg/Detection2DArray`.
    - In problematic run, publisher count was `0` (subscriber side present).
  - Suspected:
    - Prior ONNX/TensorRT parse/engine creation errors correlate with no active detection publisher.
  - Deferred:
    - Final `detections_topic` mapping/remaps and stable inference bringup are pending colleague input.

## Commands that worked

Topic presence/type/publisher checks:

```bash
ros2 topic list
ros2 topic info -v /detections_output
ros2 topic info -v /oak/stereo/image_raw
ros2 node list | grep -E 'tensor|yolo|detect|oak|stereo'
```

Encoding check:

```bash
ros2 topic echo --once /oak/stereo/image_raw --field encoding
```

Depth quick stats (raw buffer parse; cv_bridge-independent path):

```bash
python3 - <<'PY'
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

class OneShot(Node):
    def __init__(self):
        super().__init__('depth_stats_oneshot')
        self.sub = self.create_subscription(Image, '/oak/stereo/image_raw', self.cb, 10)

    def cb(self, msg: Image):
        if msg.encoding != '16UC1':
            print(f'encoding={msg.encoding} (expected 16UC1 for this check)')
            rclpy.shutdown()
            return
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        valid = arr[(arr > 0) & (arr < 65535)]
        if valid.size == 0:
            print('no valid depth samples')
        else:
            print(f'min={int(valid.min())} median={float(np.median(valid)):.1f} max={int(valid.max())}')
        rclpy.shutdown()

rclpy.init()
node = OneShot()
rclpy.spin(node)
PY
```

Detection message type check:

```bash
ros2 interface show vision_msgs/msg/Detection2DArray
```

## Verified vs Suspected vs Deferred summary

Verified:
- `/oak/stereo/image_raw` is `16UC1` in observed Jetson run.
- Depth samples were mm-scale plausible (median around `1609`).
- `/detections_output` existed as `Detection2DArray` but had publisher count `0` in a problematic run.

Suspected:
- Detection publisher loss linked to ONNX/TensorRT engine parse/create failures.
- Topic naming/remap drift between launch profiles may hide active publisher path.

Deferred:
- Detection topic stabilization and final remap contract after colleague review.

## What to ask colleague next

- Which launch file is authoritative for production detection on Jetson?
- Which exact detection topic name should observer subscribe to in that launch profile?
- Are there nitros-only topics where ROS bridge topics are intentionally absent?
- Where are remaps set (file paths), and what are expected final names/types?
- Which ONNX + TensorRT workflow/version is known good on current image?

## Closure criteria

- `ros2 topic info -v <detections_topic>` shows publisher count `> 0` during run.
- `ros2 topic hz <detections_topic>` is stable non-zero.
- `target_observer_node` publishes `/sc2/visual_servo/target_obs` from real detections.
- Controller publishes `/sc2/visual_servo/cmd_vel_body` under `use_stub_target:=false`.
