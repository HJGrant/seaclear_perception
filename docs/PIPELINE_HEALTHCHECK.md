# SC2 Pipeline Healthcheck

Purpose: provide a repeatable diagnostic checklist for OAK-D pipeline health before using runs as VSLAM or perception-validation evidence.

Scope:
- Run commands inside the Humble container unless explicitly noted.
- Topic names are provisional and may require colleague-confirmed remaps.

## 1) Preconditions

Status: In Progress

- Start the target launch profile and keep it running while collecting diagnostics.
- Use container shell for ROS2 commands.
- If detections topic name is not confirmed, replace with candidate topic and record that it is provisional.

## 2) Core commands (copy/paste)

### 2.1 ROS graph and topic presence

```bash
ros2 node list
ros2 topic list
ros2 topic info -v /oak/stereo/image_raw
ros2 topic info -v /oak/rgb/image_raw
ros2 topic info -v /detections_output
```

### 2.2 Topic rate and continuity

```bash
ros2 topic hz /oak/stereo/image_raw
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /detections_output
```

### 2.3 Depth encoding and quick stats

```bash
ros2 topic echo --once /oak/stereo/image_raw --field encoding
```

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

### 2.4 Exposure/whiteout checks

```bash
ros2 topic echo --once /oak/rgb/image_raw --field header.frame_id
ros2 topic echo --once /oak/rgb/image_raw --field width
ros2 topic echo --once /oak/rgb/image_raw --field height
```

```bash
python3 - <<'PY'
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

class RGBSat(Node):
    def __init__(self):
        super().__init__('rgb_saturation_check')
        self.sub = self.create_subscription(Image, '/oak/rgb/image_raw', self.cb, 10)

    def cb(self, msg: Image):
        # This quick check assumes 8-bit image payload.
        data = np.frombuffer(msg.data, dtype=np.uint8)
        sat_ratio = float((data >= 250).sum()) / max(1, data.size)
        print(f'saturation_ratio={sat_ratio:.4f}')
        rclpy.shutdown()

rclpy.init()
node = RGBSat()
rclpy.spin(node)
PY
```

### 2.5 System and transport checks (Jetson host shell)

```bash
tegrastats
```

```bash
dmesg | tail -n 120
journalctl -n 200 --no-pager
```

```bash
ip -s link
```

### 2.6 TF availability checks

```bash
ros2 topic echo --once /tf
ros2 topic echo --once /tf_static
```

## 3) Acceptance criteria (text-only)

### 3.1 Exposure health gate

Status: TBD

- Saturation ratio from RGB check is not consistently high across representative scenes.
- Visual inspection no longer shows persistent whiteout dominating the frame.
- VSLAM/perception validation runs can proceed without immediate failure due to overexposure.

### 3.2 OAK-D stability gate

Status: TBD

- Bringup succeeds repeatedly without manual restart during trial window.
- Core topics (`/oak/rgb/image_raw`, `/oak/stereo/image_raw`) maintain expected publication rate.
- No sustained stream drops in `ros2 topic hz` and no repeated critical errors in system logs.

### 3.3 Detections readiness gate (provisional topic)

Status: Deferred

- Candidate detections topic shows active publisher during inference run.
- Publication rate is sustained and non-zero.
- `target_observer_node` publishes `/sc2/visual_servo/target_obs` from detections in non-stub mode.

## 4) How to interpret results

Observed:
- A single successful launch is not enough; require repeated stable runs before marking gate as Verified.

Hypothesis:
- Whiteout and OAK-D instability can jointly invalidate VSLAM/perception conclusions even when launch appears successful.

Decision rule:
- If exposure or stability gate fails, prioritize pipeline health fixes before VSLAM tuning.
- Keep controller verification unblocked by using stub/manual `TargetObservation` or rosbag replay.
